#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::panic::PanicInfo;
use log::{error, info};

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi imports - using esp-radio
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use esp_radio::wifi::{
    AccessPointInfo, ClientConfig, Config, ModeConfig, ScanConfig, WifiController,
};

// ESP32 HAL imports
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::logger::init_logger_from_env;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// Heap statistics function
fn report_heap_stats(context: &str) {
    let used = esp_alloc::HEAP.used();
    let free = esp_alloc::HEAP.free();
    info!(
        "[HEAP STATS] {}: Used: {} bytes, Free: {} bytes, Total: {} bytes",
        context,
        used,
        free,
        used + free
    );
}

// ESP32-S3-LCD-EV-Board hardware imports
use esp_hal::delay::Delay;
use esp_hal::dma::{CHUNK_SIZE, DmaDescriptor, DmaTxBuf};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode, Phase, Polarity,
        dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
    },
};

// Slint platform imports
use slint::PhysicalSize;
use slint::platform::software_renderer::Rgb565Pixel;

slint::include_modules!();

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// Display constants for ESP32-S3-LCD-EV-Board - 480x480 RGB display
const LCD_H_RES: u16 = 480;
const LCD_V_RES: u16 = 480;
const LCD_H_RES_USIZE: usize = 480;
const LCD_V_RES_USIZE: usize = 480;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;
const FRAME_BYTES: usize = LCD_BUFFER_SIZE * 2; // 2 bytes per RGB565 pixel
const NUM_DMA_DESC: usize = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

// Place DMA descriptors in DMA-capable RAM
#[unsafe(link_section = ".dma")]
static mut TX_DESCRIPTORS: [DmaDescriptor; NUM_DMA_DESC] = [DmaDescriptor::EMPTY; NUM_DMA_DESC];

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {}
}

/// FT5x06 Touch Controller for ESP32-S3-LCD-EV-Board
struct Ft5x06<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Ft5x06<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Reads the first touch point. Returns Some((x, y)) if touched, None otherwise.
    pub fn get_touch(&mut self) -> Result<Option<(u16, u16)>, I2C::Error> {
        // 1) read touch count from register 0x02
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[0x02], &mut buf)?;
        let count = buf[0] & 0x0F;
        if count == 0 {
            return Ok(None);
        }

        // 2) read first touch coordinates from regs 0x03..0x06
        let mut data = [0u8; 4];
        self.i2c.write_read(self.address, &[0x03], &mut data)?;
        let x = (((data[0] & 0x0F) as u16) << 8) | data[1] as u16;
        let y = (((data[2] & 0x0F) as u16) << 8) | data[3] as u16;

        Ok(Some((x, y)))
    }
}

/// TCA9554 I2C I/O Expander for display control
struct Tca9554 {
    i2c: esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>,
    address: u8,
}

impl Tca9554 {
    pub fn new(i2c: esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>) -> Self {
        Self { i2c, address: 0x20 }
    }

    pub fn write_direction_reg(&mut self, value: u8) -> Result<(), esp_hal::i2c::master::Error> {
        self.i2c.write(self.address, &[0x03, value])
    }

    pub fn write_output_reg(&mut self, value: u8) -> Result<(), esp_hal::i2c::master::Error> {
        self.i2c.write(self.address, &[0x01, value])
    }

    pub fn into_i2c(self) -> esp_hal::i2c::master::I2c<'static, esp_hal::Blocking> {
        self.i2c
    }
}

struct EspEmbassyBackend {
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl EspEmbassyBackend {
    fn new(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl slint::platform::Platform for EspEmbassyBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        embassy_time::Instant::now()
            .duration_since(embassy_time::Instant::from_secs(0))
            .into()
    }
}

// Display initialization commands for the ESP32-S3-LCD-EV-Board
#[derive(Copy, Clone, Debug)]
enum InitCmd {
    Cmd(u8, &'static [u8]),
    Delay(u8),
}

static INIT_CMDS: &[InitCmd] = &[
    InitCmd::Cmd(0xf0, &[0x55, 0xaa, 0x52, 0x08, 0x00]),
    InitCmd::Cmd(0xf6, &[0x5a, 0x87]),
    InitCmd::Cmd(0xc1, &[0x3f]),
    InitCmd::Cmd(0xc2, &[0x0e]),
    InitCmd::Cmd(0xc6, &[0xf8]),
    InitCmd::Cmd(0xc9, &[0x10]),
    InitCmd::Cmd(0xcd, &[0x25]),
    InitCmd::Cmd(0xf8, &[0x8a]),
    InitCmd::Cmd(0xac, &[0x45]),
    InitCmd::Cmd(0xa0, &[0xdd]),
    InitCmd::Cmd(0xa7, &[0x47]),
    InitCmd::Cmd(0xfa, &[0x00, 0x00, 0x00, 0x04]),
    InitCmd::Cmd(0x86, &[0x99, 0xa3, 0xa3, 0x51]),
    InitCmd::Cmd(0xa3, &[0xee]),
    InitCmd::Cmd(0xfd, &[0x3c, 0x3]),
    InitCmd::Cmd(0x71, &[0x48]),
    InitCmd::Cmd(0x72, &[0x48]),
    InitCmd::Cmd(0x73, &[0x00, 0x44]),
    InitCmd::Cmd(0x97, &[0xee]),
    InitCmd::Cmd(0x83, &[0x93]),
    InitCmd::Cmd(0x9a, &[0x72]),
    InitCmd::Cmd(0x9b, &[0x5a]),
    InitCmd::Cmd(0x82, &[0x2c, 0x2c]),
    InitCmd::Cmd(0xB1, &[0x10]),
    InitCmd::Cmd(
        0x6d,
        &[
            0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a, 0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e,
            0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01, 0x09, 0x0b, 0x0d, 0x0f,
            0x1a, 0x19, 0x1f, 0x00,
        ],
    ),
    InitCmd::Cmd(
        0x64,
        &[
            0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04, 0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x65,
        &[
            0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02, 0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x66,
        &[
            0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00, 0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x67,
        &[
            0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02, 0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x68,
        &[
            0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(0x60, &[0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a]),
    InitCmd::Cmd(0x63, &[0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a]),
    InitCmd::Cmd(0x69, &[0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08]),
    InitCmd::Cmd(0x6b, &[0x07]),
    InitCmd::Cmd(0x7a, &[0x08, 0x13]),
    InitCmd::Cmd(0x7b, &[0x08, 0x13]),
    InitCmd::Cmd(
        0xd1,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd2,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd3,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd4,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd5,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd6,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(0x36, &[0x00]),
    InitCmd::Cmd(0x2A, &[0x00, 0x00, 0x01, 0xDF]), // 0 to 479 (0x1DF)
    // Set full row address range
    InitCmd::Cmd(0x2B, &[0x00, 0x00, 0x01, 0xDF]), // 0 to 479 (0x1DF)
    InitCmd::Cmd(0x3A, &[0x66]),
    InitCmd::Cmd(0x11, &[]),
    InitCmd::Delay(120),
    InitCmd::Cmd(0x29, &[]),
    InitCmd::Delay(20),
];

// Automatic WiFi UI refresh task - periodically checks for new scan results
#[embassy_executor::task]
async fn auto_wifi_refresh_task(ui_weak: slint::Weak<MainWindow>) {
    info!("=== Auto WiFi refresh task started ====");

    let mut ticker = Ticker::every(Duration::from_secs(2));

    loop {
        ticker.next().await;

        // Check if new WiFi scan results are available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            // Try to upgrade weak reference to UI
            if let Some(ui) = ui_weak.upgrade() {
                info!("Auto-refreshing WiFi UI with new scan results");
                ui.invoke_wifi_refresh();
            } else {
                // UI has been dropped, stop the task
                info!("UI reference dropped, stopping auto-refresh task");
                break;
            }
        }
    }
}

// Graphics rendering task - handles display output and WiFi UI refresh
#[embassy_executor::task]
async fn graphics_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
    mut dpi: esp_hal::lcd_cam::lcd::dpi::Dpi<'static, esp_hal::Blocking>,
    mut dma_tx: esp_hal::dma::DmaTxBuf,
    pixel_buf: &'static mut [slint::platform::software_renderer::Rgb565Pixel; LCD_BUFFER_SIZE],
) {
    info!("=== Graphics rendering task started ====");

    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut frame_counter = 0u32;

    info!(
        "Graphics task initialized with {}x{} buffer",
        LCD_H_RES, LCD_V_RES
    );

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Request redraw to ensure rendering occurs
        window.request_redraw();

        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                info!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Render the frame if needed
        let rendered = window.draw_if_needed(|renderer| {
            renderer.render(pixel_buf, LCD_H_RES as usize);
        });

        // If a frame was rendered, transfer it via DMA
        if rendered {
            // Pack pixels into DMA buffer
            let dst = dma_tx.as_mut_slice();
            for (i, px) in pixel_buf.iter().enumerate() {
                let [lo, hi] = px.0.to_le_bytes();
                dst[2 * i] = lo;
                dst[2 * i + 1] = hi;
            }

            // One-shot DMA transfer of the full frame
            match dpi.send(false, dma_tx) {
                Ok(xfer) => {
                    let (res, dpi2, tx2) = xfer.wait();
                    dpi = dpi2;
                    dma_tx = tx2;
                    if let Err(e) = res {
                        error!("DMA error: {:?}", e);
                    }
                }
                Err((e, dpi2, tx2)) => {
                    error!("DMA send error: {:?}", e);
                    dpi = dpi2;
                    dma_tx = tx2;
                }
            }

            if frame_counter % 60 == 0 {
                info!(
                    "Frame {} rendered and displayed on ESP32-S3-LCD-EV-Board",
                    frame_counter
                );
            }
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            info!(
                "Graphics: Frame {}, ESP32-S3-LCD-EV-Board display active",
                frame_counter
            );
        }

        ticker.next().await;
    }
}

// WiFi scanning task
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("=== WiFi scan task started ====");

    // Check WiFi capabilities
    info!("WiFi capabilities: {:?}", wifi_controller.capabilities());

    // Report heap statistics after WiFi scanning task starts
    report_heap_stats("After WiFi scanning task spawn");

    // Configure WiFi as Client (following esope-sld-c-w-s3 working pattern)
    let client_config = ModeConfig::Client(ClientConfig::default());
    match wifi_controller.set_config(&client_config) {
        Ok(_) => info!("WiFi configuration set successfully"),
        Err(e) => info!("Failed to set WiFi configuration: {:?}", e),
    }

    // Start WiFi
    match wifi_controller.start_async().await {
        Ok(_) => info!("WiFi started successfully!"),
        Err(e) => info!("Failed to start WiFi: {:?}", e),
    }

    // Wait a bit for WiFi to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    loop {
        info!("Performing WiFi scan...");

        match wifi_controller
            .scan_with_config_async(ScanConfig::default().with_max(10))
            .await
        {
            Ok(results) => {
                info!("Found {} networks:", results.len());
                for (i, ap) in results.iter().enumerate() {
                    info!(
                        "  {}: SSID: {}, Signal: {:?}, Auth: {:?}, Channel: {}",
                        i + 1,
                        ap.ssid.as_str(),
                        ap.signal_strength,
                        ap.auth_method,
                        ap.channel
                    );
                }

                // Store scan results in shared state
                if let Ok(mut scan_results) = WIFI_SCAN_RESULTS.try_lock() {
                    scan_results.clear();
                    scan_results.extend_from_slice(&results);
                    WIFI_SCAN_UPDATED.store(true, Ordering::Relaxed);
                    info!("Stored {} scan results for UI", scan_results.len());
                } else {
                    info!("Could not store scan results (mutex locked)");
                }
            }
            Err(e) => {
                info!("WiFi scan failed: {:?}", e);
            }
        }

        // Wait 10 seconds before next scan
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize peripherals first
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize IRAM heap for WiFi and small allocations
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 70 * 1024);
    esp_alloc::heap_allocator!(size: 90 * 1024);

    // Initialize PSRAM heap for large allocations like framebuffer using esp-hal 1.0.0 macro
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    info!("PSRAM heap initialized using psram_allocator! macro");

    // Initialize logger
    init_logger_from_env();
    info!("Peripherals initialized");

    // Report initial heap statistics
    report_heap_stats("After heap initialization");

    info!("Starting Slint ESP32-S3-LCD-EV-Board Workshop");

    // Setup I2C for the TCA9554 IO expander and FT5x06 touch controller
    let i2c = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO47)
    .with_scl(peripherals.GPIO48);

    // Initialize the IO expander for controlling the display
    let mut expander = Tca9554::new(i2c);
    expander.write_output_reg(0b1111_0011).unwrap();
    expander.write_direction_reg(0b1111_0001).unwrap();

    let delay = Delay::new();
    info!("Initializing display...");

    // Set up the write_byte function for sending commands to the display
    let mut write_byte = |b: u8, is_cmd: bool| {
        const SCS_BIT: u8 = 0b0000_0010;
        const SCL_BIT: u8 = 0b0000_0100;
        const SDA_BIT: u8 = 0b0000_1000;

        let mut output = 0b1111_0001 & !SCS_BIT;
        expander.write_output_reg(output).unwrap();

        for bit in core::iter::once(!is_cmd).chain((0..8).map(|i| (b >> i) & 0b1 != 0).rev()) {
            let prev = output;
            if bit {
                output |= SDA_BIT;
            } else {
                output &= !SDA_BIT;
            }
            if prev != output {
                expander.write_output_reg(output).unwrap();
            }

            output &= !SCL_BIT;
            expander.write_output_reg(output).unwrap();

            output |= SCL_BIT;
            expander.write_output_reg(output).unwrap();
        }

        output &= !SCL_BIT;
        expander.write_output_reg(output).unwrap();

        output &= !SDA_BIT;
        expander.write_output_reg(output).unwrap();

        output |= SCS_BIT;
        expander.write_output_reg(output).unwrap();
    };

    // VSYNC must be high during initialization
    let vsync_pin = mk_static!(esp_hal::peripherals::GPIO3, peripherals.GPIO3);
    let vsync_guard = Output::new(vsync_pin.reborrow(), Level::High, OutputConfig::default());

    // Initialize the display by sending the initialization commands
    for &init in INIT_CMDS.iter() {
        match init {
            InitCmd::Cmd(cmd, args) => {
                write_byte(cmd, true);
                for &arg in args {
                    write_byte(arg, false);
                }
            }
            InitCmd::Delay(ms) => {
                delay.delay_millis(ms as _);
            }
        }
    }
    drop(vsync_guard);

    // Set up DMA channel for LCD
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Configure the RGB display
    let config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(10))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: LCD_H_RES as usize,
            vertical_active_height: LCD_V_RES as usize,
            horizontal_total_width: 600,
            horizontal_blank_front_porch: 80,
            vertical_total_height: 600,
            vertical_blank_front_porch: 80,
            hsync_width: 10,
            vsync_width: 4,
            hsync_position: 10,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let mut dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
        .unwrap()
        .with_vsync(vsync_pin.reborrow())
        .with_hsync(peripherals.GPIO46)
        .with_de(peripherals.GPIO17)
        .with_pclk(peripherals.GPIO9)
        .with_data0(peripherals.GPIO10)
        .with_data1(peripherals.GPIO11)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13)
        .with_data4(peripherals.GPIO14)
        .with_data5(peripherals.GPIO21)
        .with_data6(peripherals.GPIO8)
        .with_data7(peripherals.GPIO18)
        .with_data8(peripherals.GPIO45)
        .with_data9(peripherals.GPIO38)
        .with_data10(peripherals.GPIO39)
        .with_data11(peripherals.GPIO40)
        .with_data12(peripherals.GPIO41)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO2)
        .with_data15(peripherals.GPIO1);

    info!("Display initialized, entering main loop...");

    // Allocate a PSRAM-backed DMA buffer for the frame
    let buf_box: Box<[u8; FRAME_BYTES]> = Box::new([0; FRAME_BYTES]);
    let psram_buf: &'static mut [u8] = Box::leak(buf_box);
    let mut dma_tx: DmaTxBuf = unsafe {
        let descriptors = &mut *core::ptr::addr_of_mut!(TX_DESCRIPTORS);
        DmaTxBuf::new(descriptors, psram_buf).unwrap()
    };
    let mut pixel_box: Box<[Rgb565Pixel; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565Pixel(0); LCD_BUFFER_SIZE]);
    let pixel_buf: &mut [Rgb565Pixel] = &mut *pixel_box;

    // Initialize pixel buffer and DMA buffer
    let dst = dma_tx.as_mut_slice();
    for (i, px) in pixel_buf.iter().enumerate() {
        let [lo, hi] = px.0.to_le_bytes();
        dst[2 * i] = lo;
        dst[2 * i + 1] = hi;
    }

    // Initial flush of the screen buffer
    match dpi.send(false, dma_tx) {
        Ok(xfer) => {
            let (_res, dpi2, tx2) = xfer.wait();
            dpi = dpi2;
            dma_tx = tx2;
        }
        Err((e, dpi2, tx2)) => {
            error!("Initial DMA send error: {:?}", e);
            dpi = dpi2;
            dma_tx = tx2;
        }
    }

    // Initialize embassy timer for task scheduling BEFORE spawning tasks
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    // For ESP32-S3 (Xtensa), we don't need the software interrupt parameter
    esp_rtos::start(timg0.timer0);
    info!("esp-rtos timer initialized");

    // Initialize WiFi using new esp-radio API FIRST (before framebuffer allocation)
    info!("Initializing WiFi...");
    let radio_init = mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Config::default())
            .expect("Failed to initialize Wi-Fi controller");

    // Extract the station interface for WiFi operations
    let _wifi_interface = interfaces.sta;
    info!("WiFi controller initialized with station interface");

    // Small delay to ensure WiFi initialization is complete
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    // Create custom Slint window and backend
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.set_size(PhysicalSize::new(LCD_H_RES.into(), LCD_V_RES.into()));

    let backend = Box::new(EspEmbassyBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("Custom Slint backend initialized");

    // Create the UI
    let ui = MainWindow::new().unwrap();

    // Create empty WiFi network model with some placeholder data
    let placeholder_networks = vec![
        WifiNetwork {
            ssid: "WiFi Scanning...".into(),
        },
        WifiNetwork {
            ssid: "Please wait".into(),
        },
    ];

    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(placeholder_networks));
    ui.set_wifi_network_model(wifi_model.clone().into());

    // Set up WiFi refresh handler with real WiFi functionality
    ui.on_wifi_refresh(move || {
        info!("WiFi refresh requested - checking for scan results");

        // Check if we have new scan results
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            // Access the scan results
            let scan_results = WIFI_SCAN_RESULTS.try_lock();
            if let Ok(results) = scan_results {
                let mut networks = alloc::vec::Vec::new();

                for ap in results.iter() {
                    networks.push(WifiNetwork {
                        ssid: ap.ssid.as_str().into(),
                    });
                }

                if networks.is_empty() {
                    networks.push(WifiNetwork {
                        ssid: "No networks found".into(),
                    });
                }

                info!("Updated UI with {} real networks", networks.len());
                wifi_model.set_vec(networks);

                // Reset the update flag
                WIFI_SCAN_UPDATED.store(false, Ordering::Relaxed);
            } else {
                info!("Could not access scan results (locked)");
            }
        } else {
            info!("No new scan results available");
        }
    });

    // Trigger initial refresh
    ui.invoke_wifi_refresh();

    // Convert pixel buffer to static for graphics task
    let pixel_buf_static: &'static mut [Rgb565Pixel; LCD_BUFFER_SIZE] = Box::leak(pixel_box);

    // Store UI reference for tasks
    let ui_for_tasks = ui.as_weak();
    let ui_for_auto_refresh = ui.as_weak();

    // Store WiFi controller for the render loop task
    let wifi_ctrl = wifi_controller;

    // Spawn WiFi scanning task
    info!("Spawning WiFi scan task");
    spawner.spawn(wifi_scan_task(wifi_ctrl)).ok();

    // Spawn automatic WiFi UI refresh task
    info!("Spawning automatic WiFi refresh task");
    spawner
        .spawn(auto_wifi_refresh_task(ui_for_auto_refresh))
        .ok();

    // Spawn graphics rendering task with all required resources
    info!("Spawning graphics rendering task");
    spawner
        .spawn(graphics_task(
            window.clone(),
            ui_for_tasks,
            dpi,
            dma_tx,
            pixel_buf_static,
        ))
        .ok();

    // Show the window
    ui.show().unwrap();

    info!("=== All systems initialized, entering main loop ===");

    // Simple main loop to keep the Embassy executor alive (matching ESoPe pattern)
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;

        // Check for WiFi updates periodically
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            info!("WiFi scan results available for UI update");
        }
    }
}
