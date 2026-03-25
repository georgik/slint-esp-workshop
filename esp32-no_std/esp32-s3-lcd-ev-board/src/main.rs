#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::panic::PanicInfo;
use log::{debug, error, info};

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi imports - using esp-radio
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};
use esp_radio::wifi::{AccessPointInfo, ClientConfig, ModeConfig, ScanConfig, WifiController};

// ESP32 HAL imports
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::dma::ExternalBurstConfig;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

// PSRAM configuration imports
#[cfg(feature = "psram")]
use esp_hal::psram::{PsramConfig, SpiRamFreq};
use esp_println::logger::init_logger_from_env;

// Type alias for touch controller I2C to avoid impl Trait in task signatures
type TouchI2c = esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>;

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
use slint::PhysicalPosition;
use slint::PhysicalSize;
use slint::platform::software_renderer::Rgb565Pixel;
use slint::platform::{PointerEventButton, WindowEvent};

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

// Embassy multicore: allocate app core stack
static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();

// PSRAM synchronization signals
static PSRAM_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static mut PSRAM_BUF_PTR: *mut u8 = core::ptr::null_mut();
static mut PSRAM_BUF_LEN: usize = 0;

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

// DMA display task - runs on Core 1, handles pure display output
#[embassy_executor::task]
async fn dma_display_task(
    mut dpi: esp_hal::lcd_cam::lcd::dpi::Dpi<'static, esp_hal::Blocking>,
    mut dma_tx: esp_hal::dma::DmaTxBuf,
) {
    info!("[CORE 1] DMA display task started, continuous refresh for RGB display");

    loop {
        let frame_bytes = LCD_H_RES_USIZE * LCD_V_RES_USIZE * 2; // RGB565: 2 bytes per pixel
        dma_tx.set_length(frame_bytes);

        match dpi.send(false, dma_tx) {
            Ok(xfer) => {
                let (res, new_dpi, new_dma_tx) = xfer.wait();
                dpi = new_dpi;
                dma_tx = new_dma_tx;
                if let Err(e) = res {
                    error!("[CORE 1] DMA transfer error: {:?}", e);
                }
            }
            Err((e, new_dpi, new_dma_tx)) => {
                error!("[CORE 1] DMA send error: {:?}", e);
                dpi = new_dpi;
                dma_tx = new_dma_tx;
            }
        }
    }
}

// Slint rendering task - runs on Core 0, handles UI rendering and touch polling
#[embassy_executor::task]
async fn slint_rendering_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
    mut touch_controller: Ft5x06<TouchI2c>,
) {
    info!("[CORE 0] Slint rendering task started");

    // Wait until PSRAM is ready
    loop {
        if PSRAM_READY.try_take().is_some() {
            break;
        }
        core::hint::spin_loop();
    }
    debug!("[CORE 0] PSRAM ready, starting Slint rendering");

    // SAFETY: PSRAM_BUF_PTR and PSRAM_BUF_LEN are published before this task starts
    let psram_ptr = unsafe { PSRAM_BUF_PTR };
    let _psram_len = unsafe { PSRAM_BUF_LEN };

    // Convert to Rgb565 buffer following esope approach
    let pixel_buf: &mut [slint::platform::software_renderer::Rgb565Pixel; LCD_BUFFER_SIZE] = unsafe {
        &mut *(psram_ptr as *mut [slint::platform::software_renderer::Rgb565Pixel; LCD_BUFFER_SIZE])
    };

    let mut frame_counter = 0u32;
    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    let mut touch_ticker = Ticker::every(Duration::from_millis(16)); // ~60Hz touch polling
    let mut last_touch_state: Option<(u16, u16)> = None;
    let mut last_touch_position = slint::LogicalPosition::new(0.0, 0.0);

    loop {
        // === Touch Polling (Low Priority) ===
        match touch_controller.get_touch() {
            Ok(touch_result) => {
                match (last_touch_state.as_ref(), touch_result) {
                    // Touch press event (transition from None to Some)
                    (None, Some((x, y))) => {
                        let physical_position = PhysicalPosition::new(x as i32, y as i32);
                        let logical_position = physical_position.to_logical(window.scale_factor());
                        last_touch_position = logical_position;

                        let pointer_event = WindowEvent::PointerPressed {
                            position: logical_position,
                            button: PointerEventButton::Left,
                        };

                        window.dispatch_event(pointer_event);
                        debug!("[CORE 0] Touch PRESSED at {},{}", x, y);
                    }
                    // Touch release event (transition from Some to None)
                    (Some(_), None) => {
                        let pointer_released = WindowEvent::PointerReleased {
                            position: last_touch_position,
                            button: PointerEventButton::Left,
                        };
                        window.dispatch_event(pointer_released);

                        // Also send PointerExited to complete the interaction cycle
                        let pointer_exited = WindowEvent::PointerExited;
                        window.dispatch_event(pointer_exited);
                    }
                    // Touch move event (both states are Some but potentially different positions)
                    (Some((old_x, old_y)), Some((new_x, new_y))) => {
                        // Only dispatch move event if position actually changed
                        if *old_x != new_x || *old_y != new_y {
                            let physical_position =
                                PhysicalPosition::new(new_x as i32, new_y as i32);
                            let logical_position =
                                physical_position.to_logical(window.scale_factor());
                            last_touch_position = logical_position;

                            let pointer_event = WindowEvent::PointerMoved {
                                position: logical_position,
                            };

                            window.dispatch_event(pointer_event);
                        }
                    }
                    // No state change
                    _ => {}
                }

                last_touch_state = touch_result;
            }
            Err(_) => {
                // Touch polling error - don't spam logs, just continue
            }
        }

        // === Core Rendering (High Priority) ===
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                debug!("[CORE 0] Triggered UI refresh for new WiFi scan results");
            }
        }

        // Render the frame if needed (Slint handles dirty tracking internally)
        let rendered = window.draw_if_needed(|renderer| {
            renderer.render(pixel_buf, LCD_H_RES as usize);
        });

        // For RGB displays, ALWAYS refresh the framebuffer to prevent fading
        // Even when there are no UI changes, we need continuous DMA transfers
        if rendered {
            // UI changed - frame data is already in pixel_buf
            debug!("[CORE 0] Frame {} rendered, sending to DMA", frame_counter);
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Reduce periodic status logging to avoid screen interference
        if frame_counter % 1800 == 0 {
            // Every ~30 seconds at 60fps (less frequent to reduce noise)
            info!(
                "[CORE 0] Slint: Frame {}, ESP32-S3-LCD-EV-Board rendering active",
                frame_counter
            );
        }

        // Timing coordination - advance both tickers
        ticker.next().await;
        touch_ticker.next().await;
    }
}

// WiFi scanning task
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("=== WiFi scan task started ====");

    // Check WiFi capabilities (debug level, not critical)
    debug!("WiFi capabilities: {:?}", wifi_controller.capabilities());

    // Configure WiFi as Client (following esope-sld-c-w-s3 working pattern)
    let client_config = ModeConfig::Client(ClientConfig::default());
    match wifi_controller.set_config(&client_config) {
        Ok(_) => info!("WiFi configuration set successfully"),
        Err(e) => error!("Failed to set WiFi configuration: {:?}", e),
    }

    // Start WiFi
    match wifi_controller.start_async().await {
        Ok(_) => info!("WiFi started successfully!"),
        Err(e) => error!("Failed to start WiFi: {:?}", e),
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
                // Reduce verbose network logging - only log summary
                if results.len() > 0 {
                    info!("Found {} networks, updating UI", results.len());

                    // Store scan results in shared state first (critical operation)
                    if let Ok(mut scan_results) = WIFI_SCAN_RESULTS.try_lock() {
                        scan_results.clear();
                        scan_results.extend_from_slice(&results);
                        WIFI_SCAN_UPDATED.store(true, Ordering::Relaxed);
                    }
                }

                // Log detailed network info only at debug level to avoid screen interference
                if results.len() > 0 {
                    debug!(
                        "Network details available for {} access points",
                        results.len()
                    );
                }
            }
            Err(e) => {
                error!("WiFi scan failed: {:?}", e);
            }
        }

        // Wait 10 seconds before next scan
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize peripherals first with optimized PSRAM configuration
    #[cfg(feature = "psram")]
    let config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max())
        .with_psram(PsramConfig {
            ram_frequency: SpiRamFreq::Freq120m,
            ..Default::default()
        });

    #[cfg(not(feature = "psram"))]
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());

    let peripherals = esp_hal::init(config);

    #[cfg(feature = "psram")]
    info!("ESP32-S3 initialized with 120MHz PSRAM frequency");
    #[cfg(not(feature = "psram"))]
    info!("ESP32-S3 initialized (PSRAM disabled)");

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
    // Using R16 module pin mapping (latest LCD EV kit): SDA=GPIO47, SCL=GPIO48
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

    // Get I2C bus back from expander for touch controller
    let i2c = expander.into_i2c();
    info!("Retrieved I2C bus from expander for touch controller");

    // Store I2C bus for touch controller (will be initialized after window creation)
    let touch_i2c = i2c;

    // Set up DMA channel for LCD
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Configure the RGB display - Using official BSP timing for GC9503
    let config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(16))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: LCD_H_RES as usize,
            vertical_active_height: LCD_V_RES as usize,
            horizontal_total_width: 520,
            horizontal_blank_front_porch: 20,
            vertical_total_height: 510,
            vertical_blank_front_porch: 10,
            hsync_width: 10,
            vsync_width: 10,
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

    // Initialize pixel buffer and DMA buffer using optimized bulk copy
    let dst = dma_tx.as_mut_slice();
    let src = unsafe {
        core::slice::from_raw_parts(pixel_buf.as_ptr() as *const u8, pixel_buf.len() * 2)
    };
    dst.copy_from_slice(src);

    // Initial flush of the screen buffer
    match dpi.send(false, dma_tx) {
        Ok(xfer) => {
            let (_res, dpi2, _tx2) = xfer.wait();
            dpi = dpi2;
        }
        Err((e, dpi2, _tx2)) => {
            error!("Initial DMA send error: {:?}", e);
            dpi = dpi2;
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
    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
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

    // Initialize FT5x06 touch controller using address 0x38 (from ESP BSP)
    let mut touch_controller = Ft5x06::new(touch_i2c, 0x38);
    info!("FT5x06 touch controller initialized with I2C address 0x38");

    // Test touch polling once to verify it's working
    info!("Testing touch controller polling...");
    match touch_controller.get_touch() {
        Ok(touch_result) => match touch_result {
            Some((x, y)) => {
                info!("Touch controller test: Touch detected at x={}, y={}", x, y);
            }
            None => {
                info!("Touch controller test: No touch detected");
            }
        },
        Err(e) => {
            info!("Touch controller test: Error reading touch: {:?}", e);
        }
    }
    info!("Touch controller test completed - polling functionality verified");

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

    // Allocate framebuffer in PSRAM following esope dual-core approach
    let mut fb_box: Box<[Rgb565Pixel; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565Pixel(0); LCD_BUFFER_SIZE]);

    // Create test pattern first to verify display works
    // Use solid colors to test - red in top-left, green in top-right, blue in bottom-left
    for i in 0..LCD_BUFFER_SIZE {
        let x = i % LCD_H_RES_USIZE;
        let y = i / LCD_H_RES_USIZE;

        // RGB565 format: RRRRRGGGGGGBBBBB
        // Red: bits 15-11, Green: bits 10-5, Blue: bits 4-0
        let color = if x < LCD_H_RES_USIZE / 2 && y < LCD_V_RES_USIZE / 2 {
            // Top-left: Red (0b11111_000000_00000 = 0xF800)
            0xF800
        } else if x >= LCD_H_RES_USIZE / 2 && y < LCD_V_RES_USIZE / 2 {
            // Top-right: Green (0b00000_111111_00000 = 0x07E0)
            0x07E0
        } else {
            // Bottom: Blue (0b00000_000000_11111 = 0x001F)
            0x001F
        };

        fb_box[i] = Rgb565Pixel(color);
    }
    info!("Test pattern written to framebuffer - Red/Green/Blue quadrants");

    let fb_ptr: *mut Rgb565Pixel = fb_box.as_mut_ptr();
    let psram_buf: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(fb_ptr as *mut u8, FRAME_BYTES) };

    // Verify PSRAM buffer allocation and alignment (CRITICAL!)
    let buf_ptr = psram_buf.as_ptr() as usize;
    info!("PSRAM buffer allocated at address: 0x{:08X}", buf_ptr);
    info!("PSRAM buffer length: {}", psram_buf.len());
    info!("PSRAM buffer alignment modulo 64: {}", buf_ptr % 64);
    assert!(
        buf_ptr % 64 == 0,
        "PSRAM buffer must be 64-byte aligned for DMA"
    );

    // Publish PSRAM buffer pointer and len for other cores
    unsafe {
        PSRAM_BUF_PTR = psram_buf.as_mut_ptr();
        PSRAM_BUF_LEN = psram_buf.len();
    }

    // Configure DMA buffer - matching working example
    let mut dma_tx: DmaTxBuf =
        unsafe { DmaTxBuf::new(&mut *core::ptr::addr_of_mut!(TX_DESCRIPTORS), psram_buf).unwrap() };

    // **CRITICAL**: Do initial DMA transfer with test pattern before starting tasks
    // This ensures the display shows something immediately
    info!("Starting initial DMA transfer with test pattern...");
    dma_tx.set_length(FRAME_BYTES);
    match dpi.send(false, dma_tx) {
        Ok(xfer) => {
            let (_res, dpi2, tx2) = xfer.wait();
            dpi = dpi2;
            dma_tx = tx2;
            info!("Initial DMA transfer completed successfully");
        }
        Err((e, dpi2, tx2)) => {
            error!("Initial DMA transfer failed: {:?}", e);
            dpi = dpi2;
            dma_tx = tx2;
        }
    }

    // Split peripherals for multicore usage
    let (dpi_for_display, _) = (dpi, ());

    // Signal that PSRAM is ready
    PSRAM_READY.signal(());

    // Initialize software interrupts for multicore support
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // **CRITICAL**: Start app core with esp-rtos dual-core
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_ints.software_interrupt0,
        sw_ints.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
            let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner| {
                spawner
                    .spawn(dma_display_task(dpi_for_display, dma_tx))
                    .ok();
            });
        },
    );

    // Show the window
    ui.show().unwrap();

    info!("=== All systems initialized, dual-core active ===");
    info!("Core 0: WiFi + Slint rendering + Touch polling");
    info!("Core 1: DMA display output");

    // **Core 0**: Spawn WiFi tasks
    info!("Spawning WiFi scan task on Core 0");
    spawner.spawn(wifi_scan_task(wifi_controller)).ok();

    info!("Spawning automatic WiFi refresh task on Core 0");
    spawner.spawn(auto_wifi_refresh_task(ui.as_weak())).ok();

    // **Core 0**: Spawn Slint rendering task with touch support
    info!("Spawning Slint rendering task on Core 0");
    spawner
        .spawn(slint_rendering_task(
            window.clone(),
            ui.as_weak(),
            touch_controller,
        ))
        .ok();

    // Simple main loop following esope pattern
    let mut main_ticker = Ticker::every(Duration::from_secs(10));
    loop {
        main_ticker.next().await;
        info!("Main task alive - ESP32-S3-LCD-EV-Board dual-core system running");
    }
}
