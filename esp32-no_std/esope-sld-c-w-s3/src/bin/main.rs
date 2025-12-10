#![no_std]
#![no_main]

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::cell::RefCell;

// Display imports
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_framebuf::backends::FrameBufferBackend;

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi imports
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use esp_radio::wifi::{AccessPointInfo, ClientConfig, ModeConfig, ScanConfig, WifiController};

// Display imports
use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::CpuClock;
use esp_hal::dma::ExternalBurstConfig;
use esp_hal::dma::{CHUNK_SIZE, DmaDescriptor, DmaTxBuf};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode, Phase, Polarity,
        dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
    },
};
use esp_hal::rng::Rng;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::{AnyTimer, timg::TimerGroup};
use esp_println::logger::init_logger_from_env;
use log::{debug, error, info};

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use static_cell::StaticCell;

// Static storage for I2C bus
static I2C_BUS: StaticCell<RefCell<I2c<'static, esp_hal::Blocking>>> = StaticCell::new();

// FrameBufferBackend wrapper for a PSRAM-backed [Rgb565; N] slice.
pub struct PSRAMFrameBuffer<'a> {
    buf: &'a mut [Rgb565; LCD_BUFFER_SIZE],
}

impl<'a> PSRAMFrameBuffer<'a> {
    pub fn new(buf: &'a mut [Rgb565; LCD_BUFFER_SIZE]) -> Self {
        Self { buf }
    }
}

impl<'a> FrameBufferBackend for PSRAMFrameBuffer<'a> {
    type Color = Rgb565;
    fn set(&mut self, index: usize, color: Self::Color) {
        self.buf[index] = color;
    }
    fn get(&self, index: usize) -> Self::Color {
        self.buf[index]
    }
    fn nr_elements(&self) -> usize {
        LCD_BUFFER_SIZE
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {}
}

extern crate alloc;

// Constants
const LCD_H_RES_USIZE: usize = 320;
const LCD_V_RES_USIZE: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;
const FRAME_BYTES: usize = LCD_BUFFER_SIZE * 2;

// Embassy multicore: allocate app core stack
static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();

// PSRAM synchronization signals
static PSRAM_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static mut PSRAM_BUF_PTR: *mut u8 = core::ptr::null_mut();
static mut PSRAM_BUF_LEN: usize = 0;

// Full-screen DMA constants
const MAX_FRAME_BYTES: usize = 320 * 240 * 2;
const MAX_NUM_DMA_DESC: usize = (MAX_FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

#[unsafe(link_section = ".dma")]
static mut TX_DESCRIPTORS: [DmaDescriptor; MAX_NUM_DMA_DESC] =
    [DmaDescriptor::EMPTY; MAX_NUM_DMA_DESC];

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
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

// WiFi scanning task - runs on Core 0
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("=== WiFi scan task started ====");

    // Check WiFi capabilities
    info!("WiFi capabilities: {:?}", wifi_controller.capabilities());

    // Configure WiFi as Client (following working pattern)
    let client_config = ModeConfig::Client(ClientConfig::default());

    match wifi_controller.set_config(&client_config) {
        Ok(_) => info!("WiFi configuration set successfully"),
        Err(e) => info!("Failed to set WiFi configuration: {:?}", e),
    }

    match wifi_controller.start_async().await {
        Ok(_) => info!("WiFi started successfully!"),
        Err(e) => info!("Failed to start WiFi: {:?}", e),
    }

    // Wait a bit for WiFi to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    loop {
        info!("Performing WiFi scan...");

        let scan_config = ScanConfig::default().with_max(10);
        match wifi_controller.scan_with_config_async(scan_config).await {
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

// Automatic WiFi UI refresh task - runs on Core 0
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

// DMA display task - runs on Core 1
#[embassy_executor::task]
async fn dma_display_task(mut dpi: Dpi<'static, esp_hal::Blocking>, mut dma_tx: DmaTxBuf) {
    info!("[CORE 1] DMA display task started, sending DMA frames");

    loop {
        let frame_bytes = 320 * 240 * 2; // Fixed to known display size
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

// Slint rendering task - runs on Core 0
#[embassy_executor::task]
async fn slint_rendering_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    info!("[CORE 0] Slint rendering task started");

    // Wait until PSRAM is ready
    loop {
        if PSRAM_READY.try_take().is_some() {
            break;
        }
        core::hint::spin_loop();
    }
    info!("[CORE 0] PSRAM ready, starting Slint rendering");

    // SAFETY: PSRAM_BUF_PTR and PSRAM_BUF_LEN are published before this task starts
    let psram_ptr = unsafe { PSRAM_BUF_PTR };
    let psram_len = unsafe { PSRAM_BUF_LEN };

    // Convert to Rgb565 buffer following Conway's approach
    let fb: &mut [Rgb565; LCD_BUFFER_SIZE] =
        unsafe { &mut *(psram_ptr as *mut [Rgb565; LCD_BUFFER_SIZE]) };

    let mut frame_counter = 0u32;

    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                debug!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Use draw_if_needed to check if we need to render and get access to the renderer
        let rendered = window.draw_if_needed(|renderer| {
            // Render directly to PSRAM buffer
            let pixel_slice = unsafe {
                core::slice::from_raw_parts_mut(
                    fb.as_mut_ptr() as *mut slint::platform::software_renderer::Rgb565Pixel,
                    LCD_BUFFER_SIZE,
                )
            };
            renderer.render(pixel_slice, LCD_H_RES_USIZE);

            if frame_counter % 60 == 0 {
                debug!("[CORE 0] Frame {} rendered by Slint", frame_counter);
            }
        });

        // If a frame was rendered, log it
        if rendered {
            if frame_counter % 60 == 0 {
                info!(
                    "[CORE 0] Frame {} rendered and displayed on ESP32-S3 ESoPe",
                    frame_counter
                );
            }
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            info!(
                "[CORE 0] Slint: Frame {}, ESP32-S3 ESoPe display active",
                frame_counter
            );
        }

        ticker.next().await;
    }
}

// Use Slint build compilation helper
slint::include_modules!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // Initialize peripherals with multiple heap allocators
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize IRAM heap for WiFi and small allocations (following esp32-s3-box-3 pattern)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 70 * 1024);
    esp_alloc::heap_allocator!(size: 90 * 1024);

    // Initialize PSRAM heap for large allocations like framebuffer using esp-hal 1.0.0 macro
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    info!("PSRAM heap initialized using psram_allocator! macro");

    // Initialize Embassy timer for esp-rtos (Xtensa devices use single timer)
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    esp_rtos::start(timer0);
    info!("Embassy timer initialized");

    // Initialize logger
    init_logger_from_env();
    info!("Peripherals initialized");

    info!("Starting Slint ESP32 ESoPe Board WiFi Workshop");

    // Initialize WiFi AFTER embassy is started
    info!("Initializing WiFi...");
    let esp_radio_ctrl = &*mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );
    info!("WiFi controller initialized");

    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(esp_radio_ctrl, peripherals.WIFI, Default::default())
            .expect("Failed to create WiFi interface");

    // Extract the station interface for WiFi operations
    let _wifi_interface = interfaces.sta;
    info!("WiFi controller initialized with station interface");

    // Initialize display hardware using PROVEN timing from Conway's implementation
    info!("=== Starting ESoPe Board Display Initialization ===");

    // Read display configuration from EEPROM
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO1)
        .with_scl(peripherals.GPIO41);
    let i2c_bus = I2C_BUS.init(RefCell::new(i2c));
    let mut eeid = [0u8; 0x1c];
    let mut eeprom = Eeprom24x::new_24x01(RefCellDevice::new(i2c_bus), SlaveAddr::default());
    eeprom.read_data(0x00, &mut eeid).unwrap();
    let display_width = u16::from_be_bytes([eeid[8], eeid[9]]);
    let display_height = u16::from_be_bytes([eeid[10], eeid[11]]);
    info!(
        "Display size from EEPROM: {}x{}",
        display_width, display_height
    );

    // Use hardcoded display size if EEPROM is empty or invalid
    let actual_display_width = if display_width == 0 {
        320
    } else {
        display_width
    };
    let actual_display_height = if display_height == 0 {
        240
    } else {
        display_height
    };
    info!(
        "Using display size: {}x{}",
        actual_display_width, actual_display_height
    );

    // Enable panel / backlight
    let mut panel_enable = Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default());
    panel_enable.set_high();

    let mut backlight = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default());
    backlight.set_high();

    let mut _touch_reset = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    // Add a delay to ensure the display power is stable
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    info!("Display initialized, setting up dual-core rendering...");

    // **KEY FIX**: Allocate framebuffer in PSRAM following Conway's approach
    let mut fb_box: Box<[Rgb565; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565::new(0, 0, 0); LCD_BUFFER_SIZE]);

    // Create test pattern first to verify display works
    for i in 0..LCD_BUFFER_SIZE {
        let x = i % LCD_H_RES_USIZE;
        let y = i / LCD_H_RES_USIZE;
        // Create a simple test pattern - gradient from green to blue
        let g = (x * 63 / LCD_H_RES_USIZE) as u8;
        let b = (y * 31 / LCD_V_RES_USIZE) as u8;
        fb_box[i] = Rgb565::new(0, g, b);
    }
    info!("Test pattern written to framebuffer");

    let fb_ptr: *mut Rgb565 = fb_box.as_mut_ptr();
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

    // Configure DMA buffer with proper burst configuration (following Conway)
    let dma_tx: DmaTxBuf = unsafe {
        DmaTxBuf::new_with_config(
            &mut *core::ptr::addr_of_mut!(TX_DESCRIPTORS),
            psram_buf,
            ExternalBurstConfig::Size64,
        )
        .unwrap()
    };

    // Initialize LCD DPI interface with PROVEN TIMING from Conway's implementation
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Read configuration from EEPROM
    let pclk_hz = ((eeid[12] as u32) * 1_000_000 + (eeid[13] as u32) * 100_000).min(13_600_000);
    let flags = eeid[25];
    let hsync_idle_low = (flags & 0x01) != 0;
    let vsync_idle_low = (flags & 0x02) != 0;
    let de_idle_high = (flags & 0x04) != 0;
    let pclk_active_neg = (flags & 0x20) != 0;

    // Use safe defaults if EEPROM values are invalid
    let actual_pclk_hz = if pclk_hz == 0 { 10_000_000 } else { pclk_hz }; // 10MHz default

    // Log display configuration
    info!("Display configuration:");
    info!("  EEPROM Resolution: {}x{}", display_width, display_height);
    info!(
        "  Actual Resolution: {}x{}",
        actual_display_width, actual_display_height
    );
    info!("  EEPROM PCLK: {} Hz", pclk_hz);
    info!("  Actual PCLK: {} Hz", actual_pclk_hz);
    info!("  Flags: 0x{:02X}", flags);
    info!("  HSYNC idle low: {}", hsync_idle_low);
    info!("  VSYNC idle low: {}", vsync_idle_low);
    info!("  DE idle high: {}", de_idle_high);
    info!("  PCLK active neg: {}", pclk_active_neg);

    // Use EXACT timing from Conway's working implementation
    let dpi_config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: if pclk_active_neg {
                Polarity::IdleHigh
            } else {
                Polarity::IdleLow
            },
            phase: if pclk_active_neg {
                Phase::ShiftHigh
            } else {
                Phase::ShiftLow
            },
        })
        .with_frequency(Rate::from_hz(actual_pclk_hz))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: 320,
            horizontal_total_width: 320 + 4 + 43 + 79 + 8, // =446 (Conway's working value)
            horizontal_blank_front_porch: 79 + 8,          // was 47, add 32px
            vertical_active_height: 240,
            vertical_total_height: 240 + 4 + 12 + 16, // increased blank front porch to 16
            vertical_blank_front_porch: 16,
            hsync_width: 4,
            vsync_width: 4,
            hsync_position: 43 + 4, // (= back_porch + pulse = 47) Conway's working value
        })
        .with_vsync_idle_level(if vsync_idle_low {
            Level::Low
        } else {
            Level::High
        })
        .with_hsync_idle_level(if hsync_idle_low {
            Level::Low
        } else {
            Level::High
        })
        .with_de_idle_level(if de_idle_high {
            Level::High
        } else {
            Level::Low
        })
        .with_disable_black_region(false);

    let dpi = Dpi::new(lcd_cam.lcd, peripherals.DMA_CH2, dpi_config)
        .unwrap()
        .with_vsync(peripherals.GPIO6)
        .with_hsync(peripherals.GPIO15)
        .with_de(peripherals.GPIO5)
        .with_pclk(peripherals.GPIO4)
        // Blue bus
        .with_data0(peripherals.GPIO9)
        .with_data1(peripherals.GPIO17)
        .with_data2(peripherals.GPIO46)
        .with_data3(peripherals.GPIO16)
        .with_data4(peripherals.GPIO7)
        // Green bus
        .with_data5(peripherals.GPIO8)
        .with_data6(peripherals.GPIO21)
        .with_data7(peripherals.GPIO3)
        .with_data8(peripherals.GPIO11)
        .with_data9(peripherals.GPIO18)
        .with_data10(peripherals.GPIO10)
        // Red bus
        .with_data11(peripherals.GPIO14)
        .with_data12(peripherals.GPIO20)
        .with_data13(peripherals.GPIO13)
        .with_data14(peripherals.GPIO19)
        .with_data15(peripherals.GPIO12);

    // Prepare RNG for app core task
    let rng_for_app = Rng::new();

    // Split peripherals for multicore usage
    let (dpi_for_display, _) = (dpi, rng_for_app);

    // Signal that PSRAM is ready
    PSRAM_READY.signal(());

    // Initialize software interrupts for multicore support (following Conway)
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // **CRITICAL**: Start app core with esp-rtos (Conway's approach)
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

    // Create custom Slint window and backend
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.set_size(slint::PhysicalSize::new(320, 240));

    slint::platform::set_platform(Box::new(EspEmbassyBackend::new(window.clone())))
        .expect("backend already initialized");
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

    // Store UI reference for automatic refresh
    let ui_for_refresh = ui.as_weak();

    // **Core 0**: Spawn WiFi tasks
    info!("Spawning WiFi scan task on Core 0");
    spawner.spawn(wifi_scan_task(wifi_controller)).ok();

    info!("Spawning automatic WiFi refresh task on Core 0");
    spawner.spawn(auto_wifi_refresh_task(ui_for_refresh)).ok();

    // **Core 0**: Spawn Slint rendering task
    info!("Spawning Slint rendering task on Core 0");
    spawner
        .spawn(slint_rendering_task(window.clone(), ui.as_weak()))
        .ok();

    // Show the window
    ui.show().unwrap();

    info!("=== All systems initialized, dual-core active ===");
    info!("Core 0: WiFi + Slint rendering");
    info!("Core 1: DMA display output");

    // Main loop - keep the main task alive (Core 0)
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        ticker.next().await;
        info!("Main task alive - WiFi scanning and Slint rendering on Core 0");
    }
}
