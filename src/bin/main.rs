#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::SmartLedsAdapter;
use log::info;
use pomodoro_cube::qmi8658;
use smart_leds::{RGB8, SmartLedsWrite};

// --- CONFIGURATION CONSTANTS ---
const COLOR_5_MIN: RGB8 = RGB8 { r: 0, g: 20, b: 0 };
const COLOR_30_MIN: RGB8 = RGB8 { r: 20, g: 10, b: 0 };
const COLOR_60_MIN: RGB8 = RGB8 { r: 20, g: 0, b: 0 };
const COLOR_BG: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const GRAVITY_THRESHOLD: f32 = 0.85;
// Buffer Size: 64 LEDs * 24 bits (R,G,B) + 1 Stop Code
const BUFFER_SIZE: usize = 64 * 24 + 1;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.1.0

    esp_println::logger::init_logger_from_env();

    // System Init
    // Set CPU to 80MHz to save power
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);

    // Runtime Init
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Hardware Init
    //LEDs
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT0");
    let mut rmt_buffer = [PulseCode::default(); BUFFER_SIZE];
    let mut led_strip = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO14, &mut rmt_buffer);

    // IMU
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("Failed to create I2c")
    .with_sda(peripherals.GPIO11)
    .with_scl(peripherals.GPIO12);
    let mut imu = qmi8658::Qmi8658::new(i2c);
    if let Err(e) = imu.init() {
        panic!("Failed to initialize QMI8658: {:?}", e);
    } else {
        info!("QMI8658 initialized successfully");
    }

    info!("Initialized!");

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        // Read all 3 axes (X, Y, Z)
        if let Ok((x, y, z)) = imu.read_accel_all() {
            // Check which axis is feeling gravity (> 0.85g or < -0.85g)
            // Gravity pulls "Down", so the sensor reads +1g or -1g depending on orientation.
            info!("X: {:.2}g, Y: {:.2}g, Z: {:.2}g", x, y, z);

            // TODO: Maybe use Z axis for power down to give an additional time option?
            // TODO: Batch logging to reduce log spam
            if y < -GRAVITY_THRESHOLD {
                // --- CASE 1: Y- is DOWN ---
                // ACTION: OFF / IDLE
                fill_matrix(&mut led_strip, COLOR_BG);
                Timer::after(Duration::from_millis(500)).await;
                info!("IDLE");
            } else if x < -GRAVITY_THRESHOLD {
                // --- CASE 2: X- is DOWN ---
                // ACTION: 5 MINUTES
                info!("5 MINUTES");
                flash_color(&mut led_strip, COLOR_5_MIN).await;
                run_timer(5, &mut led_strip, COLOR_5_MIN).await;
            } else if x > GRAVITY_THRESHOLD {
                // --- CASE 3: X+ is DOWN ---
                // ACTION: 30 MINUTES
                info!("30 MINUTES");
                flash_color(&mut led_strip, COLOR_30_MIN).await;
                run_timer(30, &mut led_strip, COLOR_30_MIN).await;
            } else if y > GRAVITY_THRESHOLD {
                // --- CASE 4: Y+ is DOWN ---
                // ACTION: 60 MINUTES
                info!("60 MINUTES");
                flash_color(&mut led_strip, COLOR_60_MIN).await;
                run_timer(60, &mut led_strip, COLOR_60_MIN).await;
            } else {
                // --- AMBIGUOUS / TRANSITION ---
                // Waiting for box to settle
                info!("SETTLING");
                Timer::after(Duration::from_millis(200)).await;
            }
        } else {
            // Sensor error retry
            info!("Sensor read error, retrying...");
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

// TODO: Implement a sand grain falling animation during the timer countdown
// TODO: Implement orientation based direction of filling the LEDs (so it fills "downwards" based on cube orientation)
// TODO: Implement light or deep sleep between readings and after completion to save power. Wake up if orientation changes
// TODO: Detect if orientation changed to another valid one during countdown, and pause timer until settled
//   and use the new orientation to restart the timer if the orientation changed from previous one
async fn run_timer<S>(minutes: u32, leds: &mut S, color: RGB8)
where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    let total_seconds = minutes * 60;

    for elapsed in 0..total_seconds {
        let percent = elapsed as f32 / total_seconds as f32;
        let pixels_filled = (percent * 64.0) as usize;

        let mut pixels = [COLOR_BG; 64];

        // This fills pixels 0->63 regardless of cube rotation.
        for (idx, pixel) in pixels.iter_mut().enumerate() {
            if idx < pixels_filled {
                *pixel = color;
            }
        }

        // Heartbeat
        if elapsed % 2 == 0 {
            pixels[63] = RGB8 { r: 5, g: 5, b: 5 };
        }

        let _ = leds.write(pixels.iter().copied());
        Timer::after(Duration::from_secs(1)).await;
    }

    run_breathing_animation(leds, color).await;
}

async fn run_breathing_animation<S>(leds: &mut S, color: RGB8)
where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    for _ in 0..3 {
        fill_matrix(leds, color);
        Timer::after(Duration::from_millis(500)).await;
        fill_matrix(leds, COLOR_BG);
        Timer::after(Duration::from_millis(500)).await;
    }
}

async fn flash_color<S>(leds: &mut S, color: RGB8)
where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    fill_matrix(leds, color);
    Timer::after(Duration::from_millis(300)).await;
    fill_matrix(leds, COLOR_BG);
    Timer::after(Duration::from_millis(100)).await;
}

fn fill_matrix<S>(leds: &mut S, color: RGB8)
where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    let pixels = [color; 64];
    let _ = leds.write(pixels.iter().copied());
}
