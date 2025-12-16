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
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::rtc_cntl::{Rtc, sleep::TimerWakeupSource};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::SmartLedsAdapter;
use log::info;
use pomodoro_cube::qmi8658;
use smart_leds::{RGB8, SmartLedsWrite};

use core::time::Duration as CoreDuration;

// --- CONFIGURATION CONSTANTS ---
const COLOR_5_MIN: RGB8 = RGB8 { r: 0, g: 10, b: 0 };
const COLOR_15_MIN: RGB8 = RGB8 { r: 0, g: 0, b: 10 };
const COLOR_30_MIN: RGB8 = RGB8 { r: 10, g: 10, b: 0 };
const COLOR_60_MIN: RGB8 = RGB8 { r: 10, g: 0, b: 0 };
const COLOR_BG: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const GRAVITY_THRESHOLD: f32 = 0.85;
// Buffer Size: 64 LEDs * 24 bits (R,G,B) + 1 Stop Code
const BUFFER_SIZE: usize = 64 * 24 + 1;

// Light sleep can disrupt `espflash monitor` (USB-Serial-JTAG/CDC) because the USB link may drop
// while the chip sleeps. Keep this OFF while developing/debugging over USB.
const USE_LIGHT_SLEEP: bool = false;

#[derive(Copy, Clone, Debug)]
enum FillDirection {
    Down,
    Up,
    Left,
    Right,
}

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
    info!("App booted; initializing...");

    // System Init
    // Set CPU to 80MHz to save power
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);

    // RTC (for true light sleep)
    let mut rtc = Rtc::new(peripherals.LPWR);

    // Runtime Init
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Give the host time to enumerate/attach a serial monitor after reset/flash.
    // This wait should be done after esp_rtos::start to ensure the timer is running.
    Timer::after(Duration::from_secs(4)).await;

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
                // ACTION: 5 MINUTES
                info!("5 MINUTES");
                flash_color(&mut led_strip, COLOR_5_MIN).await;
                run_timer(
                    5,
                    &mut led_strip,
                    COLOR_5_MIN,
                    FillDirection::Right,
                    &mut rtc,
                )
                .await;
            } else if x < -GRAVITY_THRESHOLD {
                // --- CASE 2: X- is DOWN ---
                // ACTION: 15 MINUTES
                info!("15 MINUTES");
                flash_color(&mut led_strip, COLOR_15_MIN).await;
                run_timer(
                    15,
                    &mut led_strip,
                    COLOR_15_MIN,
                    FillDirection::Up,
                    &mut rtc,
                )
                .await;
            } else if x > GRAVITY_THRESHOLD {
                // --- CASE 3: X+ is DOWN ---
                // ACTION: 30 MINUTES
                info!("30 MINUTES");
                flash_color(&mut led_strip, COLOR_30_MIN).await;
                run_timer(
                    30,
                    &mut led_strip,
                    COLOR_30_MIN,
                    FillDirection::Down,
                    &mut rtc,
                )
                .await;
            } else if y > GRAVITY_THRESHOLD {
                // --- CASE 4: Y+ is DOWN ---
                // ACTION: 60 MINUTES
                info!("60 MINUTES");
                flash_color(&mut led_strip, COLOR_60_MIN).await;
                run_timer(
                    60,
                    &mut led_strip,
                    COLOR_60_MIN,
                    FillDirection::Left,
                    &mut rtc,
                )
                .await;
            } else if !(-GRAVITY_THRESHOLD..=GRAVITY_THRESHOLD).contains(&z) {
                // --- CASE 5: Z is DOWN/UP ---
                // ACTION: Do nothing / ignore
                info!("IGNORING Z-AXIS ORIENTATION");
                //TODO: Maybe go to light sleep until orientation changes?
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

// TODO: Go to light or deep sleep after the grain reaches its position till next grain needs to appear
// TODO: Implement light or deep sleep between readings and after completion to save power. Wake up if orientation changes
// TODO: Stop the timer after it completes and wait for user to move the cube to restart
// TODO: Detect if orientation changed to another valid one during countdown, and pause timer until settled
//   and use the new orientation to restart the timer if the orientation changed from previous one
async fn run_timer<S>(
    minutes: u32,
    leds: &mut S,
    color: RGB8,
    dir: FillDirection,
    rtc: &mut Rtc<'_>,
) where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    let total_seconds = minutes * 60;

    // Sand animation strategy:
    // - Only wake when the next "grain" (pixel) needs to be added.
    // - Animate a single grain falling toward its target position.
    // This approximates "sleep until next grain" without forcing deep-sleep resets.

    // Initial render: empty matrix.
    render_sand_state(leds, 0, None, color, dir);

    let mut elapsed: u32 = 0;
    while elapsed < total_seconds {
        let filled_now = filled_pixels(elapsed, total_seconds);
        if filled_now >= 64 {
            break;
        }

        // Compute when the next pixel should be added (ceil division).
        let next_elapsed = next_elapsed_for_filled(filled_now + 1, total_seconds);
        if next_elapsed <= elapsed {
            // Shouldn't happen, but avoid getting stuck.
            elapsed += 1;
            continue;
        }

        sleep_until_next_grain(rtc, next_elapsed - elapsed).await;
        elapsed = next_elapsed;

        let filled_after = filled_pixels(elapsed, total_seconds);
        if filled_after > filled_now {
            let target_fill_index = filled_after - 1;
            animate_grain_to_fill_index(leds, target_fill_index, filled_after, color, dir).await;
        }
    }

    run_breathing_animation(leds, color).await;
}

fn light_sleep_secs(rtc: &mut Rtc<'_>, seconds: u32) {
    if seconds == 0 {
        return;
    }

    let timer = TimerWakeupSource::new(CoreDuration::from_secs(seconds as u64));
    Delay::new().delay_millis(100);
    rtc.sleep_light(&[&timer]);
}

async fn sleep_until_next_grain(rtc: &mut Rtc<'_>, seconds: u32) {
    if seconds == 0 {
        return;
    }

    if USE_LIGHT_SLEEP {
        light_sleep_secs(rtc, seconds);
    } else {
        Timer::after(Duration::from_secs(seconds as u64)).await;
    }
}

fn filled_pixels(elapsed: u32, total_seconds: u32) -> usize {
    // Integer progress mapping: 0..64 over 0..total_seconds.
    // At elapsed == total_seconds, this becomes 64.
    ((elapsed as u64 * 64) / (total_seconds as u64)) as usize
}

fn next_elapsed_for_filled(next_filled: usize, total_seconds: u32) -> u32 {
    // Solve for minimal t such that floor(t*64/total_seconds) >= next_filled.
    // t >= ceil(next_filled*total_seconds/64)
    let numerator = (next_filled as u64) * (total_seconds as u64);
    numerator.div_ceil(64) as u32
}

async fn animate_grain_to_fill_index<S>(
    leds: &mut S,
    target_fill_index: usize,
    filled_count: usize,
    color: RGB8,
    dir: FillDirection,
) where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    // Determine the target cell for this grain.
    let (target_row, target_col) = coord_for_fill_index(target_fill_index, dir);

    // Start on the side opposite gravity, then fall toward the target.
    let (mut row, mut col) = match dir {
        FillDirection::Down => (0usize, target_col),
        FillDirection::Up => (7usize, target_col),
        FillDirection::Left => (target_row, 7usize),
        FillDirection::Right => (target_row, 0usize),
    };

    // Step toward the target with a short animation.
    // Keep total animation short to preserve the "sleepy" behavior.
    for _ in 0..8 {
        render_sand_state(
            leds,
            filled_count.saturating_sub(1),
            Some((row, col)),
            color,
            dir,
        );

        if row == target_row && col == target_col {
            break;
        }

        match dir {
            FillDirection::Down => {
                if row < target_row {
                    row += 1;
                }
            }
            FillDirection::Up => {
                if row > target_row {
                    row -= 1;
                }
            }
            FillDirection::Left => {
                if col > target_col {
                    col -= 1;
                }
            }
            FillDirection::Right => {
                if col < target_col {
                    col += 1;
                }
            }
        }

        Timer::after(Duration::from_millis(35)).await;
    }

    // Final state with the new pixel committed.
    render_sand_state(leds, filled_count, None, color, dir);
}

fn render_sand_state<S>(
    leds: &mut S,
    filled_count: usize,
    grain: Option<(usize, usize)>,
    color: RGB8,
    dir: FillDirection,
) where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
{
    let mut pixels = [COLOR_BG; 64];

    // Fill the "pile" starting from the gravity side.
    let fill_limit = core::cmp::min(filled_count, 64);
    for fill_index in 0..fill_limit {
        let (row, col) = coord_for_fill_index(fill_index, dir);
        let idx = physical_index(row, col);
        pixels[idx] = color;
    }

    // Draw the falling grain on top.
    if let Some((row, col)) = grain
        && row < 8
        && col < 8
    {
        let idx = physical_index(row, col);
        pixels[idx] = color;
    }

    let _ = leds.write(pixels.iter().copied());
}

fn coord_for_fill_index(fill_index: usize, dir: FillDirection) -> (usize, usize) {
    // Maps a linear "pile growth" index (0..63) to a row/col based on gravity direction.
    // Assumes an 8x8 logical matrix.
    let major = fill_index / 8;
    let minor = fill_index % 8;

    match dir {
        // Gravity pulls toward row 7, so we fill row 7 -> 0.
        FillDirection::Down => (7 - major, minor),
        // Gravity pulls toward row 0, so we fill row 0 -> 7.
        FillDirection::Up => (major, minor),
        // Gravity pulls toward col 0, so we fill col 0 -> 7.
        FillDirection::Left => (minor, major),
        // Gravity pulls toward col 7, so we fill col 7 -> 0.
        FillDirection::Right => (minor, 7 - major),
    }
}

fn physical_index(row: usize, col: usize) -> usize {
    // Logical row/col to LED buffer index.
    // Assumes simple row-major wiring (0..7 left->right, then next row).
    // If your matrix is serpentine, adjust this mapping.
    (row * 8) + col
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
