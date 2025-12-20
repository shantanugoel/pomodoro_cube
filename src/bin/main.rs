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
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::rtc_cntl::{Rtc, sleep::TimerWakeupSource};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::SmartLedsAdapter;
use log::info;
use pomodoro_cube::qmi8658::{self, WomInterrupt};
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

// Wake-on-Motion threshold in mg (1mg/LSB). Lower = more sensitive.
// 100mg is a good balance for detecting cube rotation without false triggers.
const WOM_THRESHOLD_MG: u8 = 100;
// Number of accelerometer samples to ignore when enabling WoM (avoids spurious triggers)
const WOM_BLANKING_SAMPLES: u8 = 10;

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

/// Represents the detected cube orientation based on accelerometer readings.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Orientation {
    /// Y- is down (5 minutes)
    YNeg,
    /// X- is down (15 minutes)
    XNeg,
    /// X+ is down (30 minutes)
    XPos,
    /// Y+ is down (60 minutes)
    YPos,
    /// Z axis is dominant (ignored)
    ZAxis,
    /// Ambiguous/transitioning
    Unknown,
}

impl Orientation {
    /// Get timer duration in minutes for this orientation, if valid.
    fn timer_minutes(self) -> Option<u32> {
        match self {
            Orientation::YNeg => Some(5),
            Orientation::XNeg => Some(15),
            Orientation::XPos => Some(30),
            Orientation::YPos => Some(60),
            Orientation::ZAxis | Orientation::Unknown => None,
        }
    }

    /// Get the fill direction for LED animation.
    fn fill_direction(self) -> FillDirection {
        match self {
            Orientation::YNeg => FillDirection::Right,
            Orientation::XNeg => FillDirection::Up,
            Orientation::XPos => FillDirection::Down,
            Orientation::YPos => FillDirection::Left,
            Orientation::ZAxis | Orientation::Unknown => FillDirection::Down,
        }
    }

    /// Get the LED color for this orientation.
    fn color(self) -> RGB8 {
        match self {
            Orientation::YNeg => COLOR_5_MIN,
            Orientation::XNeg => COLOR_15_MIN,
            Orientation::XPos => COLOR_30_MIN,
            Orientation::YPos => COLOR_60_MIN,
            Orientation::ZAxis | Orientation::Unknown => COLOR_BG,
        }
    }

    /// Returns true if this is a valid timer orientation.
    fn is_valid_timer_orientation(self) -> bool {
        self.timer_minutes().is_some()
    }
}

/// Determine orientation from accelerometer readings.
fn detect_orientation(x: f32, y: f32, z: f32) -> Orientation {
    if y < -GRAVITY_THRESHOLD {
        Orientation::YNeg
    } else if x < -GRAVITY_THRESHOLD {
        Orientation::XNeg
    } else if x > GRAVITY_THRESHOLD {
        Orientation::XPos
    } else if y > GRAVITY_THRESHOLD {
        Orientation::YPos
    } else if !(-GRAVITY_THRESHOLD..=GRAVITY_THRESHOLD).contains(&z) {
        Orientation::ZAxis
    } else {
        Orientation::Unknown
    }
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

    // IMU interrupt pin (INT2) - used for Wake-on-Motion
    // Common pins on Waveshare ESP32-S3-Matrix: GPIO10 or GPIO13
    let mut imu_int = Input::new(
        peripherals.GPIO13,
        InputConfig::default().with_pull(Pull::Down),
    );

    info!("Initialized!");

    let _ = spawner;

    // On first boot, wait for user to pick up/move the cube before starting
    info!("Waiting for initial motion to start...");
    wait_for_motion_wom(&mut imu, &mut imu_int).await;
    info!("Motion detected, ready to start!");

    loop {
        // Wait for a valid orientation to start a timer
        let orientation = wait_for_valid_orientation(&mut imu).await;

        let minutes = orientation.timer_minutes().unwrap();
        let color = orientation.color();

        info!("{} MINUTES timer starting", minutes);
        flash_color(&mut led_strip, color).await;

        // Run the timer with orientation change detection
        let result = run_timer_with_motion_detection(
            minutes,
            orientation,
            &mut led_strip,
            &mut imu,
            &mut rtc,
        )
        .await;

        match result {
            TimerResult::Completed => {
                info!("Timer completed! Waiting for cube to be moved...");
                // Timer finished - wait for user to move the cube before allowing restart
                wait_for_motion_wom(&mut imu, &mut imu_int).await;
                info!("Motion detected, checking new orientation...");
            }
            TimerResult::OrientationChanged(new_orientation) => {
                info!(
                    "Orientation changed during timer to {:?}, restarting...",
                    new_orientation
                );
                // The main loop will pick up the new orientation on next iteration
            }
        }
    }
}

/// Result of a timer run
enum TimerResult {
    /// Timer completed successfully
    Completed,
    /// Orientation changed during countdown to a new valid orientation
    OrientationChanged(Orientation),
}

/// Wait until the cube is in a valid timer orientation (stable).
async fn wait_for_valid_orientation<I, E>(imu: &mut qmi8658::Qmi8658<I>) -> Orientation
where
    I: embedded_hal::i2c::I2c<Error = E>,
{
    loop {
        if let Ok((x, y, z)) = imu.read_accel_all() {
            let orientation = detect_orientation(x, y, z);
            if orientation.is_valid_timer_orientation() {
                // Wait a bit to confirm it's stable
                Timer::after(Duration::from_millis(300)).await;

                // Re-check
                if let Ok((x2, y2, z2)) = imu.read_accel_all() {
                    let orientation2 = detect_orientation(x2, y2, z2);
                    if orientation == orientation2 {
                        return orientation;
                    }
                }
            } else {
                match orientation {
                    Orientation::ZAxis => {
                        info!("Z-axis orientation (ignored), waiting...");
                    }
                    Orientation::Unknown => {
                        info!("Settling...");
                    }
                    _ => {}
                }
            }
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// Enable Wake-on-Motion and wait for an interrupt indicating motion.
/// Falls back to polling if interrupt doesn't arrive within timeout.
#[allow(clippy::large_stack_frames)]
async fn wait_for_motion_wom<I, E>(imu: &mut qmi8658::Qmi8658<I>, int_pin: &mut Input<'_>)
where
    I: embedded_hal::i2c::I2c<Error = E>,
{
    // Enable WoM on INT2 (initial value low, so it goes high on motion)
    if let Err(_e) = imu.enable_wake_on_motion(
        WOM_THRESHOLD_MG,
        WomInterrupt::Int2Low,
        WOM_BLANKING_SAMPLES,
    ) {
        info!("Failed to enable WoM, falling back to polling");
        wait_for_motion_polling(imu).await;
        return;
    }

    info!("WoM enabled, waiting for motion interrupt...");
    info!("If no interrupt arrives in 5 seconds, will fall back to polling");

    // Wait for interrupt with timeout
    // The INT pin might not be connected on all boards
    let interrupt_received =
        embassy_time::with_timeout(Duration::from_secs(5), int_pin.wait_for_any_edge())
            .await
            .is_ok();

    if interrupt_received {
        info!("WoM interrupt received!");
        // Clear the WoM status by reading STATUS1
        let _ = imu.check_wom_event();
    } else {
        info!("No interrupt received - INT pin may not be connected");
        info!("Falling back to polling mode for motion detection");

        // Disable WoM first
        if let Err(_e) = imu.disable_wake_on_motion() {
            info!("Failed to disable WoM");
        }

        // Fall back to polling
        wait_for_motion_polling(imu).await;
        return;
    }

    // Disable WoM and restore normal operation
    if let Err(_e) = imu.disable_wake_on_motion() {
        info!("Failed to disable WoM, continuing anyway");
    }
}

/// Fallback method: Poll accelerometer to detect motion when interrupt isn't available.
async fn wait_for_motion_polling<I, E>(imu: &mut qmi8658::Qmi8658<I>)
where
    I: embedded_hal::i2c::I2c<Error = E>,
{
    // Read baseline
    let baseline = if let Ok((x, y, z)) = imu.read_accel_all() {
        (x, y, z)
    } else {
        info!("Failed to read baseline, waiting 1s");
        Timer::after(Duration::from_secs(1)).await;
        return;
    };

    info!("Polling for motion (move the cube to continue)...");

    // Poll for significant change
    loop {
        Timer::after(Duration::from_millis(100)).await;

        if let Ok((x, y, z)) = imu.read_accel_all() {
            // Check if any axis changed significantly (> 0.3g = 30% of gravity)
            let dx = (x - baseline.0).abs();
            let dy = (y - baseline.1).abs();
            let dz = (z - baseline.2).abs();

            if dx > 0.3 || dy > 0.3 || dz > 0.3 {
                info!("Motion detected via polling!");
                return;
            }
        }
    }
}

/// Run the timer with periodic orientation change detection.
/// Returns `TimerResult::Completed` if the timer finishes normally,
/// or `TimerResult::OrientationChanged(new_orientation)` if the cube was rotated.
#[allow(clippy::large_stack_frames)]
async fn run_timer_with_motion_detection<S, I, E>(
    minutes: u32,
    original_orientation: Orientation,
    leds: &mut S,
    imu: &mut qmi8658::Qmi8658<I>,
    rtc: &mut Rtc<'_>,
) -> TimerResult
where
    S: SmartLedsWrite,
    S::Color: From<RGB8>,
    I: embedded_hal::i2c::I2c<Error = E>,
{
    let color = original_orientation.color();
    let dir = original_orientation.fill_direction();
    let total_seconds = minutes * 60;

    // Initial render: empty matrix.
    render_sand_state(leds, 0, None, color, dir);

    // Immediately animate the first pixel for instant visual feedback.
    animate_grain_to_fill_index(leds, 0, 1, color, dir).await;

    let mut elapsed: u32 = 0;

    while elapsed < total_seconds {
        let filled_now = filled_pixels(elapsed, total_seconds);
        if filled_now >= 64 {
            break;
        }

        // Compute when the next pixel should be added.
        let next_elapsed = next_elapsed_for_filled(filled_now + 1, total_seconds);
        if next_elapsed <= elapsed {
            elapsed += 1;
            continue;
        }

        let sleep_duration = next_elapsed - elapsed;

        // Instead of sleeping the full duration, sleep in shorter intervals
        // and check orientation periodically (every ~1 second).
        let check_interval = 1u32; // Check orientation every second
        let mut remaining = sleep_duration;

        while remaining > 0 {
            let sleep_time = remaining.min(check_interval);

            if USE_LIGHT_SLEEP && sleep_time > 1 {
                light_sleep_secs(rtc, sleep_time);
            } else {
                Timer::after(Duration::from_secs(sleep_time as u64)).await;
            }

            remaining = remaining.saturating_sub(sleep_time);

            // Check if orientation changed
            if let Ok((x, y, z)) = imu.read_accel_all() {
                let current_orientation = detect_orientation(x, y, z);

                if current_orientation != original_orientation {
                    if current_orientation.is_valid_timer_orientation() {
                        // Orientation changed to a different valid position
                        // Wait briefly to confirm it's stable
                        Timer::after(Duration::from_millis(300)).await;

                        if let Ok((x2, y2, z2)) = imu.read_accel_all() {
                            let confirmed_orientation = detect_orientation(x2, y2, z2);
                            if confirmed_orientation == current_orientation {
                                info!(
                                    "Orientation changed from {:?} to {:?}",
                                    original_orientation, current_orientation
                                );
                                // Clear LEDs before returning
                                fill_matrix(leds, COLOR_BG);
                                return TimerResult::OrientationChanged(current_orientation);
                            }
                        }
                    } else if current_orientation == Orientation::ZAxis {
                        // Cube flipped to Z-axis (stop position) - pause timer and wait
                        info!("Z-axis detected - pausing timer...");
                        fill_matrix(leds, COLOR_BG);

                        // Wait for cube to settle back to original orientation or a new valid one
                        loop {
                            Timer::after(Duration::from_millis(200)).await;
                            if let Ok((x3, y3, z3)) = imu.read_accel_all() {
                                let settled_orientation = detect_orientation(x3, y3, z3);
                                if settled_orientation == original_orientation {
                                    // Back to original - resume timer
                                    info!("Back to original orientation, resuming timer...");
                                    render_sand_state(leds, filled_now, None, color, dir);
                                    break;
                                } else if settled_orientation.is_valid_timer_orientation() {
                                    // Settled to a different timer orientation
                                    info!("Changed to new orientation, restarting timer...");
                                    return TimerResult::OrientationChanged(settled_orientation);
                                } else if settled_orientation == Orientation::ZAxis {
                                    // Still on Z-axis, keep waiting
                                    continue;
                                }
                                // Unknown orientation, keep waiting to settle
                            }
                        }
                    } else if current_orientation == Orientation::Unknown {
                        // Cube is being moved - pause and wait for it to settle
                        info!("Cube moving, pausing timer...");
                        fill_matrix(leds, COLOR_BG);

                        // Wait for cube to settle back to original orientation or a new one
                        loop {
                            Timer::after(Duration::from_millis(200)).await;
                            if let Ok((x3, y3, z3)) = imu.read_accel_all() {
                                let settled_orientation = detect_orientation(x3, y3, z3);
                                if settled_orientation == original_orientation {
                                    // Back to original - resume timer
                                    info!("Back to original orientation, resuming...");
                                    render_sand_state(leds, filled_now, None, color, dir);
                                    break;
                                } else if settled_orientation.is_valid_timer_orientation() {
                                    // Settled to a different orientation
                                    return TimerResult::OrientationChanged(settled_orientation);
                                } else if settled_orientation == Orientation::ZAxis {
                                    // Settled to Z-axis (stop position)
                                    info!("Z-axis detected - timer paused");
                                    // Will loop and keep waiting
                                }
                                // Still settling or unknown, keep waiting
                            }
                        }
                    }
                    // Other orientations handled above
                }
            }
        }

        elapsed = next_elapsed;

        let filled_after = filled_pixels(elapsed, total_seconds);
        if filled_after > filled_now {
            let target_fill_index = filled_after - 1;
            animate_grain_to_fill_index(leds, target_fill_index, filled_after, color, dir).await;
        }
    }

    // Timer completed successfully
    run_breathing_animation(leds, color).await;
    TimerResult::Completed
}

#[allow(dead_code)]
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
    // First pixel shown immediately, remaining 63 distributed over full duration.
    // At elapsed == 0: returns 1
    // At elapsed == total_seconds: returns 64
    1 + ((elapsed as u64 * 63) / (total_seconds as u64)) as usize
}

fn next_elapsed_for_filled(next_filled: usize, total_seconds: u32) -> u32 {
    // First pixel is at t=0, so for pixels 2-64 we use 63 intervals.
    // Solve for minimal t such that 1 + floor(t*63/total_seconds) >= next_filled.
    // t >= ceil((next_filled - 1)*total_seconds/63)
    if next_filled <= 1 {
        return 0;
    }
    let numerator = ((next_filled - 1) as u64) * (total_seconds as u64);
    numerator.div_ceil(63) as u32
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
