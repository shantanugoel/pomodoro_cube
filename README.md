# Pomodoro Cube

A physical timer cube with visual feedback designed to help with time management and focus. Place the cube on different sides to start timers of varying durations, with an LED matrix showing progress through a falling-sand animation.

## Hardware Requirements

- **Waveshare ESP32-S3-Matrix** development board
  - Includes built-in 8x8 WS2812B LED matrix (GPIO14)
  - ESP32-S3 microcontroller
  - USB-C connector for power and programming

- **QMI8658 IMU** (accelerometer/gyroscope)
  - Connected via I2C on GPIO11 (SDA) and GPIO12 (SCL)
  - Optional interrupt pin on GPIO13 for motion detection

## Timer Durations

The cube detects which side is facing down and starts the corresponding timer:

- **Y- face down**: 5 minutes (green)
- **X- face down**: 15 minutes (blue)
- **X+ face down**: 30 minutes (yellow)
- **Y+ face down**: 60 minutes (red)

When you flip the cube to a different valid side during a timer, it restarts with the new duration. Placing the cube on its Z-axis face (top/bottom) pauses the timer.

## Development Setup

### Prerequisites

1. Install Rust:
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```

2. Install cargo-binstall for faster tool installation:
   ```bash
   curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash
   ```

3. Install espup to manage ESP32 Rust toolchains:
   ```bash
   cargo binstall espup
   ```

4. Install the ESP32 toolchain:
   ```bash
   espup install
   ```
   This installs the Xtensa Rust compiler and tools needed for ESP32-S3.

5. Source the environment (run this in each new terminal session, or add to your shell profile):
   ```bash
   source $HOME/export-esp.sh
   ```

6. Install cargo-espflash for flashing and monitoring:
   ```bash
   cargo binstall cargo-espflash
   ```

### Building

Build the project with:
```bash
cargo build
```

Or use the provided build task if working in VS Code:
```bash
cargo build
```

### Flashing and Monitoring

To flash the firmware and monitor serial output:
```bash
cargo espflash flash --monitor
```

For release builds (smaller size, better performance):
```bash
cargo espflash flash --release --monitor
```

The cargo espflash command automatically builds the project before flashing.

### Development Notes

- Set `USE_LIGHT_SLEEP = false` in the code while developing, as light sleep can disrupt the USB serial connection used for monitoring.
  - This is currently experimental so suggest to keep it false till this is fixed fully. I am currently seeing crashes with this.
- The CPU runs at 80MHz to conserve power while maintaining smooth LED animations.
- Log messages are sent over USB-Serial-JTAG and can be viewed with `cargo espflash monitor`.

## Usage

1. **Power on**: Connect the cube via USB-C or use a battery pack.

2. **Initial setup**: The device waits for you to move the cube after power-on. This confirms the IMU is working and you're ready to start.

3. **Start a timer**: Place the cube with your desired duration's face down. The LEDs will flash briefly in the corresponding color, then the first pixel will drop, indicating the timer has started.

4. **Visual feedback**: Watch the LED matrix fill up with a falling-sand animation. Each pixel represents progress toward completion.

5. **Pause**: Flip the cube onto its top or bottom face (Z-axis). The LEDs turn off and the timer pauses until you place it back on a valid timer face.

6. **Change duration**: Flip to a different valid face during a timer to restart with the new duration.

7. **Completion**: When the timer finishes, the LED matrix displays a breathing animation in the timer's color. Move the cube to acknowledge and prepare for the next session.

## Hardware Connections

If building your own or troubleshooting:

| Component | GPIO Pin | Notes |
|-----------|----------|-------|
| LED Matrix (WS2812B) | GPIO14 | Uses RMT peripheral, 64 LEDs |
| IMU SDA | GPIO11 | I2C data line, 400kHz |
| IMU SCL | GPIO12 | I2C clock line |
| IMU INT2 | GPIO13 | Optional, for wake-on-motion |

Pull-down resistor is configured in software for the interrupt pin.

## Troubleshooting

**No LED activity**: Check that GPIO14 is correctly connected to the LED matrix data line. Verify power supply can provide sufficient current (up to 3.8A at full brightness for all 64 LEDs, though this firmware uses low brightness).

**Timer doesn't start**: Ensure the IMU is properly connected and functioning. Check serial logs for initialization messages. The cube should detect orientation within a second of being placed on a face.

**Unexpected timer behavior**: The accelerometer threshold is set to 0.85g. If the cube is on an unlevel surface or experiencing vibration, it may detect false orientation changes. Place on a stable, level surface.

**Serial monitor disconnects**: If you enabled light sleep, the USB connection may drop during sleep periods. This is normal and documented in the code. Set `USE_LIGHT_SLEEP = false` for stable monitoring during development.

## TODO
While the cube is now fully functional, it has a few improvements that can be made as below:
- [ ] Fix Light sleep issues
- [ ] Add timer stopping and deep sleep for saving power further
- [ ] Fix interrupt issues. Currently it always falls back to polling
- [ ] Add hardware updates for battery and switch connections etc
