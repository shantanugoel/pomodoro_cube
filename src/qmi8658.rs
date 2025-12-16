use embedded_hal::i2c::I2c;

pub const QMI8658_ADDRESS_LOW: u8 = 0x6A;
pub const QMI8658_ADDRESS_HIGH: u8 = 0x6B;

const REG_WHO_AM_I: u8 = 0x00;
const REG_CTRL1: u8 = 0x02;
const REG_CTRL2: u8 = 0x03;
const REG_CTRL7: u8 = 0x08;
const REG_CTRL9: u8 = 0x0A;

// Calibration registers used for WoM configuration
const REG_CAL1_L: u8 = 0x0B; // WoM threshold (mg, 1mg/LSB)
const REG_CAL1_H: u8 = 0x0C; // WoM interrupt select (bits 7:6) + blanking time (bits 5:0)

#[allow(dead_code)]
const REG_STATUS0: u8 = 0x2E;
const REG_STATUS1: u8 = 0x2F;

const REG_ACC_X_L: u8 = 0x35;
const REG_ACC_Z_L: u8 = 0x39;

const WHO_AM_I_EXPECTED: u8 = 0x05;

// CTRL9 commands
const CTRL9_CMD_WOM_SETTING: u8 = 0x08;

// Accelerometer ranges
const ACCEL_RANGE_2G: u8 = 0x00;
const ACCEL_RANGE_8G: u8 = 0x02;

// Low-power ODRs live in the low nibble as 0x0C..0x0F in the reference driver.
// Pick 11Hz as a close low-power rate
const ACCEL_ODR_LOWPOWER_11HZ: u8 = 0x0E;
const ACCEL_ODR_LOWPOWER_21HZ: u8 = 0x0D;

// Sensitivity for different ranges
const ACCEL_LSB_PER_G_2G: f32 = 16384.0;
const ACCEL_LSB_PER_G_8G: f32 = 4096.0;

/// Which interrupt pin to use for Wake-on-Motion
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WomInterrupt {
    /// INT1 with initial value 0
    Int1Low,
    /// INT1 with initial value 1
    Int1High,
    /// INT2 with initial value 0
    Int2Low,
    /// INT2 with initial value 1
    Int2High,
}

impl WomInterrupt {
    fn to_bits(self) -> u8 {
        match self {
            WomInterrupt::Int1Low => 0b00,
            WomInterrupt::Int1High => 0b10,
            WomInterrupt::Int2Low => 0b01,
            WomInterrupt::Int2High => 0b11,
        }
    }
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidWhoAmI(u8),
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::I2c(value)
    }
}

pub struct Qmi8658<I> {
    i2c: I,
    addr: u8,
    accel_lsb_per_g: f32,
}

impl<I, E> Qmi8658<I>
where
    I: I2c<Error = E>,
{
    // ESP32 S3 Matrix board from waveshare uses the high address 0x6B
    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            addr: QMI8658_ADDRESS_HIGH,
            accel_lsb_per_g: ACCEL_LSB_PER_G_8G,
        }
    }

    // Just in case if needed for another board.
    pub fn new_with_addr(i2c: I, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            accel_lsb_per_g: ACCEL_LSB_PER_G_8G,
        }
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Sanity-check the sensor is responding at this address.
        let who = self.read_who_am_i()?;
        if who != WHO_AM_I_EXPECTED {
            return Err(Error::InvalidWhoAmI(who));
        }

        // Reference driver starts by writing CTRL1=0x60.
        // (This is required for normal operation; accel ODR/range are configured in CTRL2.)
        self.i2c.write(self.addr, &[REG_CTRL1, 0x60])?;

        // Configure accel: range in upper nibble, ODR in lower nibble.
        let ctrl2 = (ACCEL_RANGE_8G << 4) | ACCEL_ODR_LOWPOWER_11HZ;
        self.i2c.write(self.addr, &[REG_CTRL2, ctrl2])?;

        // Enable accel only (CTRL7 bit0).
        self.i2c.write(self.addr, &[REG_CTRL7, 0x01])?;

        Ok(())
    }

    pub fn read_who_am_i(&mut self) -> Result<u8, Error<E>> {
        let mut data = [0u8; 1];
        self.i2c.write_read(self.addr, &[REG_WHO_AM_I], &mut data)?;
        Ok(data[0])
    }

    pub fn read_z_gravity(&mut self) -> Result<f32, Error<E>> {
        let mut data = [0u8; 2];
        // Read 2 bytes starting from REG_ACC_Z_L (0x39, 0x3A)
        self.i2c.write_read(self.addr, &[REG_ACC_Z_L], &mut data)?;

        // Convert raw bytes to i16 (little-endian)
        let raw_z = i16::from_le_bytes(data);

        // Convert to gravity (g)
        let g_force = raw_z as f32 / ACCEL_LSB_PER_G_8G;

        Ok(g_force)
    }

    // Returns (X, Y, Z) in Gs
    pub fn read_accel_all(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let mut data = [0u8; 6];
        // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
        self.i2c.write_read(self.addr, &[REG_ACC_X_L], &mut data)?;

        let raw_x = i16::from_le_bytes([data[0], data[1]]);
        let raw_y = i16::from_le_bytes([data[2], data[3]]);
        let raw_z = i16::from_le_bytes([data[4], data[5]]);

        let scale = self.accel_lsb_per_g;
        Ok((
            raw_x as f32 / scale,
            raw_y as f32 / scale,
            raw_z as f32 / scale,
        ))
    }

    /// Disable all sensors (accel + gyro).
    pub fn disable_sensors(&mut self) -> Result<(), Error<E>> {
        self.i2c.write(self.addr, &[REG_CTRL7, 0x00])?;
        Ok(())
    }

    /// Enable accelerometer only.
    pub fn enable_accel(&mut self) -> Result<(), Error<E>> {
        self.i2c.write(self.addr, &[REG_CTRL7, 0x01])?;
        Ok(())
    }

    /// Set accelerometer range and ODR for Wake-on-Motion mode.
    /// Uses 2G range for better sensitivity and low-power 21Hz ODR.
    fn configure_accel_for_wom(&mut self) -> Result<(), Error<E>> {
        // Range 2G (upper nibble) + ODR low-power 21Hz (lower nibble)
        let ctrl2 = (ACCEL_RANGE_2G << 4) | ACCEL_ODR_LOWPOWER_21HZ;
        self.i2c.write(self.addr, &[REG_CTRL2, ctrl2])?;
        self.accel_lsb_per_g = ACCEL_LSB_PER_G_2G;
        Ok(())
    }

    /// Restore accelerometer to normal operation (8G range, 11Hz low-power).
    pub fn configure_accel_normal(&mut self) -> Result<(), Error<E>> {
        let ctrl2 = (ACCEL_RANGE_8G << 4) | ACCEL_ODR_LOWPOWER_11HZ;
        self.i2c.write(self.addr, &[REG_CTRL2, ctrl2])?;
        self.accel_lsb_per_g = ACCEL_LSB_PER_G_8G;
        Ok(())
    }

    /// Execute a CTRL9 command and wait for completion.
    /// The CTRL9 protocol requires writing the command, then polling STATUS1
    /// until bit 0 (CmdDone) is set, then reading STATUS1 to acknowledge.
    fn execute_ctrl9_cmd(&mut self, cmd: u8) -> Result<(), Error<E>> {
        // Write command to CTRL9
        self.i2c.write(self.addr, &[REG_CTRL9, cmd])?;

        // Poll STATUS1 for CmdDone (bit 0)
        // In practice this is very fast, but we add a simple spin loop
        let mut status = [0u8; 1];
        for _ in 0..100 {
            self.i2c
                .write_read(self.addr, &[REG_STATUS1], &mut status)?;
            if status[0] & 0x01 != 0 {
                // CmdDone is set, read clears it and INT1
                return Ok(());
            }
        }
        // Timeout - command didn't complete, but we continue anyway
        Ok(())
    }

    /// Enable Wake-on-Motion mode.
    ///
    /// # Arguments
    /// * `threshold_mg` - Motion threshold in mg (1-255). Higher = less sensitive.
    ///   Typical values: 50-200mg for reasonable motion detection.
    /// * `interrupt` - Which interrupt pin to use and its initial state.
    /// * `blanking_samples` - Number of accelerometer samples to ignore after enabling
    ///   WoM to avoid spurious triggers (0-63).
    ///
    /// After calling this, the selected interrupt pin will toggle on each motion event.
    /// Read STATUS1 to clear the WoM bit and reset the interrupt.
    pub fn enable_wake_on_motion(
        &mut self,
        threshold_mg: u8,
        interrupt: WomInterrupt,
        blanking_samples: u8,
    ) -> Result<(), Error<E>> {
        // Step 1: Disable all sensors
        self.disable_sensors()?;

        // Step 2: Configure accelerometer for WoM (2G range, low-power ODR)
        self.configure_accel_for_wom()?;

        // Step 3: Set WoM threshold in CAL1_L
        self.i2c.write(self.addr, &[REG_CAL1_L, threshold_mg])?;

        // Step 4: Set interrupt select (bits 7:6) and blanking time (bits 5:0) in CAL1_H
        let cal1_h = (interrupt.to_bits() << 6) | (blanking_samples & 0x3F);
        self.i2c.write(self.addr, &[REG_CAL1_H, cal1_h])?;

        // Step 5: Execute CTRL9 command to configure WoM
        self.execute_ctrl9_cmd(CTRL9_CMD_WOM_SETTING)?;

        // Step 6: Enable accelerometer
        self.enable_accel()?;

        Ok(())
    }

    /// Disable Wake-on-Motion mode and return to normal operation.
    pub fn disable_wake_on_motion(&mut self) -> Result<(), Error<E>> {
        // Step 1: Disable all sensors
        self.disable_sensors()?;

        // Step 2: Set WoM threshold to 0 (disables WoM)
        self.i2c.write(self.addr, &[REG_CAL1_L, 0x00])?;
        self.i2c.write(self.addr, &[REG_CAL1_H, 0x00])?;

        // Step 3: Execute CTRL9 command to apply WoM disable
        self.execute_ctrl9_cmd(CTRL9_CMD_WOM_SETTING)?;

        // Step 4: Restore normal accelerometer config
        self.configure_accel_normal()?;

        // Step 5: Re-enable accelerometer
        self.enable_accel()?;

        Ok(())
    }

    /// Read STATUS1 register. This clears the WoM bit and resets the interrupt.
    /// Returns the status byte where bit 2 indicates a WoM event occurred.
    pub fn read_status1(&mut self) -> Result<u8, Error<E>> {
        let mut data = [0u8; 1];
        self.i2c.write_read(self.addr, &[REG_STATUS1], &mut data)?;
        Ok(data[0])
    }

    /// Check if a Wake-on-Motion event occurred.
    /// This reads and clears STATUS1, returning true if bit 2 (WoM) was set.
    pub fn check_wom_event(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_status1()?;
        Ok(status & 0x04 != 0)
    }
}
