use embedded_hal::i2c::I2c;

pub const QMI8658_ADDRESS_LOW: u8 = 0x6A;
pub const QMI8658_ADDRESS_HIGH: u8 = 0x6B;

const REG_WHO_AM_I: u8 = 0x00;
const REG_CTRL1: u8 = 0x02;
const REG_CTRL2: u8 = 0x03;
const REG_CTRL7: u8 = 0x08;

const REG_ACC_X_L: u8 = 0x35;
const REG_ACC_Z_L: u8 = 0x39;

const WHO_AM_I_EXPECTED: u8 = 0x05;

const ACCEL_RANGE_8G: u8 = 0x02;
// Low-power ODRs live in the low nibble as 0x0C..0x0F in the reference driver.
// Pick 11Hz as a close low-power rate
const ACCEL_ODR_LOWPOWER_11HZ: u8 = 0x0E;

// Sensitivity for 8g range is 4096 LSB/g.
const ACCEL_LSB_PER_G_8G: f32 = 4096.0;

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
        }
    }

    // Just in case if needed for another board.
    pub fn new_with_addr(i2c: I, addr: u8) -> Self {
        Self { i2c, addr }
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

        let scale = ACCEL_LSB_PER_G_8G;
        Ok((
            raw_x as f32 / scale,
            raw_y as f32 / scale,
            raw_z as f32 / scale,
        ))
    }
}
