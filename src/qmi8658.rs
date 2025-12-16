use embedded_hal::i2c::I2c;

const ADDR: u8 = 0x6B;
const REG_CTRL1: u8 = 0x03;
const REG_CTRL7: u8 = 0x09;
const REG_ACC_Z_L: u8 = 0x39;
const REG_ACC_X_L: u8 = 0x35;

pub struct Qmi8658<I> {
    i2c: I,
}

impl<I, E> Qmi8658<I>
where
    I: I2c<Error = E>,
{
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }

    pub fn init(&mut self) -> Result<(), E> {
        // 1. Enable Sensors (CTRL7)
        // Bit 0: Accel Enable, Bit 1: Gyro Enable
        // We only enable Accel (0x01) to save power
        self.i2c.write(ADDR, &[REG_CTRL7, 0x01])?;

        // 2. Configure Accel (CTRL1)
        // 0x60 = 12.5Hz Output Data Rate (Low Power)
        // 0x02 = 8g Full Scale
        // Combined: 0x62
        self.i2c.write(ADDR, &[REG_CTRL1, 0x62])?;

        Ok(())
    }

    pub fn read_z_gravity(&mut self) -> Result<f32, E> {
        let mut data = [0u8; 2];
        // Read 2 bytes starting from REG_ACC_Z_L (0x39, 0x3A)
        self.i2c.write_read(ADDR, &[REG_ACC_Z_L], &mut data)?;

        // Convert raw bytes to i16
        let raw_z = (data[1] as i16) << 8 | (data[0] as i16);

        // Convert to gravity (g)
        // Sensitivity for 8g range is approx 4096 LSB/g
        let g_force = raw_z as f32 / 4096.0;

        Ok(g_force)
    }

    // Returns (X, Y, Z) in Gs
    pub fn read_accel_all(&mut self) -> Result<(f32, f32, f32), E> {
        let mut data = [0u8; 6];
        // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
        self.i2c.write_read(ADDR, &[REG_ACC_X_L], &mut data)?;

        let raw_x = (data[1] as i16) << 8 | (data[0] as i16);
        let raw_y = (data[3] as i16) << 8 | (data[2] as i16);
        let raw_z = (data[5] as i16) << 8 | (data[4] as i16);

        let scale = 4096.0; // LSB/g for 8g range
        Ok((
            raw_x as f32 / scale,
            raw_y as f32 / scale,
            raw_z as f32 / scale,
        ))
    }
}
