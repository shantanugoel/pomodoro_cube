use embedded_hal::i2c::I2c;

pub struct Qmi8658<I> {
    i2c: I,
}

impl<I, E> Qmi8658<I>
where
    I: I2c<Error = E>,
{
    pub fn new(i2c: I) -> Self {
        Qmi8658 { i2c }
    }

    // Add methods for interacting with the QMI8658 sensor here
}
