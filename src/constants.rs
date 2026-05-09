
// External high-speed crystal on the pico board is 12Mhz
pub const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000;
pub const BAUD_RATE: u32 = 460_800;
pub const GRAVITY: f32 = 9.818f32; // m/s^2
pub const ACCEL_LSB: f32 = 16_384.0; // 16384/g
pub const GYRO_LSB: f32 = 131.0;

// I2C address of the MPU6050 sensor
pub const SENSOR_I2C_ADDR: u8 = 0x68;

pub const SENSOR_DATA_REG: u8 = 0x3B;
pub const SENSOR_DATA_NUM_BYTES: usize = 14;

// Thresholds for when IMU is stationary
pub const GYRO_THRESHOLD: f32 = 0.3; // rad/s
pub const ACCEL_THRESHOLD: f32 = 1.0; // m/s²

pub const PWM_TOP: u16 = 20000;
pub const UART_READER_CAPACITY: usize = 32;
pub const MSG_Q_CAPACITY: usize = 32;