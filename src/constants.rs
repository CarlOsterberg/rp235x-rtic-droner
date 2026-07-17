
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

// Average values for when stationary, in degrees (converted to radians where used,
// since ComplementaryFilter::get_roll()/get_pitch() return radians).
pub const ROLL_SETPOINT: f32 = 1.0;
pub const PITCH_SETPOINT: f32 = 4.5;

pub const MOTOR_MIN: u16 = 2000;
pub const MOTOR_MAX: u16 = 4000;

// Clamp for the angle PIDs' output (commanded rate, rad/s) to prevent integral windup.
pub const ANGLE_PID_RATE_LIMIT: f32 = 3.0;
// Clamp for the rate PIDs' output (PWM duty contribution per axis) to prevent integral windup.
pub const RATE_PID_OUTPUT_LIMIT: f32 = 500.0;

pub const CONTROLLER_TRIGGER_CONVERSION_RATIO: f32 = 2000.0 / 255.0;

pub const PWM_TOP: u16 = 20000;
// pub const UART_READER_CAPACITY: usize = 32; // Not used
// pub const MSG_Q_CAPACITY: usize = 32;

// nRF24L01 max SPI clock is 10MHz, use 8MHz to be safe
pub const SPI_BAUDRATE: u32 = 8_000_000;
pub const SPI_PERIPHERAL_CLK: u32 = 125_000_000;

// nRF24L01 register addresses
pub const REG_CONFIG: u8       = 0x00;
pub const REG_EN_AA: u8        = 0x01;
pub const REG_EN_RXADDR: u8    = 0x02;
pub const REG_SETUP_AW: u8     = 0x03;
pub const REG_SETUP_RETR: u8   = 0x04;
pub const REG_RF_CH: u8        = 0x05;
pub const REG_RF_SETUP: u8     = 0x06;
pub const REG_STATUS: u8       = 0x07;
pub const REG_RX_ADDR_P0: u8   = 0x0A;
pub const REG_TX_ADDR: u8      = 0x10;
pub const REG_RX_PW_P0: u8     = 0x11;

// nRF24L01 SPI commands
pub const CMD_W_REGISTER: u8   = 0x20; // OR with register address
pub const CMD_W_TX_PAYLOAD: u8 = 0xA0;
pub const CMD_R_RX_PAYLOAD: u8 = 0x61;
pub const CMD_FLUSH_TX: u8     = 0xE1;
pub const CMD_FLUSH_RX: u8     = 0xE2;
pub const CMD_NOP: u8          = 0xFF; // No-op, returns STATUS

// nRF24L01 CONFIG register values
pub const CONFIG_PWR_DOWN: u8  = 0x08; // EN_CRC only (PWR_UP=0, PRIM_RX=0)
pub const CONFIG_RX_MODE: u8   = 0x0F; // PWR_UP | PRIM_RX | EN_CRC | CRCO
pub const CONFIG_TX_MODE: u8   = 0x0E; // PWR_UP | EN_CRC | CRCO (PRIM_RX=0)

// nRF24L01 STATUS register: clear RX_DR, TX_DS, MAX_RT interrupt flags
pub const STATUS_CLEAR_IRQ: u8 = 0x70;

// nRF24L01 configuration values
pub const EN_AA_PIPE0: u8      = 0x01; // Auto-ack on pipe 0
pub const EN_RXADDR_PIPE0: u8  = 0x01; // Enable RX address on pipe 0
pub const SETUP_AW_5BYTES: u8  = 0x03; // 5-byte address width
pub const SETUP_RETR_500US_15: u8 = 0x1F; // ARD=500µs, ARC=15
pub const RF_CH_2402MHZ: u8    = 0x02; // 2.402 GHz channel
pub const RF_SETUP_1MBPS_0DBM: u8 = 0x07; // 1Mbps, 0dBm
pub const RX_PW_P0_32BYTES: u8 = 0x20; // 16-byte payload on pipe 0
pub const DEFAULT_ADDRESS: [u8; 5] = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7];
