
use crate::constants::{ACCEL_LSB, GRAVITY, GYRO_LSB};

pub struct GyroValues {
    gx: f32,
    gy: f32,
    gz: f32,
}

impl GyroValues {
    pub fn new(gx: f32, gy: f32, gz: f32) -> Self {
        Self { gx, gy, gz }
    }

    pub fn get_gx(&self) -> f32 {
        self.gx
    }

    pub fn get_gy(&self) -> f32 {
        self.gy
    }

    pub fn get_gz(&self) -> f32 {
        self.gz
    }
}

pub struct SensorValues {
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
}

impl SensorValues {
    // Accelerometer values must be in m/s^2
    // Gyroscope values must be in rad/s
    pub fn new(buffer: &[u8;14]) -> Self {
        // | Sensor  | Register Address        | Bytes | Description  |
        // | ------- | ----------------------- | ----- | ------------ |
        // | Accel X | 0x3B (high), 0x3C (low) | 2     | Accel X-axis |
        // | Accel Y | 0x3D, 0x3E              | 2     | Accel Y-axis |
        // | Accel Z | 0x3F, 0x40              | 2     | Accel Z-axis |
        // | Temp    | 0x41, 0x42              | 2     | Temperature  |
        // | Gyro X  | 0x43, 0x44              | 2     | Gyro X-axis  |
        // | Gyro Y  | 0x45, 0x46              | 2     | Gyro Y-axis  |
        // | Gyro Z  | 0x47, 0x48              | 2     | Gyro Z-axis  |
        // see, https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

        let raw_accelerometer_x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let raw_accelerometer_y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let raw_accelerometer_z = i16::from_be_bytes([buffer[4], buffer[5]]);
        //  Convert accelerometer sensor values to m/s^2
        let ax_m_ps = raw_accelerometer_x as f32 / ACCEL_LSB * GRAVITY;
        let ay_m_ps = raw_accelerometer_y as f32 / ACCEL_LSB * GRAVITY;
        let az_m_ps = raw_accelerometer_z as f32 / ACCEL_LSB * GRAVITY;
        // ---------------------- ACCEL -----------------------

        // ----------------------- GYRO -----------------------
        let gx_degrees_ps = i16::from_be_bytes([buffer[8], buffer[9]]);
        let gy_degrees_ps = i16::from_be_bytes([buffer[10], buffer[11]]);
        let gz_degrees_ps = i16::from_be_bytes([buffer[12], buffer[13]]);
        // Convert from gyro sensor reading to rad/s
        let gx_rad_ps = (gx_degrees_ps as f32 / GYRO_LSB).to_radians();
        let gy_rad_ps = (gy_degrees_ps as f32 / GYRO_LSB).to_radians();
        let gz_rad_ps = (gz_degrees_ps as f32 / GYRO_LSB).to_radians();
        // ----------------------- GYRO -----------------------

        SensorValues {
            // Due to sensor orientation on breadboard,
            // ax = x
            // ay = -y
            // az = z
            ax: ax_m_ps,
            ay: -ay_m_ps,
            az: az_m_ps,
            // Due to sensor orientation on breadboard,
            // gx = -x
            // gy = y
            // gz = -z
            gx: -gx_rad_ps,
            gy: gy_rad_ps,
            gz: -gz_rad_ps,
        }
    }

    pub fn get_gx(&self) -> f32 {
        self.gx
    }

    pub fn get_gy(&self) -> f32 {
        self.gy
    }

    pub fn get_gz(&self) -> f32 {
        self.gz
    }

    pub fn get_ax(&self) -> f32 {
        self.ax
    }

    pub fn get_ay(&self) -> f32 {
        self.ay
    }

    pub fn get_az(&self) -> f32 {
        self.az
    }

    pub fn get_gyro_values(&self) -> GyroValues {
        GyroValues::new(self.gx, self.gy, self.gz)
    }
}