use libm::sqrtf;
use crate::constants::{ACCEL_LSB, GRAVITY, GYRO_LSB};

pub struct SensorValues {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
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
            // az = x
            // ay = y
            // -ax = z
            ax: ax_m_ps,
            ay: ay_m_ps,
            az: az_m_ps,
            // Due to sensor orientation on breadboard,
            // gz = x
            // gy = y
            // -gx = z
            gx: gx_rad_ps,
            gy: gy_rad_ps,
            gz: gz_rad_ps,
        }
    }

    pub fn is_stationary(&self, accel_threshold: f32, gyro_threshold: f32) -> bool {
        // Check if gyroscope readings are near zero
        let gyro_mag = sqrtf(self.gx * self.gx + self.gy * self.gy + self.gz * self.gz);

        // Check if accelerometer is close to 1g (gravity only)
        let accel_mag = sqrtf(self.ax * self.ax + self.ay * self.ay + self.az * self.az);
        let accel_deviation = (accel_mag - GRAVITY).abs();

        gyro_mag < gyro_threshold && accel_deviation < accel_threshold
    }
}