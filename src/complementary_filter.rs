use crate::constants;
use crate::sensor_values::SensorValues;
use libm::{asinf, atanf, cosf, sinf, tanf};
use rust_matrices::Matrix;

// Simple complementary filter using Euler angles
pub struct ComplementaryFilter {
    phi_hat: f32,
    theta_hat: f32,
    sample_rate_hz: f32,
    alpha: f32,
    is_stationary: bool,
}

impl ComplementaryFilter {
    pub fn new(sample_rate_hz: f32, alpha: f32) -> Self {
        ComplementaryFilter {
            phi_hat: 0.0,
            theta_hat: 0.0,
            sample_rate_hz,
            alpha,
            is_stationary: false,
        }
    }
    // Iterate the filter one timestep using new data
    pub fn timestep(&mut self, values: SensorValues) {
        // ---------------------- ACCEL -----------------------
        // Calculate theta_hat and phi_hat according to,
        // theta_hat = arcsin(ax/g)
        // phi_hat = arctan(ay/az)
        let theta_n_accel = asinf(values.ax / constants::GRAVITY);
        let phi_n_accel = atanf(values.ay / values.az);
        // ---------------------- ACCEL -----------------------

        // ----------------------- GYRO -----------------------
        // Calculate:
        //  ⌈  phi_dot  ⌉   ⌈ 1 sin(phi)tan(theta) cos(phi)tan(theta) ⌉   ⌈ gx ⌉
        //  ⌊ theta_dot ⌋ = ⌊ 0     cos(phi)            -sin(phi)     ⌋ * | gy |
        //                                                                ⌊ gz ⌋
        let sin_phi_hat = sinf(self.phi_hat);
        let cos_phi_hat = cosf(self.phi_hat);
        let tan_theta_hat = tanf(self.theta_hat);
        let a = Matrix::from_array([
            [
                1.0,
                sin_phi_hat * tan_theta_hat,
                cos_phi_hat * tan_theta_hat,
            ],
            [0.0, cos_phi_hat, -sin_phi_hat],
        ]);
        let b = Matrix::from_array([[values.gx], [values.gy], [values.gz]]);
        let phi_and_theta_dot = a * b;
        let phi_dot = phi_and_theta_dot.get(0, 0);
        let theta_dot = phi_and_theta_dot.get(1, 0);
        // ----------------------- GYRO -----------------------

        // --------------- COMPLEMENTARY FILTER ---------------
        // To calculate the next time step of the complementary filter, do the following:
        // theta_hat_n+1 = theta_n_accel * alpha + (1 - alpha) * (theta_hat_n + T * theta_dot_gyro_n)
        // phi_hat_n+1 = phi_n_accel * alpha + (1 - alpha) * (phi_hat_n + T * phi_dot_gyro_n)

        // Numerically integrate theta_dot and phi_dot with respect to time, dt
        let dt = 1.0 / self.sample_rate_hz;
        let theta_gyro = self.theta_hat + theta_dot * dt;
        let phi_gyro = self.phi_hat + phi_dot * dt;

        self.theta_hat = (1.0 - self.alpha) * theta_gyro + self.alpha * theta_n_accel;
        self.phi_hat = (1.0 - self.alpha) * phi_gyro + self.alpha * phi_n_accel;
        // --------------- COMPLEMENTARY FILTER ---------------

        self.is_stationary =
            values.is_stationary(constants::ACCEL_THRESHOLD, constants::GYRO_THRESHOLD);
    }

    // Get roll in radians
    pub fn get_roll(&self) -> f32 {
        self.phi_hat
    }

    // Get pitch in radians
    pub fn get_pitch(&self) -> f32 {
        self.theta_hat
    }

    pub fn get_is_stationary(&self) -> bool {
        self.is_stationary
    }
}
