pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    integral: f32,
    prev_error: f32,
    output_limits: (f32, f32),
    dt: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, setpoint: f32, dt: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            setpoint,
            integral: 0.0,
            prev_error: 0.0,
            output_limits: (f32::MIN, f32::MAX),
            dt,
        }
    }

    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.output_limits = (min, max);
    }

    pub fn set_setpoint(&mut self, sp: f32) {
        self.setpoint = sp;
    }
    pub fn update(&mut self, measurement: f32) -> f32 {
        let error = self.setpoint - measurement;

        self.integral += error * self.dt;

        // Clamp the integral *term's contribution*, not the raw integral.
        let (min, max) = self.output_limits;
        let i_term = (self.ki * self.integral).clamp(min, max);
        // Back-solve the raw integral so it stays consistent (prevents
        // silent windup while output_limits or Ki stay fixed).
        if self.ki != 0.0 {
            self.integral = i_term / self.ki;
        } else {
            self.integral = 0.0;
        }

        let derivative = if self.dt > 0.0 {
            (error - self.prev_error) / self.dt
        } else {
            0.0
        };
        self.prev_error = error;

        let output = self.kp * error + i_term + self.kd * derivative;
        output.clamp(min, max)
    }
}