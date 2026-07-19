use crate::command_generator::Command;
use crate::constants::{MOTOR_FEATHER, MOTOR_MAX, MOTOR_MIN};
use crate::motor::mix_and_clamp;
use crate::pid::Pid;
use crate::type_defs::{PwmBL, PwmBR, PwmFL, PwmFR, UartTx};
use core::cmp::PartialEq;
use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_PwmPin;
use heapless::String;

#[derive(PartialEq)]
pub enum DroneState {
    Standby,
    On,
    Feather,
}

pub struct FlightController {
    pwm_front_right: PwmFR,
    pwm_bottom_right: PwmBR,
    pwm_bottom_left: PwmBL,
    pwm_front_left: PwmFL,
    rate_pid_pitch: Pid,
    rate_pid_roll: Pid,
    rate_pid_yaw: Pid,
    uart_tx: UartTx,
    string: String<256>,
    throttle: u16,
    state: DroneState,
}

impl FlightController {
    pub fn new(
        pwm_front_right: PwmFR,
        pwm_bottom_right: PwmBR,
        pwm_bottom_left: PwmBL,
        pwm_front_left: PwmFL,
        rate_pid_pitch: Pid,
        rate_pid_roll: Pid,
        rate_pid_yaw: Pid,
        uart_tx: UartTx,
    ) -> Self {
        Self {
            pwm_front_right,
            pwm_bottom_right,
            pwm_bottom_left,
            pwm_front_left,
            rate_pid_pitch,
            rate_pid_roll,
            rate_pid_yaw,
            uart_tx,
            string: String::new(),
            throttle: MOTOR_MIN,
            state: DroneState::Standby,
        }
    }

    pub fn update(
        &mut self,
        gx: f32,
        gy: f32,
        gz: f32,
        desired_roll_rate: f32,
        desired_pitch_rate: f32,
    ) {
        if self.state == DroneState::Standby {
            self.pwm_front_right.set_duty(MOTOR_MIN);
            self.pwm_bottom_right.set_duty(MOTOR_MIN);
            self.pwm_bottom_left.set_duty(MOTOR_MIN);
            self.pwm_front_left.set_duty(MOTOR_MIN);
            return;
        }

        self.rate_pid_roll.set_setpoint(desired_roll_rate);
        self.rate_pid_pitch.set_setpoint(desired_pitch_rate);
        self.rate_pid_yaw.set_setpoint(0.0);

        let roll_out = self.rate_pid_roll.update(gx);
        let pitch_out = self.rate_pid_pitch.update(gy);
        let yaw_out = self.rate_pid_yaw.update(gz);

        let motors = mix_and_clamp(
            self.throttle as f32,
            roll_out,
            pitch_out,
            yaw_out,
            MOTOR_MIN as f32,
            MOTOR_MAX as f32,
        );

        self.pwm_front_right.set_duty(motors.get_fr() as u16);
        self.pwm_bottom_right.set_duty(motors.get_br() as u16);
        self.pwm_bottom_left.set_duty(motors.get_bl() as u16);
        self.pwm_front_left.set_duty(motors.get_fl() as u16);

        write!(
            self.string,
            "roll_out:{:.3}\tpitch_out:{:.3}\t\
                    yaw_out:{:.3}\tfr:{:.3}\tbr:{:.3}\tbl:{:.3}\tfl:{:.3}\r\n",
            roll_out,
            pitch_out,
            yaw_out,
            motors.get_fr(),
            motors.get_br(),
            motors.get_bl(),
            motors.get_fl()
        )
        .unwrap();
        self.uart_tx.write_full_blocking(self.string.as_bytes());
        self.string.clear();
    }

    pub fn iterate_state(&mut self, command: Command) {
        match command {
            Command::Throttle(throttle) => {
                if self.state == DroneState::On {
                    self.state = DroneState::On;
                    self.throttle = throttle;
                    defmt::info!("Throttle {}", throttle);
                }
            }
            Command::Start => {
                if self.state == DroneState::Standby || self.state == DroneState::Feather {
                    self.state = DroneState::On;
                    defmt::info!("DroneState::On");
                }
            }
            Command::Stop => {
                self.state = DroneState::Standby;
                self.throttle = MOTOR_MIN;
                defmt::info!("DroneState::Standby");
            }
            Command::Feather => {
                if self.state == DroneState::On || self.state == DroneState::Standby {
                    defmt::info!("DroneState::Feather");
                    self.state = DroneState::Feather;
                    self.throttle = MOTOR_FEATHER;
                }
            }
        }
    }

    pub fn set_feather(&mut self) {
        if self.state != DroneState::Standby && self.state != DroneState::Feather {
            defmt::info!("DroneState::Feather TIMEOUT");
            self.state = DroneState::Feather;
            self.throttle = MOTOR_FEATHER;
        }
    }

    pub fn set_standby(&mut self) {
        if self.state != DroneState::Standby {
            defmt::info!("DroneState::Standby TIMEOUT");
            self.state = DroneState::Standby;
            self.throttle = MOTOR_MIN;
        }
    }
}
