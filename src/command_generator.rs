use crate::command_generator::Command::{Feather, Throttle};
use crate::constants::{CONTROLLER_TRIGGER_CONVERSION_RATIO, MOTOR_MIN};
use controller_radio_interface::*;

#[derive(Copy, Clone, defmt::Format)]
pub enum Command {
    Throttle(u16),
    Start,
    Stop,
    Feather,
}

pub struct CommandGenerator {
    old_controller_state: ControllerState,
    previous_button: Option<Button>,
}

impl CommandGenerator {
    pub fn new() -> Self {
        Self {
            old_controller_state: ControllerState::new(),
            previous_button: None,
        }
    }

    pub fn generate(&mut self, controller_state: ControllerState) -> [Option<Command>; 10] {
        let mut command_list: [Option<Command>; 10] = [None; 10];
        if self.old_controller_state == controller_state {
            return command_list;
        }
        let mut index = 0;
        let mut old_trigger_value: i16 = 0;
        let mut old_start_value = false;
        let mut old_north_value = false;
        let mut old_east_value = false;
        let mut old_south_value = false;
        let mut old_west_value = false;
        for field in self.old_controller_state.fields() {
            match field {
                ControllerField::RightTrigger(value) => old_trigger_value = value,
                ControllerField::Button(Button::Start, value) => old_start_value = value,
                ControllerField::Button(Button::North, value) => old_north_value = value,
                ControllerField::Button(Button::East, value) => old_east_value = value,
                ControllerField::Button(Button::South, value) => old_south_value = value,
                ControllerField::Button(Button::West, value) => old_west_value = value,
                _ => {}
            }
        }

        for field in controller_state.fields() {
            match field {
                ControllerField::Button(Button::Start, new_value) => {
                    if old_start_value == false && new_value == true {
                        command_list[index] = Some(Feather);
                        index += 1;
                        if index == command_list.len() {
                            break;
                        }
                    }
                }
                ControllerField::Button(Button::North, new_value) => {
                    if old_north_value == false && new_value == true {
                        self.previous_button = Some(Button::North);
                    }
                }
                ControllerField::Button(Button::East, new_value) => {
                    if old_east_value == false && new_value == true {
                        if self.previous_button.is_some() {
                            if self.previous_button.unwrap() == Button::North {
                                self.previous_button = Some(Button::East);
                            } else {
                                self.previous_button = None;
                            }
                        }
                    }
                }
                ControllerField::Button(Button::West, new_value) => {
                    if old_west_value == false && new_value == true {
                        if self.previous_button.is_some() {
                            if self.previous_button.unwrap() == Button::North {
                                self.previous_button = Some(Button::West);
                            } else {
                                self.previous_button = None;
                            }
                        }
                    }
                }
                ControllerField::Button(Button::South, new_value) => {
                    if old_south_value == false && new_value == true {
                        if self.previous_button.is_some() {
                            if self.previous_button.unwrap() == Button::East {
                                self.previous_button = None;
                                command_list[index] = Some(Command::Stop);
                                index += 1;
                                if index == command_list.len() {
                                    break;
                                }
                            } else if self.previous_button.unwrap() == Button::West {
                                self.previous_button = None;
                                command_list[index] = Some(Command::Start);
                                index += 1;
                                if index == command_list.len() {
                                    break;
                                }
                            } else {
                                self.previous_button = None;
                            }
                        }
                    }
                }
                ControllerField::RightTrigger(new_value) => {
                    if new_value != old_trigger_value {
                        let throttle = MOTOR_MIN
                            + (CONTROLLER_TRIGGER_CONVERSION_RATIO * new_value as f32) as u16;
                        command_list[index] = Some(Throttle(throttle));
                        index += 1;
                        if index == command_list.len() {
                            break;
                        }
                    }
                }
                _ => {}
            }
        }
        self.old_controller_state = controller_state;
        command_list
    }
}
