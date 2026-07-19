#![no_std]
#![no_main]

mod command_generator;
mod complementary_filter;
mod constants;
mod flight_controller;
mod motor;
mod nrf24l01;
mod pid;
mod sensor_values;
mod type_defs;

use cortex_m::prelude::_embedded_hal_PwmPin;
use defmt_rtt as _;
use embedded_hal::{i2c::I2c, spi::MODE_0};
use fugit::RateExtU32;
use rp235x_hal::{
    self as hal, Clock,
    clocks::init_clocks_and_plls,
    gpio::FunctionSpi,
    pwm::Slices,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
};
use rtic_monotonics::rp235x::prelude::*;
use rtic_sync::channel::{Receiver, Sender};

use crate::command_generator::{Command, CommandGenerator};
use crate::complementary_filter::*;
use crate::constants::*;
use crate::flight_controller::*;
use crate::nrf24l01::*;
use crate::pid::*;
use crate::sensor_values::*;
use crate::type_defs::*;
use controller_radio_interface::*;

rp235x_timer_monotonic!(Mono);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic {}", defmt::Display2Format(info));

    // Halt or reset the device
    cortex_m::peripheral::SCB::sys_reset();
}

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 RTIC droner"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

#[rtic::app(
    device = rp235x_hal::pac,
    peripherals = true,
    dispatchers = [
        TIMER0_IRQ_1,
        TIMER0_IRQ_2,
        TIMER0_IRQ_3,
        TIMER1_IRQ_1,
    ]
)]
mod app {
    use super::*;
    use rtic_sync::make_channel;

    #[shared]
    struct Shared {
        gyro_values: GyroValues,
        desired_roll_rate: f32,
        desired_pitch_rate: f32,
    }

    #[local]
    struct Local {
        i2c: I2c1,
        interrupt_imu: InterruptPinIMU,
        buffer: [u8; 14],
        nrf24l01: NRF24L01,
        interrupt_nrf: InterruptPinNRF,
        sleep_duration_ms: u64,
        flight_controller: FlightController,
        angle_pid_pitch: Pid,
        angle_pid_roll: Pid,
        complementary_filter: ComplementaryFilter,
        command_generator: CommandGenerator,
        command_sender: Sender<'static, Command, MSG_Q_CAPACITY>,
        command_receiver: Receiver<'static, Command, MSG_Q_CAPACITY>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        let clocks = init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(ctx.device.SIO);

        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        let uart_pins = (
            pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
            pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
        );

        let uart =
            hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(BAUD_RATE.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        let (_, uart_tx) = uart.split();

        // -------------------------- TR PWM4 chA gpio8/pin11 --------------------------
        let pin11 = pins.gpio8.into_function::<hal::gpio::FunctionPwm>();
        let mut pwm4 = pwm_slices.pwm4;
        pwm4.set_ph_correct();
        pwm4.set_div_int(35);
        pwm4.set_top(PWM_TOP);
        pwm4.enable();
        let mut pwm_pin_11 = pwm4.channel_a;
        pwm_pin_11.output_to(pin11);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_11.set_duty(MOTOR_MIN);

        // -------------------------- BR PWM5 chA gpio10/pin14 --------------------------
        let pin14 = pins.gpio10.into_function::<hal::gpio::FunctionPwm>();
        let mut pwm5 = pwm_slices.pwm5;
        pwm5.set_ph_correct();
        pwm5.set_div_int(35);
        pwm5.set_top(PWM_TOP);
        pwm5.enable();
        let mut pwm_pin_14 = pwm5.channel_a;
        pwm_pin_14.output_to(pin14);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_14.set_duty(MOTOR_MIN);

        // -------------------------- BL PWM0 chA gpio16/pin21 --------------------------
        let pin21 = pins.gpio16.into_function::<hal::gpio::FunctionPwm>();
        let mut pwm0 = pwm_slices.pwm0;
        pwm0.set_ph_correct();
        pwm0.set_div_int(35);
        pwm0.set_top(PWM_TOP);
        pwm0.enable();
        let mut pwm_pin_21 = pwm0.channel_a;
        pwm_pin_21.output_to(pin21);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_21.set_duty(MOTOR_MIN);

        // -------------------------- TL PWM2 chA gpio20/pin27 --------------------------
        let pin27 = pins.gpio20.into_function::<hal::gpio::FunctionPwm>();
        let mut pwm2 = pwm_slices.pwm2;
        pwm2.set_ph_correct();
        pwm2.set_div_int(35);
        pwm2.set_top(PWM_TOP);
        pwm2.enable();
        let mut pwm_pin_27 = pwm2.channel_a;
        pwm_pin_27.output_to(pin27);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_27.set_duty(MOTOR_MIN);

        // -------------------------- MPU6050 --------------------------
        let sda_pin = pins
            .gpio14
            .into_function::<hal::gpio::FunctionI2C>()
            .into_pull_type::<hal::gpio::PullUp>();
        let scl_pin = pins
            .gpio15
            .into_function::<hal::gpio::FunctionI2C>()
            .into_pull_type::<hal::gpio::PullUp>();

        let mut i2c = hal::i2c::I2C::i2c1(
            ctx.device.I2C1,
            sda_pin,
            scl_pin,
            100.kHz(),
            &mut ctx.device.RESETS,
            clocks.system_clock.freq(),
        );

        // Enable the interrupts on this specific pin,
        // the sensor will pull this high when data is available
        let interrupt_imu = pins.gpio13.into_pull_up_input();
        interrupt_imu.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        // Wake up the MPU6050 sensor, it starts in sleep mode
        match i2c.write(SENSOR_I2C_ADDR, &[0x6B, 0x00]) {
            Ok(_) => defmt::info!("Sensor wakeup OK"),
            Err(_) => defmt::info!("Sensor wakeup failed"),
        }
        // Enable Digital Low Pass Filter (DLPF), this also sets sample rate to 1 kHz,
        // without DLPF the sample rate is 8 kHz.
        match i2c.write(SENSOR_I2C_ADDR, &[0x1A, 0x06]) {
            Ok(_) => defmt::info!("DLPF configuration OK"),
            Err(_) => defmt::info!("DLPF configuration failed"),
        }
        // Set sample rate divider - slows output to 100 Hz ( 1000Hz / (1 + 9) )
        let sample_rate_divider = 0x9;
        match i2c.write(SENSOR_I2C_ADDR, &[0x19, sample_rate_divider]) {
            Ok(_) => defmt::info!("Sample rate set OK"),
            Err(_) => defmt::info!("Sample rate set failed"),
        }
        let sample_rate: f32 = 1000.0 / (1 + sample_rate_divider) as f32;
        // Enable sensor to generate an interrupt when new data is available
        match i2c.write(SENSOR_I2C_ADDR, &[0x38, 0x01]) {
            Ok(_) => defmt::info!("Sensor interrupt enable OK"),
            Err(_) => defmt::info!("Sensor interrupt enable failed"),
        }

        let alpha: f32 = 0.05;
        let complementary_filter = ComplementaryFilter::new(sample_rate, alpha);

        let sleep_duration_ms = 10;
        let dt = 1.0 / (sleep_duration_ms as f32 / 1000.0);
        let mut angle_pid_pitch = Pid::new(100.0, 0.0, 0.0, PITCH_SETPOINT.to_radians(), dt);
        let mut angle_pid_roll = Pid::new(100.0, 0.0, 0.0, ROLL_SETPOINT.to_radians(), dt);
        angle_pid_pitch.set_output_limits(-ANGLE_PID_RATE_LIMIT, ANGLE_PID_RATE_LIMIT);
        angle_pid_roll.set_output_limits(-ANGLE_PID_RATE_LIMIT, ANGLE_PID_RATE_LIMIT);

        let mut rate_pid_pitch = Pid::new(100.0, 0.0, 0.0, 0.0, dt);
        let mut rate_pid_roll = Pid::new(100.0, 0.0, 0.0, 0.0, dt);
        let mut rate_pid_yaw = Pid::new(100.0, 0.0, 0.0, 0.0, dt);
        rate_pid_pitch.set_output_limits(-RATE_PID_OUTPUT_LIMIT, RATE_PID_OUTPUT_LIMIT);
        rate_pid_roll.set_output_limits(-RATE_PID_OUTPUT_LIMIT, RATE_PID_OUTPUT_LIMIT);
        rate_pid_yaw.set_output_limits(-RATE_PID_OUTPUT_LIMIT, RATE_PID_OUTPUT_LIMIT);

        let flight_controller = FlightController::new(
            pwm_pin_27,
            pwm_pin_21,
            pwm_pin_11,
            pwm_pin_14,
            rate_pid_pitch,
            rate_pid_roll,
            rate_pid_yaw,
            uart_tx,
        );

        let buffer: [u8; SENSOR_DATA_NUM_BYTES] = [0u8; SENSOR_DATA_NUM_BYTES];

        // -------------------------- NRF24L01 --------------------------
        let sck = pins.gpio2.into_function::<FunctionSpi>();
        let tx = pins.gpio3.into_function::<FunctionSpi>(); // MOSI
        let rx = pins.gpio4.into_function::<FunctionSpi>(); // MISO
        let spi_pin_layout = (tx, rx, sck);
        let spi: Spi = hal::Spi::new(ctx.device.SPI0, spi_pin_layout).init(
            &mut ctx.device.RESETS,
            SPI_PERIPHERAL_CLK.Hz(),
            SPI_BAUDRATE.Hz(),
            MODE_0,
        );

        let nrf_ce = pins.gpio7.into_push_pull_output();
        let nrf_csn = pins.gpio5.into_push_pull_output();

        let mut nrf24l01 = NRF24L01::new(spi, nrf_ce, nrf_csn);
        nrf24l01.init();
        let status = nrf24l01.read_status();
        defmt::info!("nrf24l01 initiated, STATUS after init: {:#010b}", status);

        // Enable the interrupts on this pin to allow nrf chip to signal us.
        let interrupt_nrf = pins.gpio6.into_pull_down_input();
        interrupt_nrf.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

        let gyro_values = GyroValues::new(0.0f32, 0.0f32, 0.0f32);

        let desired_roll_rate = 0.0f32;
        let desired_pitch_rate = 0.0f32;

        let command_generator = CommandGenerator::new();

        let (s, r) = make_channel!(Command, MSG_Q_CAPACITY);

        flight_controller::spawn().ok();

        (
            Shared {
                gyro_values,
                desired_roll_rate,
                desired_pitch_rate,
            },
            Local {
                i2c,
                interrupt_imu,
                buffer,
                nrf24l01,
                interrupt_nrf,
                sleep_duration_ms,
                flight_controller,
                angle_pid_pitch,
                angle_pid_roll,
                complementary_filter,
                command_generator,
                command_sender: s,
                command_receiver: r,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [interrupt_imu, interrupt_nrf], priority = 1)]
    fn gpio_irq(ctx: gpio_irq::Context) {
        if ctx
            .local
            .interrupt_imu
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            ctx.local
                .interrupt_imu
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            imu::spawn().ok();
        } else if ctx
            .local
            .interrupt_nrf
            .interrupt_status(hal::gpio::Interrupt::EdgeLow)
        {
            ctx.local
                .interrupt_nrf
                .clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            radio::spawn().ok();
        }
    }

    #[task(local =
    [
        i2c,
        buffer,
        angle_pid_pitch,
        angle_pid_roll,
        complementary_filter
    ],
    shared = [
        gyro_values,
        desired_roll_rate,
        desired_pitch_rate
    ],
    priority = 2
    )]
    async fn imu(mut ctx: imu::Context) {
        let local = ctx.local;
        match local
            .i2c
            .write_read(SENSOR_I2C_ADDR, &[SENSOR_DATA_REG], local.buffer)
        {
            Ok(_) => {
                let sensor_values = SensorValues::new(local.buffer);
                ctx.shared
                    .gyro_values
                    .lock(|gv| *gv = sensor_values.get_gyro_values());
                local.complementary_filter.timestep(sensor_values);
                let roll = local.complementary_filter.get_roll();
                let pitch = local.complementary_filter.get_pitch();
                ctx.shared
                    .desired_roll_rate
                    .lock(|drr| *drr = local.angle_pid_roll.update(roll));
                ctx.shared
                    .desired_pitch_rate
                    .lock(|dpr| *dpr = local.angle_pid_pitch.update(pitch));
                // defmt::info!("roll {}\tpitch {}", roll.to_degrees(), pitch.to_degrees());
            }
            Err(_) => {
                defmt::error!("mpu6050 i2c read error");
            }
        }
    }

    #[task(local = [
        sleep_duration_ms,
        flight_controller,
        command_receiver,
    ],
    shared = [
        gyro_values,
        desired_roll_rate,
        desired_pitch_rate,
    ],
    priority = 4)]
    async fn flight_controller(mut ctx: flight_controller::Context) {
        let local = ctx.local;

        loop {
            Mono::delay(local.sleep_duration_ms.millis()).await;

            let (gx, gy, gz) = ctx
                .shared
                .gyro_values
                .lock(|gv| (gv.get_gx(), gv.get_gy(), gv.get_gz()));

            let drr = ctx.shared.desired_roll_rate.lock(|drr| *drr);
            let dpr = ctx.shared.desired_pitch_rate.lock(|dpr| *dpr);

            match local.command_receiver.try_recv()
            {
                Ok(command) => {
                    local.flight_controller.iterate_state(command);
                }
                Err(_) => {
                }
            }

            local
                .flight_controller
                .update(gx, gy, gz, drr, dpr);
        }
    }

    #[task(local = [nrf24l01, command_generator, command_sender], priority = 3)]
    async fn radio(ctx: radio::Context) {
        let nrf24l01 = ctx.local.nrf24l01;
        let status = nrf24l01.read_status();
        nrf24l01.clear_interrupts();
        match nrf24l01.get_state() {
            State::Receiver => {
                let payload = nrf24l01.read_payload();
                let result = ControllerState::deserialize(&payload);
                match result {
                    Ok(input) => {
                        for command in ctx.local.command_generator.generate(input) {
                            if command.is_none() {
                                break;
                            }
                            defmt::info!("{}", command.unwrap());
                            if ctx.local.command_sender.try_send(command.unwrap()).is_err()
                            {
                                defmt::error!("Command Q full, dropping data");
                                break;
                            }
                        }
                    }
                    Err(error) => {
                        defmt::error!("{:?}", error);
                    }
                }
            }
            State::Transmitter => {
                if status & 0x20 != 0 {
                    // TX_DS - packet sent successfully
                    nrf24l01.set_receiver_mode();
                    defmt::info!("radio::Transmit successful");
                } else if status & 0x10 != 0 {
                    // MAX_RT - transmission failed, must flush TX FIFO
                    nrf24l01.send_command(CMD_FLUSH_TX);
                    nrf24l01.set_receiver_mode();
                    defmt::error!("radio::Transmit failed (MAX_RT)");
                }
            }
            State::Standby => {}
        }
    }
}

// End of file
