#![no_std]
#![no_main]

pub mod complementary_filter;
pub mod constants;
mod nrf24l01;
pub mod sensor_values;
pub mod type_defs;

use core::fmt::Write;
use defmt_rtt as _;
use embedded_hal::{i2c::I2c, spi::MODE_0};
use fugit::RateExtU32;
use heapless::String;
use panic_probe as _;
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

use crate::complementary_filter::*;
use crate::constants::*;
use crate::nrf24l01::*;
use crate::sensor_values::*;
use crate::type_defs::*;
use controller_radio_interface::{ControllerState};

use cortex_m::prelude::_embedded_hal_PwmPin;

rp235x_timer_monotonic!(Mono);

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

#[rtic::app(device = rp235x_hal::pac, peripherals = true, dispatchers = [TIMER0_IRQ_1, TIMER0_IRQ_2, TIMER0_IRQ_3])]
mod app {
    use super::*;
    use rtic_sync::{channel::*, make_channel};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        uart_tx: UartTx,
        uart_rx: UartRx,
        i2c: I2c1,
        interrupt_imu: InterruptPinIMU,
        complementary_filter: ComplementaryFilter,
        string: String<64>,
        buffer: [u8; 14],
        buffer_reader: [u8; UART_READER_CAPACITY],
        pwm_top_right: PwmTR,
        pwm_bottom_right: PwmBR,
        pwm_bottom_left: PwmBL,
        pwm_top_left: PwmTL,
        msg_q_sender: Sender<'static, u8, MSG_Q_CAPACITY>,
        msg_q_receiver: Receiver<'static, u8, MSG_Q_CAPACITY>,
        nrf24l01: NRF24L01,
        interrupt_nrf: InterruptPinNRF,
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

        let mut uart =
            hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(BAUD_RATE.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart.enable_rx_interrupt();
        let (uart_rx, uart_tx) = uart.split();

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
        pwm_pin_11.set_duty(PWM_TOP / 10);

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
        pwm_pin_14.set_duty(PWM_TOP / 10);

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
        pwm_pin_21.set_duty(PWM_TOP / 10);

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
        pwm_pin_27.set_duty(PWM_TOP / 10);

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
        let sample_rate_hz: u32 = 1000 / (1 + sample_rate_divider as u32);
        // Enable sensor to generate an interrupt when new data is available
        match i2c.write(SENSOR_I2C_ADDR, &[0x38, 0x01]) {
            Ok(_) => defmt::info!("Sensor interrupt enable OK"),
            Err(_) => defmt::info!("Sensor interrupt enable failed"),
        }

        let alpha: f32 = 0.05;
        let complementary_filter = ComplementaryFilter::new(sample_rate_hz as f32, alpha);

        let buffer: [u8; SENSOR_DATA_NUM_BYTES] = [0u8; SENSOR_DATA_NUM_BYTES];
        let string: String<64> = String::new();

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
        defmt::info!("STATUS after init: {:#010b}", status);

        defmt::info!("nrf24l01 initiated");
        uart_tx.write_full_blocking("nrf24l01 initiated\r\n".as_bytes());

        // Enable the interrupts on this pin to allow nrf chip to signal us.
        let interrupt_nrf = pins.gpio6.into_pull_down_input();
        interrupt_nrf.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

        // -------------------------- Message Queue --------------------------
        let (s, r) = make_channel!(u8, MSG_Q_CAPACITY);

        defmt::info!(
            "Write any number 0-5 to adjust all 4 pwms. 0 is no throttle and 5 is maximum."
        );

        let buffer_reader: [u8; UART_READER_CAPACITY] = [0; UART_READER_CAPACITY];

        esc::spawn().ok();

        (
            Shared {},
            Local {
                uart_rx,
                uart_tx,
                i2c,
                interrupt_imu,
                complementary_filter,
                buffer,
                buffer_reader,
                string,
                pwm_top_right: pwm_pin_11,
                pwm_bottom_right: pwm_pin_14,
                pwm_bottom_left: pwm_pin_21,
                pwm_top_left: pwm_pin_27,
                msg_q_sender: s,
                msg_q_receiver: r,
                nrf24l01,
                interrupt_nrf,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [interrupt_imu, interrupt_nrf], priority = 2)]
    fn gpio_irq(ctx: gpio_irq::Context) {
        if ctx
            .local
            .interrupt_imu
            .interrupt_status(hal::gpio::Interrupt::EdgeHigh)
        {
            ctx.local
                .interrupt_imu
                .clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            read_i2c::spawn().ok();
        } else if ctx
            .local
            .interrupt_nrf
            .interrupt_status(hal::gpio::Interrupt::EdgeLow)
        {
            ctx.local
                .interrupt_nrf
                .clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            nrf_comms::spawn().ok();
        }
    }

    #[task(local = [i2c, uart_tx, complementary_filter, buffer, string], priority = 3)]
    async fn read_i2c(ctx: read_i2c::Context) {
        let local = ctx.local;
        match local
            .i2c
            .write_read(SENSOR_I2C_ADDR, &[SENSOR_DATA_REG], local.buffer)
        {
            Ok(_) => {
                let sensor_values = SensorValues::new(local.buffer);
                local.complementary_filter.timestep(sensor_values);

                write!(
                    local.string,
                    "roll:{:.3}\tpitch:{:.3}\tstationary:{:?}\r\n",
                    local.complementary_filter.get_roll().to_degrees(),
                    local.complementary_filter.get_pitch().to_degrees(),
                    local.complementary_filter.get_is_stationary()
                )
                .unwrap();
                local.uart_tx.write_full_blocking(local.string.as_bytes());
                local.string.clear();
            }
            Err(_) => {
                local.uart_tx.write_full_blocking(b"Read error\r\n");
            }
        }
    }

    #[task(binds = UART0_IRQ, priority = 1, local = [uart_rx, buffer_reader, msg_q_sender])]
    fn uart_rx_int(ctx: uart_rx_int::Context) {
        let res = ctx.local.uart_rx.read_raw(ctx.local.buffer_reader);
        match res {
            Ok(chars) => {
                for i in 0..chars {
                    ctx.local
                        .msg_q_sender
                        .try_send(ctx.local.buffer_reader[i])
                        .unwrap();
                    defmt::info!("Received {}", ctx.local.buffer_reader[i]);
                }
            }
            _ => {}
        }
    }

    #[task(
        local = [
            msg_q_receiver,
            pwm_top_right,
            pwm_bottom_right,
            pwm_bottom_left,
            pwm_top_left,
        ],
        priority = 4
    )]
    async fn esc(ctx: esc::Context) {
        loop {
            match ctx.local.msg_q_receiver.recv().await {
                Ok(byte) => {
                    // Only accept input values 0-5
                    if matches!(byte, 48 | 49 | 50 | 51 | 52 | 53) {
                        // Flip the value so that 0 is minimum and 5 maximum throttle.
                        let divisor: u16 = (10 - (byte - 48)) as u16;
                        defmt::info!("Updating PWM...");
                        ctx.local.pwm_top_right.set_duty(PWM_TOP / divisor);
                        ctx.local.pwm_bottom_right.set_duty(PWM_TOP / divisor);
                        ctx.local.pwm_bottom_left.set_duty(PWM_TOP / divisor);
                        ctx.local.pwm_top_left.set_duty(PWM_TOP / divisor);
                    } else {
                        defmt::error!("Unhandled data {}", byte);
                    }
                }
                Err(_) => {
                    defmt::error!("Msg Q error.");
                }
            }
        }
    }

    #[task(local = [nrf24l01], priority = 5)]
    async fn nrf_comms(ctx: nrf_comms::Context) {
        let nrf24l01 = ctx.local.nrf24l01;
        let status = nrf24l01.read_status();
        nrf24l01.clear_interrupts();
        match nrf24l01.get_state() {
            State::Receiver => {
                let payload = nrf24l01.read_payload();
                let result = ControllerState::deserialize(&payload);
                match result {
                    Ok(controller_state) => {
                        defmt::info!("{}", controller_state);
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
                    defmt::info!("nrf_comms::Transmit successful");
                } else if status & 0x10 != 0 {
                    // MAX_RT - transmission failed, must flush TX FIFO
                    nrf24l01.send_command(CMD_FLUSH_TX);
                    nrf24l01.set_receiver_mode();
                    defmt::error!("nrf_comms::Transmit failed (MAX_RT)");
                }
            }
            State::Standby => {}
        }
    }
}

// End of file
