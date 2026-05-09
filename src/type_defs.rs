
use rp235x_hal::{self as hal, gpio, uart};

pub type UartRx = uart::Reader<
    hal::pac::UART0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
    ),
>;

pub type UartTx = uart::Writer<
    hal::pac::UART0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
    ),
>;

pub type I2c1 = hal::i2c::I2C<
    hal::pac::I2C1,
    (
        gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2C, gpio::PullUp>,
        gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2C, gpio::PullUp>,
    ),
>;

pub type InterruptPin = gpio::Pin<
    gpio::bank0::Gpio13,
    gpio::FunctionSio<gpio::SioInput>,
    gpio::PullUp,
>;

pub type PwmTR =
hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm4, hal::pwm::FreeRunning>, hal::pwm::A>;

pub type PwmBR =
hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm5, hal::pwm::FreeRunning>, hal::pwm::A>;

pub type PwmBL =
hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>, hal::pwm::A>;

pub type PwmTL =
hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>, hal::pwm::A>;
