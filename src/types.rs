use display_interface_spi::SPIInterface;
use fusb302b::Fusb302b;
use rp_pico::hal::{
    self, gpio,
    pac::{I2C0, SPI0},
    spi,
};

pub type SPI0Dev = hal::spi::Spi<
    spi::Enabled,
    SPI0,
    (
        gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSpi, gpio::PullDown>,
    ),
    8,
>;

#[allow(dead_code)]
pub type SPI0Proxy = shared_bus::SpiProxy<
    'static,
    shared_bus::NullMutex<
        rp2040_hal::Spi<
            spi::Enabled,
            SPI0,
            (
                gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSpi, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSpi, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSpi, gpio::PullDown>,
            ),
            8,
        >,
    >,
>;

pub type I2C0Dev = hal::I2C<
    I2C0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2c, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2c, gpio::PullDown>,
    ),
>;
pub type I2C0Proxy = shared_bus::I2cProxy<
    'static,
    shared_bus::cortex_m::interrupt::Mutex<core::cell::RefCell<I2C0Dev>>,
>;

pub type FUSB302BDev = Fusb302b<I2C0Proxy>;
pub type PDDev = usb_pd::sink::Sink<FUSB302BDev>;
#[allow(dead_code)]
pub type I2C0Bus = shared_bus::BusManager<shared_bus::NullMutex<I2C0Dev>>;

pub type SPIInterfaceType = SPIInterface<
    SPI0Dev,
    gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
    gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
>;

pub type DisplayType = mipidsi::Display<
    SPIInterfaceType,
    mipidsi::models::ILI9341Rgb565,
    gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
>;
