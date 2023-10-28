use display_interface_spi::SPIInterface;
use rp_pico::hal::{self, gpio, spi};

pub type SpiType = hal::Spi<
    spi::Enabled,
    hal::pac::SPI0,
    (
        gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSpi, gpio::PullDown>,
    ),
    8,
>;

pub type SPIInterfaceType = SPIInterface<
    SpiType,
    gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
    gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
>;

pub type DisplayType = mipidsi::Display<
    SPIInterfaceType,
    mipidsi::models::ILI9341Rgb565,
    gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
>;
