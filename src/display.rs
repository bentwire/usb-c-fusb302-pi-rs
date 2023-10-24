use embedded_graphics::pixelcolor::Rgb565;

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::Dimensions;

use rp_pico::hal::{self, gpio, pac::SPI0, spi};

//use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use ssd1351::{
    builder::Builder, interface::SpiInterface, mode::GraphicsMode, properties::DisplayRotation,
};

pub type SSDSSPIInterface = SpiInterface<
    hal::Spi<
        spi::Enabled,
        SPI0,
        (
            gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSpi, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSpi, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSpi, gpio::PullDown>,
        ),
        8,
    >,
    gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
>;

//pub type Ssd1351 = ssd1351::mode::GraphicsMode<SSDSSPIInterface>;
pub type Ssd1351<SPI, DC> = ssd1351::mode::GraphicsMode<SpiInterface<SPI, DC>>;

pub struct Display<
    SPI: embedded_hal::blocking::spi::write::Default<u8>,
    DC: embedded_hal::digital::v2::OutputPin,
>(pub Ssd1351<SPI, DC>);

impl<
        SPI: embedded_hal::blocking::spi::write::Default<u8>,
        DC: embedded_hal::digital::v2::OutputPin,
    > Display<SPI, DC>
{
    pub fn new<RST>(spi: SPI, dc: DC, rst: &mut RST, delay: &mut cortex_m::delay::Delay) -> Self
    where
        SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
        DC: embedded_hal::digital::v2::OutputPin,
        RST: embedded_hal::digital::v2::OutputPin,
        <RST as embedded_hal::digital::v2::OutputPin>::Error: Debug,
    {
        let mut display: GraphicsMode<_> = Builder::new()
            .with_rotation(DisplayRotation::Rotate0)
            .with_size(ssd1351::properties::DisplaySize::Display128x96)
            .connect_spi(spi, dc)
            .into();

        display.reset(rst, delay).expect("RESET FAIL");

        display.init().unwrap();

        Self(display)
    }
}

impl<
        SPI: embedded_hal::blocking::spi::write::Default<u8>,
        DC: embedded_hal::digital::v2::OutputPin,
    > embedded_graphics::draw_target::DrawTarget for Display<SPI, DC>
{
    type Color = Rgb565;

    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        self.0.draw_iter(pixels)
    }
}

impl<
        SPI: embedded_hal::blocking::spi::write::Default<u8>,
        DC: embedded_hal::digital::v2::OutputPin,
    > embedded_graphics::geometry::Dimensions for Display<SPI, DC>
{
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        self.0.bounding_box()
    }
}

impl<
        SPI: embedded_hal::blocking::spi::write::Default<u8>,
        DC: embedded_hal::digital::v2::OutputPin,
    > core::ops::Deref for Display<SPI, DC>
{
    type Target = Ssd1351<SPI, DC>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<
        SPI: embedded_hal::blocking::spi::write::Default<u8>,
        DC: embedded_hal::digital::v2::OutputPin,
    > core::ops::DerefMut for Display<SPI, DC>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
