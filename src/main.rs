#![no_std]
#![no_main]

mod types;
//mod display;
use defmt_rtt as _;

use panic_probe as _;
#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [I2C0_IRQ, SPI0_IRQ])]
mod app {

    //use cortex_m::delay;
    use crate::types::{I2C0Dev, I2C0Proxy, PDDev};

    use defmt::{debug, info};
    use embedded_graphics_core::prelude::Point;
    use usb_pd::{
        pdo::{PowerDataObject},
        sink::{Event, Request, Sink},
    };

    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        //primitives::{Circle, PrimitiveStyle},
        text::{Alignment, Text},
    };
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    // use embedded_hal::prelude::{
    //     _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Read,
    //     _embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead,
    // };
    use fugit::{ExtU32, ExtU64, Instant, RateExtU32};
    use xpt2046::{self};

    use rp_pico::hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio, i2c, sio, spi,
        timer::monotonic::Monotonic,
        watchdog::Watchdog,
        Sio,
    };

    use display_interface_spi::SPIInterface;
    use mipidsi::{options::*, Builder};
    use rp2040_hal::timer::{Alarm0};
    use u8g2_fonts::{
        fonts,
        types::{FontColor, HorizontalAlignment, VerticalPosition},
        FontRenderer,
    };
    //use crate::display;

    // use usb_pd::sink::{Driver, DriverState};

    #[shared]
    struct Shared {
        leds: (
            gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
        ),
        i2c0: I2C0Proxy,
        pdos: heapless::Vec<PowerDataObject, 8>,
        display: crate::types::DisplayType,
        //fusb302: FUSB302BDev,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[local]
    struct Local {
        pdev: PDDev,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            12_000_000,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp2040_hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let i2c0_pins = (
            // SDA0 (GPIO0)
            pins.gpio0.into_function::<gpio::FunctionI2C>(),
            // SCL0 (GPIO1)
            pins.gpio1.into_function::<gpio::FunctionI2C>(),
        );

        let spi0_pins = (
            // SPI0 TX PIN 21
            pins.gpio7.into_function::<gpio::FunctionSpi>(),
            // SPI0 RX PIN 18
            pins.gpio4.into_function::<gpio::FunctionSpi>(),
            // SPI0 CLK PIN 20
            pins.gpio6.into_function::<gpio::FunctionSpi>(),
        );

        // GPIO 5 PIN 19
        let dis_cs = pins.gpio5.into_push_pull_output();
        // GPIO 9 PIN 23
        let dis_dc = pins.gpio9.into_push_pull_output();
        // GPIO 10 PIN 24
        let dis_rst = pins.gpio10.into_push_pull_output();

        let i2c0 = i2c::I2C::i2c0(
            c.device.I2C0,
            i2c0_pins.0,
            i2c0_pins.1,
            fugit::RateExtU32::Hz(100000_u32),
            &mut resets,
            rp2040_hal::Clock::freq(&clocks.system_clock),
        );

        // let i2c1 = i2c::I2C::i2c1(
        //     c.device.I2C1,
        //     i2c1_pins.0,
        //     i2c1_pins.1,
        //     fugit::RateExtU32::Hz(100000_u32),
        //     &mut resets,
        //     rp2040_hal::Clock::freq(&clocks.system_clock),
        // );

        let i2c_bus0: &'static _ = shared_bus::new_cortexm!(I2C0Dev = i2c0).unwrap();

        let mut pd = Sink::new(fusb302b::Fusb302b::new(i2c_bus0.acquire_i2c()));

        info!("INIT!");

        let mut leds = (
            pins.gpio11.into_push_pull_output(),
            pins.gpio12.into_push_pull_output(),
            pins.gpio13.into_push_pull_output(),
            pins.gpio14.into_push_pull_output(),
        );
        leds.0.set_low().unwrap();
        leds.1.set_low().unwrap();
        leds.2.set_low().unwrap();
        leds.3.set_low().unwrap();

        let mut timer = rp2040_hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let alarm = timer.alarm_0().unwrap();
        //let _ = rp2040_hal::timer::Alarm::schedule(&mut alarm, 100_u32.micros()).unwrap();
        //alarm.enable_interrupt();

        let i2c0 = i2c_bus0.acquire_i2c();

        let spi0 = spi::Spi::<_, _, _, 8>::new(c.device.SPI0, spi0_pins);
        let spi0 = spi0.init(
            &mut resets,
            clocks.system_clock.freq(),
            20u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        //let spi_bus0 = shared_bus::BusManagerSimple::new(spi0);

        //let spi0 = spi_bus0.acquire_spi();

        let mut delay =
            cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().to_Hz());

        let di = SPIInterface::new(spi0, dis_dc, dis_cs);
        let mut display = Builder::ili9341_rgb565(di)
            .with_display_size(240, 320)
            .with_orientation(Orientation::PortraitInverted(true))
            .with_color_order(ColorOrder::Bgr)
            .init(&mut delay, Some(dis_rst))
            .unwrap();

        // Clear the screen
        display.clear(Rgb565::BLACK).unwrap();
        // Create a new character style
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);

        Text::with_alignment("Init Complete!", Point::new(0, 8), style, Alignment::Left)
            .draw(&mut display)
            .unwrap();

        //let font1 = FontRenderer::new::<fonts::u8g2_font_DigitalDiscoThin_te>();
        let font1 = FontRenderer::new::<fonts::u8g2_font_crox2h_tf>();
        let font2 = FontRenderer::new::<fonts::u8g2_font_crox2c_tf>();
        let font3 = FontRenderer::new::<fonts::u8g2_font_crox2cb_tf>();

        font1
            .render_aligned(
                format_args!("Init Complete"),
                Point::new(0, 140),
                VerticalPosition::Baseline,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::CSS_BLUE_VIOLET),
                &mut display,
            )
            .unwrap();
        font2
            .render_aligned(
                format_args!("Init Complete"),
                Point::new(0, 160),
                VerticalPosition::Baseline,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::CSS_BLUE_VIOLET),
                &mut display,
            )
            .unwrap();
        font3
            .render_aligned(
                format_args!("Init Complete"),
                Point::new(0, 180),
                VerticalPosition::Baseline,
                HorizontalAlignment::Left,
                FontColor::Transparent(Rgb565::CSS_BLUE_VIOLET),
                &mut display,
            )
            .unwrap();

        pd.init();

        let pdos = heapless::Vec::<PowerDataObject, 8>::new();

        info!("INIT COMPLETE!");
        pd_task::spawn_after(ExtU64::millis(1)).unwrap();
        //blink_led::spawn_after(ExtU64::millis(1000)).unwrap();
        update_display::spawn_after(ExtU64::millis(100)).unwrap();

        (
            Shared {
                leds,
                i2c0,
                pdos,
                display,
            },
            Local { pdev: pd },
            //init::Monotonics(Monotonic::new(timer, alarm)),
            init::Monotonics(MyMono::new(timer, alarm)),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            debug!("IDLE");
            cortex_m::asm::wfi();
        }
    }

    #[task(local = [pdev], shared = [leds, pdos])]
    fn pd_task(mut c: pd_task::Context) {
        // Convert from us to ms
        let now = monotonics::MyMono::now();
        let now: Instant<u64, 1, 1000> = Instant::<u64, 1, 1000>::from_ticks(now.ticks() / 1000);

        let evnt = c.local.pdev.poll(now);
        if let Some(evnt) = evnt {
            match evnt {
                Event::SourceCapabilitiesChanged(caps) => {
                    info!("CAPS CHANGED!");
                    c.shared.leds.lock(|leds| {
                        leds.0.set_high().unwrap();
                    });

                    c.shared.pdos.lock(|pdos| {
                        pdos.clear();
                        pdos.extend_from_slice(caps.as_slice()).unwrap();
                    });

                    // Take maximum voltage
                    let (index, supply) = caps
                        .iter()
                        .enumerate()
                        .filter_map(|(i, cap)| {
                            if let PowerDataObject::FixedSupply(supply) = cap {
                                debug!(
                                    "supply @ {}: {}mV {}mA",
                                    i,
                                    supply.voltage() * 50,
                                    supply.max_current() * 10
                                );
                                Some((i, supply))
                            } else {
                                None
                            }
                        })
                        .max_by(|(_, x), (_, y)| x.voltage().cmp(&y.voltage()))
                        .unwrap();

                    info!("requesting supply {:?}@{}", supply, index);

                    c.local.pdev.request(Request::RequestPower {
                        index,
                        current: supply.max_current() * 10,
                    });
                }
                Event::PowerReady => {
                    info!("power ready");
                    c.shared.leds.lock(|leds| {
                        leds.1.set_high().unwrap();
                    });
                }
                Event::ProtocolChanged => {
                    info!("protocol changed");
                    c.shared.leds.lock(|leds| {
                        leds.2.toggle().unwrap();
                    });
                }
                Event::PowerAccepted => {
                    info!("power accepted");
                    c.shared.leds.lock(|leds| {
                        leds.3.set_high().unwrap();
                    });
                }
                Event::PowerRejected => {
                    info!("power rejected");
                    c.shared.leds.lock(|leds| {
                        leds.3.set_low().unwrap();
                        leds.1.set_low().unwrap();
                    });
                }
            }
        } else {
            c.shared.leds.lock(|leds| {
                leds.0.set_low().unwrap();
            });
        }

        //pd_task::spawn().unwrap();
        pd_task::spawn_after(50_u64.micros()).unwrap();
    }

    #[task(shared = [display, pdos], local = [count: u8 = 0])]
    fn update_display(mut c: update_display::Context) {
        let font = FontRenderer::new::<fonts::u8g2_font_crox2h_tf>();
        let text_color = Rgb565::WHITE;
        let _background_color = Rgb565::BLACK;

        let font_color = FontColor::Transparent(text_color);
        // let font_color = FontColor::WithBackground {
        //     fg: text_color,
        //     bg: background_color,
        // };

        c.shared.pdos.lock(|pdos| {
            for pdo in pdos.iter().enumerate() {
                let (i, pdo) = pdo;
                match pdo {
                    PowerDataObject::FixedSupply(supply) => {
                        c.shared.display.lock(|dis| {
                            font.render_aligned(
                                format_args!(
                                    "{}mV {}mA      {:x}",
                                    supply.voltage() * 50,
                                    supply.max_current() * 10,
                                    *c.local.count
                                ),
                                Point::new(
                                    0,
                                    30 + (i * (font.get_ascent() - font.get_descent()) as usize)
                                        as i32,
                                ),
                                VerticalPosition::Baseline,
                                HorizontalAlignment::Left,
                                font_color,
                                dis,
                            )
                            .unwrap();
                        });
                    }
                    _ => {}
                }
            }
        });
        *c.local.count += 1;
        update_display::spawn_after(500_u64.millis()).unwrap(); // 1 second
    }

    #[task(
        shared = [leds, i2c0],
        local = [tog: bool = false, which: u8 = 0],
    )]
    fn blink_led(mut c: blink_led::Context) {
        //info!("BLINK {:?}", *c.local.which);
        if *c.local.tog {
            c.shared.leds.lock(|l| match *c.local.which {
                0 => l.0.set_high().unwrap(),
                1 => l.1.set_high().unwrap(),
                2 => l.2.set_high().unwrap(),
                3 => l.3.set_high().unwrap(),
                4_u8..=u8::MAX => info!("WHAT?"),
            });
        } else {
            c.shared.leds.lock(|l| match *c.local.which {
                0 => l.0.set_low().unwrap(),
                1 => l.1.set_low().unwrap(),
                2 => l.2.set_low().unwrap(),
                3 => l.3.set_low().unwrap(),
                4_u8..=u8::MAX => info!("WHAT?"),
            });
        }
        *c.local.which = *c.local.which + 1;
        if *c.local.which >= 4 {
            *c.local.which = 0;
            *c.local.tog = !*c.local.tog;
        }

        //let now = monotonics::MyMono::now();
        //let foo: Instant<u64, 1, 1000> = Instant::<u64, 1, 1000>::from_ticks(now.ticks());
        //c.shared.pd.lock(|pd| pd.poll(foo));
        blink_led::spawn_after(1000_u64.millis()).unwrap();
    }

    // #[task(binds = TIMER_IRQ_0, shared = [display])]
    // fn timer_irq(c: timer_irq::Context) {
    //     c.shared.display.lock(|dis| {
    //         dis.update().unwrap();
    //     });
    //     let _ = rp2040_hal::timer::Alarm::schedule(
    //         &mut c.shared.display.lock(|dis| dis.get_mut().alarm),
    //         100_u32.millis(),
    //     );
    // }
}

// fn callback(event: Event) -> Option<Response> {
//     info!("CALLBACK");
//     match event {
//         Event::SourceCapabilitiesChanged(caps) => {
//             info!("Capabilities changed: {}", caps.len());

//             // Take maximum voltage
//             let (index, supply) = caps
//                 .iter()
//                 .enumerate()
//                 .filter_map(|(i, cap)| {
//                     if let PowerDataObject::FixedSupply(supply) = cap {
//                         debug!(
//                             "supply @ {}: {}mV {}mA",
//                             i,
//                             supply.voltage() * 50,
//                             supply.max_current() * 10
//                         );
//                         Some((i, supply))
//                     } else {
//                         None
//                     }
//                 })
//                 .max_by(|(_, x), (_, y)| x.voltage().cmp(&y.voltage()))
//                 .unwrap();

//             info!("requesting supply {:?}@{}", supply, index);

//             return Some(Response::RequestPower {
//                 index,
//                 current: supply.max_current() * 10,
//             });
//         }
//         Event::PowerReady => info!("power ready"),
//         Event::ProtocolChanged => info!("protocol changed"),
//         Event::PowerAccepted => info!("power accepted"),
//         Event::PowerRejected => info!("power rejected"),
//     }

//     None
// }
