#![no_std]
#![no_main]

//mod display;
use defmt_rtt as _;

use panic_probe as _;
#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [I2C0_IRQ])]
mod app {
    type I2C0Dev = hal::I2C<
        I2C0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2c, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2c, gpio::PullDown>,
        ),
    >;
    type I2C0Proxy = shared_bus::I2cProxy<
        'static,
        shared_bus::cortex_m::interrupt::Mutex<
            core::cell::RefCell<
                rp2040_hal::I2C<
                    I2C0,
                    (
                        rp2040_hal::gpio::Pin<
                            gpio::bank0::Gpio0,
                            gpio::FunctionI2c,
                            gpio::PullDown,
                        >,
                        rp2040_hal::gpio::Pin<
                            gpio::bank0::Gpio1,
                            gpio::FunctionI2c,
                            gpio::PullDown,
                        >,
                    ),
                >,
            >,
        >,
    >;

    #[allow(dead_code)]
    type FUSB302BDev = Fusb302b<
        shared_bus::I2cProxy<
            'static,
            cortex_m::interrupt::Mutex<
                core::cell::RefCell<
                    hal::I2C<
                        I2C0,
                        (
                            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2c, gpio::PullDown>,
                            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2c, gpio::PullDown>,
                        ),
                    >,
                >,
            >,
        >,
    >;

    type PDDev = usb_pd::sink::Sink<
        Fusb302b<
            shared_bus::I2cProxy<
                'static,
                cortex_m::interrupt::Mutex<
                    core::cell::RefCell<
                        hal::I2C<
                            I2C0,
                            (
                                gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2c, gpio::PullDown>,
                                gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2c, gpio::PullDown>,
                            ),
                        >,
                    >,
                >,
            >,
        >,
    >;

    #[allow(dead_code)]
    type I2C0Bus = shared_bus::BusManager<
        shared_bus::NullMutex<
            hal::I2C<
                I2C0,
                (
                    gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2c, gpio::PullDown>,
                    gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2c, gpio::PullDown>,
                ),
            >,
        >,
    >;

    use defmt::{debug, info};
    use usb_pd::{
        pdo::PowerDataObject,
        sink::{Event, Request, Sink},
    };

    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    // use embedded_hal::prelude::{
    //     _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Read,
    //     _embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead,
    // };
    use fugit::{ExtU64, Instant, RateExtU32};
    use fusb302b::Fusb302b;

    use rp_pico::hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        gpio, i2c,
        pac::I2C0,
        sio, spi,
        timer::{monotonic::Monotonic, Alarm0},
        watchdog::Watchdog,
        Sio,
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
            // SPI0 TX
            pins.gpio7.into_function::<gpio::FunctionSpi>(),
            // SPI0 RX
            pins.gpio4.into_function::<gpio::FunctionSpi>(),
            // SPI0 CLK
            pins.gpio6.into_function::<gpio::FunctionSpi>(),
        );

        let _dis_cs = pins.gpio5.into_push_pull_output();
        let _dis_dc = pins.gpio9.into_push_pull_output();
        let _dis_rst = pins.gpio10.into_push_pull_output();

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

        //let mut fusb302 = fusb302b::Fusb302b::new(i2c_bus0.acquire_i2c());

        //fusb302.init();

        let mut pd = Sink::new(fusb302b::Fusb302b::new(i2c_bus0.acquire_i2c()));

        pd.init();

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
        pd_task::spawn_after(ExtU64::millis(1)).unwrap();
        //blink_led::spawn_after(ExtU64::millis(1000)).unwrap();

        let i2c0 = i2c_bus0.acquire_i2c();

        let spi0 = spi::Spi::<_, _, _, 8>::new(c.device.SPI0, spi0_pins);
        let _spi0 = spi0.init(
            &mut resets,
            clocks.system_clock.freq(),
            10_u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        let _delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().to_Hz());

        info!("INIT COMPLETE!");

        (
            Shared { leds, i2c0 },
            Local { pdev: pd },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(local = [pdev], shared = [leds])]
    fn pd_task(mut c: pd_task::Context) {
        //info!("IDLE!");
        //loop {
        let now = monotonics::MyMono::now();
        let now: Instant<u64, 1, 1000> = Instant::<u64, 1, 1000>::from_ticks(now.ticks() / 1000);

        let evnt = c.local.pdev.poll(now);
        if let Some(evnt) = evnt {
            match evnt {
                Event::SourceCapabilitiesChanged(caps) => {
                    info!("CAPS CHANGED!");
                    c.shared.leds.lock(|leds| {
                        leds.0.toggle().unwrap();
                    });

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
        }

        //pd_task::spawn().unwrap();
        pd_task::spawn_after(50_u64.micros()).unwrap();
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
