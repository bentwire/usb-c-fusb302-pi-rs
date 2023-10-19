#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;

use panic_probe as _;
use usb_pd::{
    callback::{Event, Response},
    pdo::PowerDataObject,
};

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

    use defmt::info;
    use embedded_hal::digital::v2::OutputPin;
    // use embedded_hal::prelude::{
    //     _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Read,
    //     _embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead,
    // };
    use fugit::{ExtU64, Instant};
    use fusb302b::Fusb302b;
    use rp_pico::hal::{
        self,
        clocks::init_clocks_and_plls,
        gpio, i2c,
        pac::I2C0,
        sio,
        timer::{monotonic::Monotonic, Alarm0},
        watchdog::Watchdog,
        Sio,
    };
    // use usb_pd::sink::{Driver, DriverState};

    #[shared]
    struct Shared {
        leds: (
            gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
        ),
        // i2c0: rp2040_hal::I2C<
        //     I2C0,
        //     (
        //         rp2040_hal::gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2c, gpio::PullDown>,
        //         rp2040_hal::gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2c, gpio::PullDown>,
        //     ),
        // >,
        i2c0: I2C0Proxy,
        //fusb302: FUSB302BDev,
        pdev: PDDev,
        // pd: usb_pd::sink::Sink<
        //     fusb302b::Fusb302b<
        //         rp2040_hal::I2C<
        //             I2C0,
        //             (
        //                 rp2040_hal::gpio::Pin<
        //                     gpio::bank0::Gpio0,
        //                     gpio::FunctionI2c,
        //                     gpio::PullDown,
        //                 >,
        //                 rp2040_hal::gpio::Pin<
        //                     gpio::bank0::Gpio1,
        //                     gpio::FunctionI2c,
        //                     gpio::PullDown,
        //                 >,
        //             ),
        //         >,
        //     >,
        // >,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[local]
    struct Local {
        // pd: usb_pd::sink::Sink<
        //     fusb302b::Fusb302b<
        //         rp2040_hal::I2C<
        //             I2C0,
        //             (
        //                 rp2040_hal::gpio::Pin<
        //                     gpio::bank0::Gpio0,
        //                     gpio::FunctionI2c,
        //                     gpio::PullDown,
        //                 >,
        //                 rp2040_hal::gpio::Pin<
        //                     gpio::bank0::Gpio1,
        //                     gpio::FunctionI2c,
        //                     gpio::PullDown,
        //                 >,
        //             ),
        //         >,
        //     >,
        // >,
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

        let i2c0 = i2c::I2C::i2c0(
            c.device.I2C0,
            i2c0_pins.0,
            i2c0_pins.1,
            fugit::RateExtU32::Hz(100000_u32),
            &mut resets,
            rp2040_hal::Clock::freq(&clocks.system_clock),
        );

        let i2c_bus0: &'static _ = shared_bus::new_cortexm!(I2C0Dev = i2c0).unwrap();

        //let mut fusb302 = fusb302b::Fusb302b::new(i2c_bus0.acquire_i2c());

        //fusb302.init();

        let mut pd = usb_pd::sink::Sink::new(
            fusb302b::Fusb302b::new(i2c_bus0.acquire_i2c()),
            &crate::callback,
        );

        pd.init();

        let mut leds = (
            pins.gpio11.into_push_pull_output(),
            pins.gpio12.into_push_pull_output(),
            pins.gpio13.into_push_pull_output(),
            pins.gpio14.into_push_pull_output(),
        );
        leds.0.set_high().unwrap();
        leds.1.set_high().unwrap();
        leds.2.set_high().unwrap();
        leds.3.set_low().unwrap();

        let mut timer = rp2040_hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let alarm = timer.alarm_0().unwrap();
        i2c0_task::spawn_after(1.millis()).unwrap();
        blink_led::spawn_after(500.millis()).unwrap();

        let i2c0 = i2c_bus0.acquire_i2c();

        (
            Shared {
                leds,
                i2c0,
                pdev: pd,
            },
            Local {},
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(shared = [i2c0, pdev])]
    fn i2c0_task(mut c: i2c0_task::Context) {
        info!("i2c0");

        // c.shared.i2c0.lock(|i2c0| {
        //     for i in 0..=127 {
        //         let mut readbuf: [u8; 1] = [0; 1];
        //         let result = i2c0.read(i, &mut readbuf);
        //         if let Ok(_d) = result {
        //             let mut readbuf: [u8; 0x43] = [0; 0x43];
        //             // Do whatever work you want to do with found devices
        //             info!("Device found at address {:?}", i);
        //             i2c0.write(i, &[0x0c, 0x01]).unwrap();
        //             i2c0.write(i, &[0x0b, 0x0f]).unwrap();
        //             i2c0.write(i, &[0x06, 0x00]).unwrap();
        //             i2c0.write(i, &[0x09, 0x07]).unwrap();

        //             i2c0.write_read(i, &[0x01], &mut readbuf).unwrap();
        //             info!("VER? {:x}", readbuf[0]);
        //             info!("SW0: {:x}", readbuf[1]);
        //             info!("SW1: {:x}", readbuf[2]);
        //             info!("MEA: {:x}", readbuf[3])
        //         }
        //     }
        //     info!("Scan Done")
        // });
        c.shared.pdev.lock(|f| {
            let now = monotonics::MyMono::now();
            let now: Instant<u64, 1, 1000> = Instant::<u64, 1, 1000>::from_ticks(now.ticks());

            f.poll(now);
        });

        i2c0_task::spawn_after(1.millis()).unwrap();
    }
    #[task(
        shared = [leds],
        local = [tog: bool = false, which: u8 = 0],
    )]
    fn blink_led(mut c: blink_led::Context) {
        info!("BLINK {:?}", *c.local.which);
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
        blink_led::spawn_after(500.millis()).unwrap();
    }
}

fn callback(event: Event) -> Option<Response> {
    match event {
        Event::SourceCapabilitiesChanged(caps) => {
            info!("Capabilities changed: {}", caps.len());

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

            return Some(Response::RequestPower {
                index,
                current: supply.max_current() * 10,
            });
        }
        Event::PowerReady => info!("power ready"),
        Event::ProtocolChanged => info!("protocol changed"),
        Event::PowerAccepted => info!("power accepted"),
        Event::PowerRejected => info!("power rejected"),
    }

    None
}
