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

    use embedded_hal::digital::v2::OutputPin;
    use fugit::{ExtU64, Instant};
    use rp2040_hal::{
        self,
        clocks::init_clocks_and_plls,
        gpio, i2c,
        pac::I2C0,
        sio,
        timer::{monotonic::Monotonic, Alarm0},
        watchdog::Watchdog,
        Sio,
    };

    #[shared]
    struct Shared {
        led: gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSio<gpio::SioOutput>, gpio::PullDown>,
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
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio0.into_function::<gpio::FunctionI2C>(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
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

        let mut pd = usb_pd::sink::Sink::new(fusb302b::Fusb302b::new(i2c0), &crate::callback);

        pd.init();

        let mut led = pins.gpio11.into_push_pull_output();
        led.set_low().unwrap();

        let mut timer = rp2040_hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let alarm = timer.alarm_0().unwrap();
        blink_led::spawn_after(500.millis()).unwrap();

        (
            Shared { led },
            Local {},
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(
        shared = [led],
        local = [tog: bool = true],
    )]
    fn blink_led(mut c: blink_led::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }
        *c.local.tog = !*c.local.tog;

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
