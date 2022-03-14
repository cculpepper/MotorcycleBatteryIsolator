//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;


use embedded_time::rate::Extensions;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;

use embedded_time::rate::*;

use panic_halt as _;

use rp_pico::hal::prelude::*;

use rp_pico::hal::pac;
use rp_pico::hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;


#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let XTAL_FREQ = 12_000_000;

    let clocks = hal::clocks::init_clocks_and_plls(
        // rp_pico::XOSC_CRYSTAL_FREQ,
        XTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
        )
        .ok()
        .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
            ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;
    let REPEAT_INTERVAL = 1000; // ms

    let mut nextCount : u64 = (0 + (XTAL_FREQ /12* (1000 / REPEAT_INTERVAL))).into();
    let mut currentTimer = timer.get_counter();

    let _ = serial.write(b"Starting!!!\r\n");

    for _ in 1..1_000_000 {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                    break;
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
    let _ = serial.write(b"test!!!\r\n");
    let _ = serial.flush();


    for _ in 1..1_000_000 {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                    break;
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
    let _ = serial.write(b"Starting!!!\r\n");
    let _ = serial.write(b"test2!!!\r\n");
    let _ = serial.flush();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    delay.delay_ms(1000u32);
    let _ = serial.write(b"Delay!!!\r\n");
    let _ = serial.flush();

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
        );
    let _ = serial.write(b"pins\r\n");
    let _ = serial.flush();

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio22.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio21.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio25.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);
    let _ = serial.write(b"spi\r\n");
    let _ = serial.flush();

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
        );
    let _ = serial.write(b"spi init\r\n");
    let _ = serial.flush();

    //
    // cs gpio25
    // Configure Digital I/O Pin to be used as Chip Select for SPI
    let mut cs = pins.gpio25.into_push_pull_output(); //BCM7 CE0
    // cs.export().expect("cs export");
    // while !cs.is_exported() {}
    // cs.set_direction(Direction::Out).expect("CS Direction");
    // cs.set_value(1);//.expect("CS Value set to 1");
    cs.set_high();
    loop{}
}
