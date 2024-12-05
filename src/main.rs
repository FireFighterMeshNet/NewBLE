#![feature(impl_trait_in_assoc_type)] // needed for embassy's tasks on nightly for perfect sizing with generic `static`s
#![no_std]
#![no_main]

// TODO uncomment later
// #![expect(dead_code)] // Silence errors while prototyping.
// extern crate alloc;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use core::{cell::RefCell, u8};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, WithTimeout};
use embedded_io_async::Write;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{GpioPin, Input, Io, Pull},
    peripherals::{UART0, UART1, UART2},
    prelude::*,
    rng::Rng,
    time,
    timer::timg::TimerGroup,
    uart::{
        config::{AtCmdConfig, Config},
        Uart, UartRx, UartTx,
    },
    Async,
};
use esp_println::println;
use esp_wifi::{
    ble::controller::asynch::BleConnector, init, EspWifiInitFor, EspWifiInitialization,
};
use rand::{rngs::SmallRng, Rng as _, SeedableRng as _};
use static_cell::StaticCell;

type Mutex<T> = esp_hal::xtensa_lx::mutex::SpinLockMutex<T>;

/// Unsorted utilities.
#[macro_use]
pub mod util;

mod consts {
    /// Maximum number of nodes supported in the network at once.
    pub const MAX_NODES: usize = 5;
}

/// Display a message when the button is pressed to confirm the device is still responsive.
#[embassy_executor::task]
async fn boot_button_reply(gpio0: GpioPin<0>) {
    let mut boot_button = Input::new(gpio0, esp_hal::gpio::Pull::None);

    let mut i = 0u8;
    loop {
        boot_button.wait_for_falling_edge().await;
        log::info!("I'm still alive! {i}");
        i = i.wrapping_add(1);
    }
}

// rx_fifo_full_threshold
const READ_BUF_SIZE: usize = 64;
// EOT (CTRL-D)
const AT_CMD: u8 = 0x04;
const RUST_ID: u8 = 2;


#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART2, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
    mut rng: Rng
) {

    let mut byte: u8 = 0;
    loop {
        // Generate a random bit (0 or 1)

        // Send the bit over UART
        embedded_io_async::Write::write(&mut tx, &[byte, RUST_ID]).await.unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
        
        // Print debug log for the bit being sent
        esp_println::println!("RUST-{} Sent: {}", RUST_ID, byte);
        byte += 1;

        // Wait for 1 second before sending the next bit
        embassy_time::Timer::after(Duration::from_secs(1)).await;

        // Optionally, signal the writer has sent a bit (though it's not required here)
        signal.signal(1);
    }
}


#[embassy_executor::task]
async fn reader(
    mut rx: UartRx<'static, UART2, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 64;  // Adjust buffer size

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                esp_println::println!("RUST-{} Received: {}", RUST_ID, rbuf[0]);
                esp_println::println!("RUST-{} Received: {}", RUST_ID, rbuf[1]);
                offset = 0;

                // Optionally, signal the reader has received data (though it's not required here)
                signal.signal(len);
            }
            Err(e) => esp_println::println!("RUST RX Error: {:?}", e),
        }
    }
}


#[esp_hal_embassy::main]
async fn main(spawn: embassy_executor::Spawner) {
    // Provides #[global_allocator] with given number of bytes.
    // Bigger value here means smaller space left for stack.
    // `esp-wifi` recommends at least `92k` for `coex` and `72k` for wifi.
    esp_println::println!("Init!");
    esp_alloc::heap_allocator!(72_000);

    // Setup and configuration.
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    esp_println::logger::init_logger_from_env();
    esp_hal_embassy::init(timg0.timer0);
    let mut rng = Rng::new(peripherals.RNG);
    let mut prng = {
        let mut buf = [0u8; 16];
        rng.read(&mut buf);
        SmallRng::from_seed(buf)
    };
    

    spawn.must_spawn(boot_button_reply(io.pins.gpio0));

    //UART2 => tx, rx = 17, 16
    let tx_pin = io.pins.gpio4;
    let rx_pin = io.pins.gpio5;

    let config = Config::default()
        .baudrate(115_200)
        .stop_bits(esp_hal::uart::config::StopBits::STOP1)
        .data_bits(esp_hal::uart::config::DataBits::DataBits8)
        .parity_none()
        .rx_fifo_full_threshold(READ_BUF_SIZE as u16);
    let mut uart = Uart::new_async_with_config(peripherals.UART2, config, rx_pin, tx_pin).unwrap();
    uart.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    let (rx, tx) = uart.split();
    spawn.must_spawn(writer(tx, &signal, rng));
    spawn.must_spawn(reader(rx, &signal));

}
