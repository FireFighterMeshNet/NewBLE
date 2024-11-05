#![feature(impl_trait_in_assoc_type)] // needed for embassy's tasks on nightly for perfect sizing with generic `static`s
#![no_std]
#![no_main]

// TODO uncomment later
// #![expect(dead_code)] // Silence errors while prototyping.
// extern crate alloc;

use core::{sync::atomic::AtomicU8, u8};
use embassy_net::{
    tcp::TcpSocket, IpListenEndpoint, Ipv4Address, Ipv4Cidr, Runner, StackResources, StaticConfigV4,
};
use embassy_time::{Duration, WithTimeout};
use embedded_io_async::Write;
use esp_backtrace as _;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_hal::{
    delay, gpio::{GpioPin, Input, Io}, peripherals::{UART0, UART1, UART2}, rng::Rng, timer::timg::TimerGroup, uart::{config::{AtCmdConfig, Config}, Uart, UartRx, UartTx}, Async
};
// use esp_wifi::{
//     self
// };
use rand::{rngs::SmallRng, Rng as _, SeedableRng as _};
use static_cell::StaticCell;


type Mutex<T> = esp_hal::xtensa_lx::mutex::SpinLockMutex<T>;

/// Unsorted utilities.
#[macro_use]
pub mod util;

mod consts {
    include!(concat!(env!("OUT_DIR"), "/const_gen.rs"));

    /// SSID shared between all nodes in mesh.
    pub const SSID: &'static str = if let Some(x) = option_env!("SSID") {
        x
    } else {
        "esp-mesh-default-ssid"
    };

    /// One of Espressif's OUIs taken from <https://standards-oui.ieee.org/>
    pub const ESPRESSIF_OUI: &'static [u8] = [0x10, 0x06, 0x1C].as_slice();

    /// Maximum number of nodes supported in the network at once.
    pub const MAX_NODES: usize = 5;

    /// Protocol version.
    pub const PROT_VERSION: u8 = 0;
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

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write(
        &mut tx,
        b"Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
    )
    .await
    .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\nTX: -- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }

}

#[embassy_executor::task]
async fn reader(
    mut rx: UartRx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                esp_println::println!("RX println: Read: {len}, data: {:?}", &rbuf[..offset]);
                offset = 0;
                signal.signal(len);
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}


#[esp_hal_embassy::main]
async fn main(spawn: embassy_executor::Spawner) {
    // Provides #[global_allocator] with given number of bytes.
    // Bigger value here means smaller space left for stack.
    // `esp-wifi` recommends at least `92k` for `coex` and `72k` for wifi.
    esp_alloc::heap_allocator!(72_000);

    // Setup and configuration.
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timer = TimerGroup::new(peripherals.TIMG0);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    esp_println::logger::init_logger_from_env();
    esp_hal_embassy::init(timer.timer0);
    let mut rng = Rng::new(peripherals.RNG);
    let mut prng = {
        if let Some(seed) = consts::RNG_SEED {
            SmallRng::seed_from_u64(seed)
        } else {
            let mut buf = [0u8; 16];
            rng.read(&mut buf);
            SmallRng::from_seed(buf)
        }
    };

    spawn.must_spawn(boot_button_reply(io.pins.gpio0));

    // //default tx,rx = 1,3
    let tx_pin = io.pins.gpio25;
    let rx_pin = io.pins.gpio32;  

    let config = Config::default().rx_fifo_full_threshold(READ_BUF_SIZE as u16);
    let mut uart = Uart::new_async_with_config(peripherals.UART0, config, rx_pin, tx_pin).unwrap();
    // let mut uart = Uart::new_async(peripherals.UART0, rx_pin, tx_pin).unwrap();
    uart.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    
    
    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());
    
    
    let (rx, tx) = uart.split();
    spawn.must_spawn(writer(tx, &signal));
    spawn.must_spawn(reader(rx, &signal)); 


    //TODO
    //code up uart tx and rx on every pin to run at the same time on loop
    //then while all pins are txing or rxing, measure each to see whcih transmits signal



    
    // TODO fill in here
    //spawn bluetooth listener, and a UART listener
}
