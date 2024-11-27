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

// #[embassy_executor::task]
// async fn writer(
//     mut tx: UartTx<'static, UART0, Async>,
//     signal: &'static Signal<NoopRawMutex, usize>,
// ) {
//     use core::fmt::Write;
//     embedded_io_async::Write::write(
//         &mut tx,
//         b"RUST Hello async serial. Enter something ended with EOT (CTRL-D).\r\n",
//     )
//     .await
//     .unwrap();
//     embedded_io_async::Write::flush(&mut tx).await.unwrap();
//     loop {
//         let bytes_read = signal.wait().await;
//         signal.reset();
//         write!(&mut tx, "\r\n-- RUST received {} bytes --\r\n", bytes_read).unwrap();
//         embedded_io_async::Write::flush(&mut tx).await.unwrap();
//     }
// }


// #[embassy_executor::task]
// async fn reader(
//     mut rx: UartRx<'static, UART0, Async>,
//     signal: &'static Signal<NoopRawMutex, usize>,
// ) {
//     const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

//     let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
//     let mut offset = 0;
//     loop {
//         let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
//         match r {
//             Ok(len) => {
//                 offset += len;
//                 esp_println::println!(
//                     "RUST - Read: {len}, data: {:?}",
//                     &rbuf[..offset]
//                         .iter()
//                         .map(|&b| b as char)
//                         .collect::<heapless::String<256>>()
//                 ); // prints ascii values as text
//                 offset = 0;
//                 signal.signal(len);
//             }
//             Err(e) => esp_println::println!("RX Error: {:?}", e),
//         }
//     }
// }

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
    mut rng: Rng
) {
    // let mut rng = rand::thread_rng();  // Initialize random number generator

    loop {
        // Generate a random bit (0 or 1)
        let bit = rng.gen_range(0..2);

        // Send the bit over UART
        embedded_io_async::Write::write(&mut tx, &[bit]).await.unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
        
        // Print debug log for the bit being sent
        esp_println::println!("RUST Sent bit: {}", bit);

        // Wait for 1 second before sending the next bit
        embassy_time::Timer::after(Duration::from_secs(1)).await;

        // Optionally, signal the writer has sent a bit (though it's not required here)
        signal.signal(1);
    }
}


#[embassy_executor::task]
async fn reader(
    mut rx: UartRx<'static, UART0, Async>,
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
                // Print received data (should be 1 byte: the random bit)
                esp_println::println!("RUST Received bit: {}", rbuf[0]);
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
    // let button = Input::new(io.pins.gpio0, Pull::Down);

    // let init = &*make_static!(
    //     EspWifiInitialization,
    //     init(
    //         EspWifiInitFor::Ble,
    //         timg0.timer1,
    //         rng,
    //         peripherals.RADIO_CLK,
    //     )
    //     .unwrap()
    // );
    // let mut bluetooth = peripherals.BT;
    // let connector = BleConnector::new(&init, &mut bluetooth);

    // let now = || time::now().duration_since_epoch().to_millis();
    // let mut ble = Ble::new(connector, now);
    // println!("Connector created");

    // let pin_ref = RefCell::new(button);
    // let pin_ref = &pin_ref;

    spawn.must_spawn(boot_button_reply(io.pins.gpio0));

    // //default tx,rx = 1,3
    // let tx_pin = io.pins.gpio17;
    // let rx_pin = io.pins.gpio16;
    let tx_pin = io.pins.gpio1;
    let rx_pin = io.pins.gpio3;

    let config = Config::default()
        .baudrate(115_200)
        .stop_bits(esp_hal::uart::config::StopBits::STOP1)
        .data_bits(esp_hal::uart::config::DataBits::DataBits8)
        .parity_none()
        .rx_fifo_full_threshold(READ_BUF_SIZE as u16);
    let mut uart = Uart::new_async_with_config(peripherals.UART0, config, rx_pin, tx_pin).unwrap();
    // let mut uart = Uart::new_async(peripherals.UART0, rx_pin, tx_pin).unwrap();
    uart.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    let (rx, tx) = uart.split();
    spawn.must_spawn(writer(tx, &signal, rng));
    spawn.must_spawn(reader(rx, &signal));

    // loop {
    //     println!("{:?}", ble.init().await);
    //     println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
    //     println!(
    //         "{:?}",
    //         ble.cmd_set_le_advertising_data(
    //             create_advertising_data(&[
    //                 AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
    //                 AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
    //                 AdStructure::CompleteLocalName(esp_hal::chip!()),
    //             ])
    //             .unwrap()
    //         )
    //         .await
    //     );
    //     println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

    //     println!("started advertising");

    //     let mut rf = |_offset: usize, data: &mut [u8]| {
    //         data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
    //         17
    //     };
    //     let mut wf = |offset: usize, data: &[u8]| {
    //         println!("RECEIVED: {} {:?}", offset, data);
    //     };

    //     let mut wf2 = |offset: usize, data: &[u8]| {
    //         println!("RECEIVED: {} {:?}", offset, data);
    //     };

    //     let mut rf3 = |_offset: usize, data: &mut [u8]| {
    //         data[..5].copy_from_slice(&b"Hola!"[..]);
    //         5
    //     };
    //     let mut wf3 = |offset: usize, data: &[u8]| {
    //         println!("RECEIVED: Offset {}, data {:?}", offset, data);
    //     };

    //     // BLE Server - Define GATT Service and Characteristics
    //     gatt!([service {
    //         uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
    //         characteristics: [
    //             characteristic {
    //                 uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
    //                 read: rf,
    //                 write: wf,
    //             },
    //             characteristic {
    //                 uuid: "957312e0-2354-11eb-9f10-fbc30a62cf38",
    //                 write: wf2,
    //             },
    //             characteristic {
    //                 name: "my_characteristic",
    //                 uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
    //                 notify: true,
    //                 read: rf3,
    //                 write: wf3,
    //             },
    //         ],
    //     },]);

    //     let mut rng = bleps::no_rng::NoRng;
    //     let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

    //     let counter = RefCell::new(0u8);
    //     let counter = &counter;

    //     let mut notifier = || {
    //         // TODO how to check if notifications are enabled for the characteristic?
    //         // maybe pass something into the closure which just can query the characteristic
    //         // value probably passing in the attribute server won't work?

    //         async {
    //             pin_ref.borrow_mut().wait_for_rising_edge().await;
    //             let mut data = [0u8; 13];
    //             data.copy_from_slice(b"Notification0");
    //             {
    //                 let mut counter = counter.borrow_mut();
    //                 data[data.len() - 1] += *counter;
    //                 *counter = (*counter + 1) % 10;
    //             }
    //             NotificationData::new(my_characteristic_handle, &data)
    //         }
    //     };

    //     srv.run(&mut notifier).await.unwrap();
    // }
    //spawn bluetooth listener, and a UART listener
}
