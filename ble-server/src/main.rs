use esp_idf_sys as _;  // Required to link ESP-IDF framework.
use esp_idf_hal::prelude::*;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_ble::BLEDevice;
use embedded_svc::ble::server::{Server, BLEService, Characteristic, Descriptor};

const SERVICE_UUID: &str = "0000180d-0000-1000-8000-00805f9b34fb";
const CHAR_UUID: &str = "00002A37-0000-1000-8000-00805f9b34fb";
const DESC_UUID: u16 = 0x2902;

static mut DEVICE_CONNECTED: bool = false;

struct MyCallbacks;

impl embedded_svc::ble::server::Callbacks for MyCallbacks {
    fn on_connect(&mut self, _conn_handle: u16) {
        unsafe {
            DEVICE_CONNECTED = true;
        }
        println!("Connection made!");
    }

    fn on_disconnect(&mut self, _conn_handle: u16) {
        unsafe {
            DEVICE_CONNECTED = false;
        }
        println!("Connection lost!");
    }
}

fn main() {
    // Initialize logger (optional).
    esp_idf_sys::link_patches();

    println!("Starting BLE work!");

    // Initialize BLE Device.
    let ble_device = BLEDevice::new("BH BHT017302 Emulator");

    // Create BLE Server.
    let server = ble_device.create_server::<MyCallbacks>(MyCallbacks);

    // Create a BLE Service.
    let service = server.create_service(SERVICE_UUID);

    // Create a BLE Characteristic with notify property.
    let mut charac = service.add_characteristic(CHAR_UUID, Characteristic::PROPERTY_NOTIFY);
    
    // Add a descriptor.
    charac.add_descriptor(DESC_UUID);

    println!("Characteristic defined!");

    // Start the service.
    service.start();

    // Start advertising.
    server.start_advertising();

    println!("Waiting for a client connection to notify...");

    // Timer variables
    let mut last_time = FreeRtos::get_tick_count();
    let timer_delay = 1000;

    let mut value: u8 = 0;

    loop {
        unsafe {
            if DEVICE_CONNECTED && FreeRtos::get_tick_count() - last_time >= timer_delay {
                let messages: u32 = 0x02 << 24 | 0xC4 << 16 | (value as u32) << 8 | 0x16;
                let data = messages.to_be_bytes();  // Converts u32 to big-endian byte array.

                charac.notify(&data);

                // Print the data
                print!("Data: ");
                for byte in data.iter() {
                    print!("0x{:02X} ", byte);
                }
                println!();

                value += 1;
                last_time = FreeRtos::get_tick_count();
            }
        }
        // Add some delay for FreeRTOS task scheduling.
        FreeRtos::delay_ms(100);
    }
}
