use esp_idf_sys as _;
use esp_idf_hal::prelude::*;
use esp_idf_hal::serial::*;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_ble::client::{BLEClient, BLEAdvertisedDevice, BLEScan};
use esp_idf_ble::characteristics::BLECharacteristic;
use esp_idf_ble::uuid::{Uuid, BluetoothUuid};

// UUID for Heart Rate Service and Characteristic
const SERVICE_UUID: Uuid = Uuid::from_u16(0x180D);
const CHARACTERISTIC_UUID: Uuid = Uuid::from_u16(0x2A37);

fn main() -> ! {
    esp_idf_sys::link_patches();

    let mut uart2 = Uart::new(SerialConfig::default().baudrate(115_200.bps()), 
                              UartPins {
                                  tx: 27,
                                  rx: 12,
                                  ..Default::default()
                              }).unwrap();

    let client = BLEClient::new().unwrap();
    let scan = BLEScan::new(client.clone());
    
    // Start scanning for BLE devices
    println!("Starting BLE scan...");
    scan.start(5000, true).unwrap();

    // Scan callback to find devices
    scan.on_result(|device: BLEAdvertisedDevice| {
        if device.has_service_uuid(SERVICE_UUID) {
            println!("Found device advertising Heart Rate service: {:?}", device.address());
            client.connect(device.address()).unwrap();
            let characteristic = client.discover_characteristic(CHARACTERISTIC_UUID).unwrap();
            characteristic.set_notify_callback(notify_callback).unwrap();
            client.subscribe(&characteristic).unwrap();
        }
    });

    loop {
        FreeRtos::delay_ms(1000);
    }
}

fn notify_callback(data: Vec<u8>) {
    // Process incoming BLE notifications for heart rate measurement
    let bpm = if (data[0] & 0x01) == 0 { data[1] as u16 } else { ((data[1] as u16) << 8) | data[2] as u16 };

    println!("Heart Rate: {} bpm", bpm);

    // Send heart rate over UART
    uart2.write(&[bpm as u8]).unwrap();
    uart2.flush().unwrap();
}
