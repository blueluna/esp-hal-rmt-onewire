#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    rmt::{asynch::*, *},
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_hal_rmt_onewire::{Address, OneWire};
use esp_println::println;
use heapless::{FnvIndexMap, Vec};

pub struct DeviceData {
    pub temperature: Option<fixed::types::I12F4>,
}

impl DeviceData {
    pub fn new() -> Self {
        Self { temperature: None }
    }

    pub fn set_temperature_bytes(&mut self, bytes: &[u8; 2]) -> bool {
        let temperature = fixed::types::I12F4::from_le_bytes(*bytes);
        let changed = if let Some(last_temperature) = self.temperature {
            last_temperature != temperature
        } else {
            true
        };
        self.temperature = Some(temperature);
        changed
    }
}

type DeviceDataMap = FnvIndexMap<Address, DeviceData, 32>;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let clocks =
        ClockControl::boot_defaults(SystemControl::new(peripherals.SYSTEM).clock_control).freeze();
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timer_group0.timer0);
    let rmt = Rmt::new_async(peripherals.RMT, 80_u32.MHz(), &clocks).unwrap();
    let mut ow = OneWire::new(rmt.channel0, rmt.channel2, io.pins.gpio6);

    let mut devices: DeviceDataMap = FnvIndexMap::new();

    let mut scan_period = 0i32;

    let mut ticker = Ticker::every(Duration::from_secs(10));

    loop {
        let milliseconds_now = embassy_time::Instant::now().as_millis();
        // println!("{}", milliseconds_now);

        if !ow.reset().await {
            println!("Failed to reset bus");
        }

        for a in [0xCC, 0x44] {
            ow.send_byte(a).await;
        }
        Timer::after(Duration::from_secs(1)).await;

        if scan_period <= 0 {
            search(&mut ow, &mut devices).await;
            scan_period = 6;
        } else {
            scan_period = scan_period.saturating_sub(1)
        }

        for (address, device) in devices.iter_mut() {
            if address.family() == esp_hal_rmt_onewire::AddressFamily::Temperature {
                ow.reset().await;
                ow.send_byte(0x55).await;
                ow.send_address(address).await;
                ow.send_byte(0xBE).await;
                let temp_low = ow.receive_byte().await;
                let temp_high = ow.receive_byte().await;
                let changed = device.set_temperature_bytes(&[temp_low, temp_high]);
                if changed {
                    if let Some(temperature) = device.temperature {
                        println!("{:8} {}: {:8.4}", milliseconds_now, address, temperature);
                    }
                }
            }
        }
        ticker.next().await;
    }
}

pub async fn search<R: RxChannelAsync, T: TxChannelAsync>(
    ow: &mut OneWire<R, T>,
    devices: &mut DeviceDataMap,
) {
    let mut search = esp_hal_rmt_onewire::Search::new();
    let mut found_devices: Vec<Address, 32> = Vec::new();
    loop {
        match search.next(ow).await {
            Ok(address) => {
                if let Err(_) = found_devices.push(address) {
                    break;
                }
            }
            Err(_) => {
                break;
            }
        }
    }
    let mut remove_devices: Vec<Address, 32> = Vec::new();
    for address in devices.keys() {
        if !found_devices.contains(address) {
            let _ = remove_devices.push(*address);
        }
    }
    for address in found_devices {
        if !devices.contains_key(&address) {
            println!("Add device {}", address);
            let _ = devices.insert(address, DeviceData::new());
        }
    }
    for address in remove_devices {
        println!("Remove device {}", address);
        devices.remove(&address);
    }
}
