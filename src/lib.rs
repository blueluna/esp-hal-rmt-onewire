#![no_std]

use esp_hal as hal;

use embassy_futures::join::*;
use hal::gpio::*;
use hal::peripheral::*;
use hal::rmt::{self, asynch::*, PulseCode, RxChannelConfig, TxChannelConfig, *};

const BITS_MAX: usize = 12;

const SLOT_BIT_US: u16 = 60; // us
const START_BIT_US: u16 = 6; // us
const BIT_RECOVERY_US: u16 = 4; // us
const SLOT_DETECT_US: u16 = 15; // us

const ZERO_CODE: rmt::PulseCode = rmt::PulseCode {
    length1: SLOT_BIT_US,
    level1: false,
    length2: START_BIT_US + BIT_RECOVERY_US,
    level2: true,
};

const ONE_CODE: rmt::PulseCode = rmt::PulseCode {
    length1: START_BIT_US,
    level1: false,
    length2: SLOT_BIT_US + BIT_RECOVERY_US,
    level2: true,
};

pub struct OneWire<R: RxChannelAsync, T: TxChannelAsync> {
    rx: R,
    tx: T,
}

impl<R: RxChannelAsync, T: TxChannelAsync> OneWire<R, T> {
    pub fn new<
        'd,
        Tx: TxChannelCreatorAsync<'d, T, P>,
        Rx: RxChannelCreatorAsync<'d, R, P>,
        P: 'd + InputPin + OutputPin + Peripheral<P = P>,
    >(
        txcc: Tx,
        rxcc: Rx,
        mut pin: P,
    ) -> OneWire<R, T> {
        let rx_config = RxChannelConfig {
            clk_divider: 80, // 80 MHz / 80 -> 1 MHz -> 1 us per pulse
            idle_threshold: 1000,
            ..RxChannelConfig::default()
        };
        let tx_config = TxChannelConfig {
            clk_divider: 80, // 80 MHz / 80 -> 1 MHz -> 1 us per pulse
            idle_output: true,
            idle_output_level: true,
            ..TxChannelConfig::default()
        };

        let tx = txcc
            .configure(unsafe { pin.clone_unchecked() }, tx_config)
            .unwrap();
        let rx = rxcc
            .configure(unsafe { pin.clone_unchecked() }, rx_config)
            .unwrap();
        let mut pin = unsafe { pin.clone_unchecked() };
        struct Tag;
        pin.internal_pull_up(true, unsafe { core::mem::transmute(Tag) });
        pin.enable_output(true, unsafe { core::mem::transmute(Tag) });
        pin.enable_input(true, unsafe { core::mem::transmute(Tag) });
        pin.set_drive_strength(hal::gpio::DriveStrength::I40mA, unsafe {
            core::mem::transmute(Tag)
        });
        pin.enable_open_drain(true, unsafe { core::mem::transmute(Tag) });

        pin.connect_input_to_peripheral_with_options(
            hal::gpio::InputSignal::RMT_SIG_0,
            false,
            true,
            unsafe { core::mem::transmute(Tag) },
        );
        pin.connect_peripheral_to_output_with_options(
            hal::gpio::OutputSignal::RMT_SIG_0,
            false,
            false,
            false,
            true,
            unsafe { core::mem::transmute(Tag) },
        );

        OneWire { rx, tx }
    }

    pub async fn reset(&mut self) -> bool {
        let data = [
            PulseCode {
                level1: true,
                length1: 60,
                level2: false,
                length2: 600, // 600 us
            },
            PulseCode {
                level1: true,
                length1: 600, // 600 us
                level2: true,
                length2: 0,
            },
            rmt::PulseCode::default(),
        ];
        let mut indata = [rmt::PulseCode::default(); 3];

        // TODO: error handling
        let _res = self.send_and_receive(&mut indata, &data).await;

        indata[0].length1 > 0
            && indata[0].length2 > 0
            && indata[1].length1 > 100
            && indata[1].length1 < 200
    }

    pub async fn send(&mut self, data: &[PulseCode]) -> Result<(), esp_hal::rmt::Error> {
        self.tx.transmit(data).await
    }

    pub async fn send_and_receive(
        &mut self,
        indata: &mut [PulseCode],
        data: &[PulseCode],
    ) -> Result<(), esp_hal::rmt::Error> {
        // This relies on join polling in order to set up the rx & tx registers, which is not strictly documented behavior.
        let res = join(self.rx.receive(indata), self.tx.transmit(data)).await;
        res.0.and(res.1)
    }

    pub fn encode_bit(bit: bool) -> rmt::PulseCode {
        if bit {
            ONE_CODE
        } else {
            ZERO_CODE
        }
    }

    pub fn encode_sequence(bits: &[bool], codes: &mut [rmt::PulseCode]) -> usize {
        if bits.len() > codes.len() {
            return 0;
        }
        for (bit, code) in bits.iter().zip(codes.iter_mut()) {
            *code = Self::encode_bit(*bit);
        }
        bits.len()
    }

    pub fn decode_bit(code: PulseCode) -> bool {
        if !code.level1 && code.length1 < SLOT_DETECT_US {
            // at least us code
            true
        } else {
            false
        }
    }

    pub async fn receive_byte(&mut self) -> u8 {
        let mut bits = [false; 8];

        let _ = self.receive_bits(&mut bits).await;

        let mut res: u8 = 0;
        for n in 0..8 {
            if bits[n] {
                res |= 1 << n;
            }
        }
        res
    }

    pub async fn send_byte(&mut self, byte: u8) {
        let mut data = [rmt::PulseCode::default(); 9];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        // TODO: error handling
        let _res = self.tx.transmit(&data).await;
    }

    pub async fn send_bits(&mut self, send: &[bool]) -> usize {
        let bit_count = send.len();
        if bit_count >= BITS_MAX {
            return 0;
        }
        let mut codes_out = [rmt::PulseCode::default(); BITS_MAX];

        let _ = Self::encode_sequence(send, &mut codes_out[..bit_count]);
        // TODO: error handling
        let _res = self.send(&codes_out[..=bit_count]).await;
        match _res {
            Ok(()) => (),
            Err(_) => {
                return 0;
            }
        }
        send.len()
    }

    pub async fn receive_bits(&mut self, receive: &mut [bool]) -> usize {
        let bit_count = receive.len();
        if bit_count >= BITS_MAX {
            return 0;
        }
        let mut codes_out = [rmt::PulseCode::default(); BITS_MAX];
        let mut codes_in = [rmt::PulseCode::default(); BITS_MAX];
        for n in 0..bit_count {
            codes_out[n] = ONE_CODE;
        }

        let _res = self
            .send_and_receive(&mut codes_in[..bit_count], &codes_out[..=bit_count])
            .await;
        match _res {
            Ok(()) => (),
            Err(_) => {
                return 0;
            }
        }

        for n in 0..bit_count {
            receive[n] = Self::decode_bit(codes_in[n]);
        }
        bit_count
    }

    pub async fn send_u64(&mut self, val: u64) {
        for byte in val.to_le_bytes() {
            self.send_byte(byte).await;
        }
    }

    pub async fn send_address(&mut self, val: &Address) {
        self.send_u64(val.0).await
    }
}

const DEVICE_CODE_DS18S20: u8 = 0x10;
const DEVICE_CODE_DS1822: u8 = 0x22;
const DEVICE_CODE_DS18B20: u8 = 0x28;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AddressFamily {
    Temperature,
    Other,
}

impl From<Address> for AddressFamily {
    fn from(value: Address) -> Self {
        let family = value.0.to_le_bytes()[0];
        match family {
            DEVICE_CODE_DS18B20 | DEVICE_CODE_DS18S20 | DEVICE_CODE_DS1822 => Self::Temperature,
            _ => Self::Other,
        }
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub struct Address(u64);

impl core::fmt::Debug for Address {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        core::write!(f, "{:X?}", self.0.to_le_bytes())
    }
}

impl core::fmt::Display for Address {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        for k in self.0.to_le_bytes() {
            core::write!(f, "{:X}", k)?;
        }
        Ok(())
    }
}

impl core::hash::Hash for Address {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.0.hash(state);
    }
}

impl Address {
    pub fn family(&self) -> AddressFamily {
        return AddressFamily::from(*self);
    }
}

pub struct Search {
    command: u8,
    address: u64,
    #[cfg(feature = "search-masks")]
    address_mask: u64,
    last_discrepancy: Option<usize>,
    complete: bool,
}

#[derive(Debug)]
pub enum SearchError {
    SearchComplete,
    NoDevicesPresent,
}

impl Search {
    pub fn new() -> Search {
        Search {
            command: 0xF0,
            address: 0,
            #[cfg(feature = "search-masks")]
            address_mask: 0,
            last_discrepancy: None,
            complete: false,
        }
    }
    pub fn new_alarm() -> Search {
        Search {
            command: 0xEC,
            address: 0,
            #[cfg(feature = "search-masks")]
            address_mask: 0,
            last_discrepancy: None,
            complete: false,
        }
    }
    #[cfg(feature = "search-masks")]
    pub fn new_with_mask(fixed_bits: u64, bit_mask: u64) {
        Search {
            command: 0xEC,
            address: fixed_bits,
            address_mask: bit_mask,
            last_discrepancy: None,
            complete: false,
        }
    }

    pub async fn next<R: RxChannelAsync, T: TxChannelAsync>(
        &mut self,
        ow: &mut OneWire<R, T>,
    ) -> Result<Address, SearchError> {
        if self.complete {
            return Err(SearchError::SearchComplete);
        }
        let have_devices = ow.reset().await;
        let mut last_zero = None;
        ow.send_byte(self.command).await;
        let mut received_bits = [true, true];
        if have_devices {
            for id_bit_number in 0..64 {
                let current_bit = 1u64 << id_bit_number;
                let _bit_count = ow.receive_bits(&mut received_bits).await;
                let detected_bit = match received_bits {
                    #[cfg(feature = "search-masks")]
                    _ if address_mask & current_bit != 0 => address & current_bit != 0,
                    [false, true] => false,
                    [true, false] => true,
                    [true, true] => {
                        return Err(SearchError::NoDevicesPresent);
                    }
                    [false, false] => {
                        if self.last_discrepancy == Some(id_bit_number) {
                            true
                        } else if Some(id_bit_number) > self.last_discrepancy {
                            last_zero = Some(id_bit_number);
                            false
                        } else {
                            self.address & current_bit != 0
                        }
                    }
                };
                if detected_bit {
                    self.address |= current_bit;
                } else {
                    self.address &= !current_bit;
                }
                let _bit_count = ow.send_bits(&[detected_bit]).await;
            }
            self.last_discrepancy = last_zero;
            self.complete = last_zero.is_none();
            Ok(Address(self.address))
        } else {
            Err(SearchError::NoDevicesPresent)
        }
    }
}
