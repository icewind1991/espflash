use crate::elf::{FirmwareImage, RomSegment};
use crate::Error;
use bytemuck::{Pod, Zeroable};
use std::str::FromStr;

pub use esp32::Esp32;
pub use esp32c3::Esp32c3;
pub use esp8266::Esp8266;

mod esp32;
mod esp32c3;
mod esp8266;

const ESP_MAGIC: u8 = 0xe9;

pub trait ChipType {
    const CHIP_DETECT_MAGIC_VALUE: u32;
    const SPI_REGISTERS: SpiRegisters;

    fn addr_is_flash(addr: u32) -> bool;

    /// Get the firmware segments for writing an image to flash
    fn get_flash_segments<'a>(
        image: &'a FirmwareImage,
    ) -> Box<dyn Iterator<Item = Result<RomSegment<'a>, Error>> + 'a>;
}

pub struct SpiRegisters {
    base: u32,
    usr_offset: u32,
    usr1_offset: u32,
    usr2_offset: u32,
    w0_offset: u32,
    mosi_length_offset: Option<u32>,
    miso_length_offset: Option<u32>,
}

impl SpiRegisters {
    pub fn cmd(&self) -> u32 {
        self.base
    }

    pub fn usr(&self) -> u32 {
        self.base + self.usr_offset
    }

    pub fn usr1(&self) -> u32 {
        self.base + self.usr1_offset
    }

    pub fn usr2(&self) -> u32 {
        self.base + self.usr2_offset
    }

    pub fn w0(&self) -> u32 {
        self.base + self.w0_offset
    }

    pub fn mosi_length(&self) -> Option<u32> {
        self.mosi_length_offset.map(|offset| self.base + offset)
    }

    pub fn miso_length(&self) -> Option<u32> {
        self.miso_length_offset.map(|offset| self.base + offset)
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Chip {
    Esp8266,
    Esp32,
    Esp32c3,
}

impl Chip {
    pub fn from_magic_value(value: u32) -> Option<Self> {
        match value {
            Esp8266::CHIP_DETECT_MAGIC_VALUE => Some(Chip::Esp8266),
            Esp32::CHIP_DETECT_MAGIC_VALUE => Some(Chip::Esp32),
            Esp32c3::CHIP_DETECT_MAGIC_VALUE => Some(Chip::Esp32c3),
            _ => None,
        }
    }

    pub fn get_flash_segments<'a>(
        &self,
        image: &'a FirmwareImage,
    ) -> Box<dyn Iterator<Item = Result<RomSegment<'a>, Error>> + 'a> {
        match self {
            Chip::Esp8266 => Esp8266::get_flash_segments(image),
            Chip::Esp32 => Esp32::get_flash_segments(image),
            Chip::Esp32c3 => Esp32c3::get_flash_segments(image),
        }
    }

    pub fn addr_is_flash(&self, addr: u32) -> bool {
        match self {
            Chip::Esp8266 => Esp8266::addr_is_flash(addr),
            Chip::Esp32 => Esp32::addr_is_flash(addr),
            Chip::Esp32c3 => Esp32c3::addr_is_flash(addr),
        }
    }

    pub fn spi_registers(&self) -> SpiRegisters {
        match self {
            Chip::Esp8266 => Esp8266::SPI_REGISTERS,
            Chip::Esp32 => Esp32::SPI_REGISTERS,
            Chip::Esp32c3 => Esp32c3::SPI_REGISTERS,
        }
    }

    /// Get the target triplet for the chip
    pub fn target(&self) -> &'static str {
        match self {
            Chip::Esp8266 => "xtensa-esp8266-none-elf",
            Chip::Esp32 => "xtensa-esp32-none-elf",
            Chip::Esp32c3 => "riscv32imc-unknown-none-elf",
        }
    }
}

impl FromStr for Chip {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "esp32" => Ok(Chip::Esp32),
            "esp32c3" => Ok(Chip::Esp32c3),
            "esp8266" => Ok(Chip::Esp8266),
            _ => Err(Error::UnrecognizedChip),
        }
    }
}

#[derive(Copy, Clone, Zeroable, Pod, Debug)]
#[repr(C)]
struct EspCommonHeader {
    magic: u8,
    segment_count: u8,
    flash_mode: u8,
    flash_config: u8,
    entry: u32,
}

#[derive(Copy, Clone, Zeroable, Pod, Debug)]
#[repr(C)]
struct SegmentHeader {
    addr: u32,
    length: u32,
}
