use std::borrow::Cow;
use std::io::Write;
use std::iter::once;
use std::mem::size_of;

use super::{ChipType, EspCommonHeader, SegmentHeader, ESP_MAGIC};
use crate::chip::{Chip, SpiRegisters};
use crate::elf::{update_checksum, CodeSegment, FirmwareImage, RomSegment, ESP_CHECKSUM_MAGIC};
use crate::flasher::FlashSize;
use crate::Error;
use bytemuck::bytes_of;

pub const IROM_MAP_START: u32 = 0x40200000;
const IROM_MAP_END: u32 = 0x40300000;

pub struct Esp8266;

impl ChipType for Esp8266 {
    const CHIP_DETECT_MAGIC_VALUE: u32 = 0xfff0c101;
    const SPI_REGISTERS: SpiRegisters = SpiRegisters {
        base: 0x60000200,
        usr_offset: 0x1c,
        usr1_offset: 0x20,
        usr2_offset: 0x24,
        w0_offset: 0x40,
        mosi_length_offset: None,
        miso_length_offset: None,
    };

    fn addr_is_flash(addr: u32) -> bool {
        (IROM_MAP_START..IROM_MAP_END).contains(&addr)
    }

    fn get_flash_segments<'a>(
        image: &'a FirmwareImage,
    ) -> Box<dyn Iterator<Item = Result<RomSegment<'a>, Error>> + 'a> {
        // irom goes into a separate plain bin
        let irom_data = merge_rom_segments(image.rom_segments(Chip::Esp8266))
            .into_iter()
            .map(Ok);

        // my kingdom for a try {} block
        fn common<'a>(image: &'a FirmwareImage) -> Result<RomSegment<'a>, Error> {
            let mut common_data = Vec::with_capacity(
                image
                    .ram_segments(Chip::Esp8266)
                    .map(|segment| segment.size as usize)
                    .sum(),
            );
            // common header
            let header = EspCommonHeader {
                magic: ESP_MAGIC,
                segment_count: image.ram_segments(Chip::Esp8266).count() as u8,
                flash_mode: image.flash_mode as u8,
                flash_config: encode_flash_size(image.flash_size)? + image.flash_frequency as u8,
                entry: image.entry,
            };
            common_data.write_all(bytes_of(&header))?;

            let mut total_len = 8;

            let mut checksum = ESP_CHECKSUM_MAGIC;

            for segment in image.ram_segments(Chip::Esp8266) {
                let data = segment.data;
                let padding = 4 - data.len() % 4;
                let segment_header = SegmentHeader {
                    addr: segment.addr,
                    length: (data.len() + padding) as u32,
                };
                total_len += size_of::<SegmentHeader>() as u32 + segment_header.length;
                common_data.write_all(bytes_of(&segment_header))?;
                common_data.write_all(data)?;

                let padding = &[0u8; 4][0..padding];
                common_data.write_all(padding)?;
                checksum = update_checksum(data, checksum);
            }

            let padding = 15 - (total_len % 16);
            let padding = &[0u8; 16][0..padding as usize];
            common_data.write_all(padding)?;

            common_data.write_all(&[checksum])?;

            Ok(RomSegment {
                addr: 0,
                data: Cow::Owned(common_data),
            })
        }

        Box::new(irom_data.chain(once(common(image))))
    }
}

fn encode_flash_size(size: FlashSize) -> Result<u8, Error> {
    match size {
        FlashSize::Flash256Kb => Ok(0x10),
        FlashSize::Flash512Kb => Ok(0x00),
        FlashSize::Flash1Mb => Ok(0x20),
        FlashSize::Flash2Mb => Ok(0x30),
        FlashSize::Flash4Mb => Ok(0x40),
        FlashSize::Flash8Mb => Ok(0x80),
        FlashSize::Flash16Mb => Ok(0x90),
        FlashSize::FlashRetry => Err(Error::UnsupportedFlash(size as u8)),
    }
}

fn merge_rom_segments<'a>(
    mut segments: impl Iterator<Item = CodeSegment<'a>>,
) -> Option<RomSegment<'a>> {
    let first = segments.next()?;
    if let Some(second) = segments.next() {
        let mut data = Vec::with_capacity(first.data.len() + second.data.len());
        data.extend_from_slice(first.data);

        for segment in once(second).chain(segments) {
            let padding_size = segment.addr as usize - first.addr as usize - data.len();
            data.resize(data.len() + padding_size, 0);
            data.extend_from_slice(segment.data);
        }

        Some(RomSegment {
            addr: first.addr - IROM_MAP_START,
            data: Cow::Owned(data),
        })
    } else {
        Some(RomSegment {
            addr: first.addr - IROM_MAP_START,
            data: Cow::Borrowed(first.data),
        })
    }
}

#[test]
fn test_esp8266_rom() {
    use pretty_assertions::assert_eq;
    use std::fs::read;

    let input_bytes = read("./tests/data/esp8266").unwrap();
    let expected_bin = read("./tests/data/esp8266.bin").unwrap();

    let image = FirmwareImage::from_data(&input_bytes).unwrap();

    let segments = Esp8266::get_flash_segments(&image)
        .collect::<Result<Vec<_>, Error>>()
        .unwrap();

    assert_eq!(1, segments.len());
    let buff = segments[0].data.as_ref();
    assert_eq!(expected_bin.len(), buff.len());
    assert_eq!(expected_bin.as_slice(), buff);
}
