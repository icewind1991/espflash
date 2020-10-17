use std::mem::size_of;

use crate::chip::Chip;
use crate::connection::Connection;
use crate::elf::FirmwareImage;
use crate::encoder::SlipEncoder;
use crate::error::RomError;
use crate::Error;
use bytemuck::__core::time::Duration;
use bytemuck::{bytes_of, Pod, Zeroable};
use serial::{BaudRate, SerialPort};
use std::thread::sleep;
use std::io::Write;
use std::io::stdout;

type Encoder<'a> = SlipEncoder<'a, Box<dyn SerialPort>>;

const MAX_RAM_BLOCK_SIZE: usize = 0x1800;
const FLASH_SECTOR_SIZE: usize = 0x1000;
const FLASH_BLOCK_SIZE: usize = 0x100;
const FLASH_SECTORS_PER_BLOCK: usize = FLASH_SECTOR_SIZE / FLASH_BLOCK_SIZE;
const FLASH_WRITE_SIZE: usize = 0x400;

// registers used for chip detect
const UART_DATE_REG_ADDR: u32 = 0x60000078;
const UART_DATE_REG2_ADDR: u32 = 0x3f400074;

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
#[allow(dead_code)]
enum Command {
    FlashBegin = 0x02,
    FlashData = 0x03,
    FlashEnd = 0x04,
    MemBegin = 0x05,
    MemEnd = 0x06,
    MemData = 0x07,
    Sync = 0x08,
    WriteReg = 0x09,
    ReadReg = 0x0a,
    SpiSetParams = 0x0B,
    SpiAttach = 0x0D,
    ChangeBaud = 0x0F,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)]
#[repr(u8)]
pub enum FlashSize {
    Flash256KB = 0x12,
    Flash512KB = 0x13,
    Flash1MB = 0x14,
    Flash2MB = 0x15,
    Flash4MB = 0x16,
    Flash8MB = 0x17,
    Flash16MB = 0x18,
}

impl FlashSize {
    fn from(value: u8) -> Result<FlashSize, Error> {
        match value {
            0x12 => Ok(FlashSize::Flash256KB),
            0x13 => Ok(FlashSize::Flash512KB),
            0x14 => Ok(FlashSize::Flash1MB),
            0x15 => Ok(FlashSize::Flash2MB),
            0x16 => Ok(FlashSize::Flash4MB),
            0x17 => Ok(FlashSize::Flash8MB),
            0x18 => Ok(FlashSize::Flash16MB),
            _ => Err(Error::UnsupportedFlash(value)),
        }
    }
}

#[derive(Zeroable, Pod, Copy, Clone, Debug)]
#[repr(C)]
struct BlockParams {
    size: u32,
    sequence: u32,
    dummy1: u32,
    dummy2: u32,
}

#[derive(Zeroable, Pod, Copy, Clone, Debug)]
#[repr(C)]
struct BeginParams {
    size: u32,
    blocks: u32,
    block_size: u32,
    offset: u32,
}

#[derive(Zeroable, Pod, Copy, Clone, Debug)]
#[repr(C)]
struct WriteRegParams {
    addr: u32,
    value: u32,
    mask: u32,
    delay_us: u32,
}

#[derive(Zeroable, Pod, Copy, Clone)]
#[repr(C)]
struct EntryParams {
    no_entry: u32,
    entry: u32,
}

pub struct Flasher {
    connection: Connection,
    chip: Chip,
    flash_size: FlashSize,
}

impl Flasher {
    pub fn connect(
        serial: impl SerialPort + 'static,
        speed: Option<BaudRate>,
    ) -> Result<Self, Error> {
        let mut flasher = Flasher {
            connection: Connection::new(serial), // default baud is always 115200
            chip: Chip::Esp8266,                 // dummy, set properly later
            flash_size: FlashSize::Flash4MB,
        };
        flasher.start_connection()?;
        flasher.connection.set_timeout(Duration::from_secs(3))?;
        flasher.chip_detect()?;
        flasher.enable_flash()?;
        flasher.flash_detect()?;

        if let Some(b) = speed {
            match flasher.chip {
                Chip::Esp8266 => (), /* Not available */
                Chip::Esp32 => {
                    if b.speed() > BaudRate::Baud115200.speed() {
                        println!("WARN setting baud rate higher than 115200 can cause issues.");
                        flasher.change_baud(b)?;
                    }
                }
            }
        }

        Ok(flasher)
    }

    fn chip_detect(&mut self) -> Result<(), Error> {
        let reg1 = self.read_reg(UART_DATE_REG_ADDR)?;
        let reg2 = self.read_reg(UART_DATE_REG2_ADDR)?;
        let chip = Chip::from_regs(reg1, reg2).ok_or(Error::UnrecognizedChip)?;

        self.chip = chip;
        Ok(())
    }

    fn flash_detect(&mut self) -> Result<(), Error> {
        let flash_id = self.spi_command(0x9f, &[], 24)?;
        let size_id = flash_id >> 16;

        self.flash_size = FlashSize::from(size_id as u8)?;
        Ok(())
    }

    fn sync(&mut self) -> Result<(), Error> {
        self.connection
            .with_timeout(Duration::from_millis(100), |connection| {
                let data = &[
                    0x07, 0x07, 0x12, 0x20, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                ][..];

                connection.write_command(Command::Sync as u8, data, 0)?;

                for _ in 0..100 {
                    match connection.read_response()? {
                        Some(response) if response.return_op == Command::Sync as u8 => {
                            if response.status == 1 {
                                let _error = connection.flush();
                                return Err(Error::RomError(RomError::from(response.error)));
                            } else {
                                break;
                            }
                        }
                        _ => continue,
                    }
                }
                Ok(())
            })?;
            for _ in 0..7 {
                for _ in 0..100 {
                    match self.connection.read_response()? {
                        Some(_) => break,
                        _ => continue,
                    }
                }
            };
            Ok(())
    }

    fn start_connection(&mut self) -> Result<(), Error> {
        self.connection.reset_to_flash()?;
        for _ in 0..10 {
            self.connection.flush()?;
            if let Ok(_) = self.sync() {
                return Ok(());
            }
        }
        Err(Error::ConnectionFailed)
    }

    fn begin_command(
        &mut self,
        command: Command,
        size: u32,
        blocks: u32,
        block_size: u32,
        offset: u32,
    ) -> Result<(), Error> {
        let params = BeginParams {
            size,
            blocks,
            block_size,
            offset,
        };
        self.connection
            .command(command as u8, bytes_of(&params), 0)?;
        Ok(())
    }

    fn block_command(
        &mut self,
        command: Command,
        data: &[u8],
        padding: usize,
        padding_byte: u8,
        sequence: u32,
    ) -> Result<(), Error> {
        let params = BlockParams {
            size: (data.len() + padding) as u32,
            sequence,
            dummy1: 0,
            dummy2: 0,
        };

        let length = size_of::<BlockParams>() + data.len() + padding;

        let mut check = checksum(&data, CHECKSUM_INIT);

        for _ in 0..padding {
            check = checksum(&[padding_byte], check);
        }

        self.connection.command(
            command as u8,
            (length as u16, |encoder: &mut Encoder| {
                encoder.write(bytes_of(&params))?;
                encoder.write(&data)?;
                let padding = &[padding_byte; FLASH_WRITE_SIZE][0..padding];
                encoder.write(padding)?;
                Ok(())
            }),
            check as u32,
        )?;
        Ok(())
    }

    fn mem_finish(&mut self, entry: u32) -> Result<(), Error> {
        let params = EntryParams {
            no_entry: (entry == 0) as u32,
            entry,
        };
        self.connection
            .write_command(Command::MemEnd as u8, bytes_of(&params), 0)?;
        Ok(())
    }

    fn flash_finish(&mut self, reboot: bool) -> Result<(), Error> {
        self.connection
            .write_command(Command::FlashEnd as u8, &[(!reboot) as u8][..], 0)?;
        Ok(())
    }

    fn enable_flash(&mut self) -> Result<(), Error> {
        match self.chip {
            Chip::Esp8266 => {
                self.begin_command(Command::FlashBegin, 0, 0, FLASH_WRITE_SIZE as u32, 0)?;
            }
            Chip::Esp32 => {
                self.connection
                    .command(Command::SpiAttach as u8, &[0; 5][..], 0)?;
            }
        }
        Ok(())
    }

    fn spi_command(&mut self, command: u8, data: &[u8], read_bits: u32) -> Result<u32, Error> {
        assert!(read_bits < 32);
        assert!(data.len() < 64);

        let spi_registers = self.chip.spi_registers();

        let old_spi_usr = self.read_reg(spi_registers.usr())?;
        let old_spi_usr2 = self.read_reg(spi_registers.usr2())?;

        let mut flags = 1 << 31;
        if !data.is_empty() {
            flags |= 1 << 27;
        }
        if read_bits > 0 {
            flags |= 1 << 28;
        }

        self.write_reg(spi_registers.usr(), flags, None)?;
        self.write_reg(spi_registers.usr2(), 7 << 28 | command as u32, None)?;

        if let (Some(mosi_data_length), Some(miso_data_length)) =
            (spi_registers.mosi_length(), spi_registers.miso_length())
        {
            if data.len() > 0 {
                self.write_reg(mosi_data_length, data.len() as u32 * 8 - 1, None)?;
            }
            if read_bits > 0 {
                self.write_reg(miso_data_length, read_bits - 1, None)?;
            }
        } else {
            let mosi_mask = if data.is_empty() {
                0
            } else {
                data.len() as u32 * 8 - 1
            };
            let miso_mask = if read_bits == 0 { 0 } else { read_bits - 1 };
            self.write_reg(spi_registers.usr1(), miso_mask << 8 | mosi_mask << 17, None)?;
        }

        if data.is_empty() {
            self.write_reg(spi_registers.w0(), 0, None)?;
        } else {
            for (i, bytes) in data.chunks(4).enumerate() {
                let mut data_bytes = [0; 4];
                data_bytes[0..bytes.len()].copy_from_slice(bytes);
                let data = u32::from_le_bytes(data_bytes);
                self.write_reg(spi_registers.w0() + i as u32, data, None)?;
            }
        }

        self.write_reg(spi_registers.cmd(), 1 << 18, None)?;

        let mut i = 0;
        loop {
            sleep(Duration::from_millis(1));
            if self.read_reg(spi_registers.usr())? & (1 << 18) == 0 {
                break;
            }
            i += 1;
            if i > 10 {
                return Err(Error::Timeout);
            }
        }

        let result = self.read_reg(spi_registers.w0())?;
        self.write_reg(spi_registers.usr(), old_spi_usr, None)?;
        self.write_reg(spi_registers.usr2(), old_spi_usr2, None)?;

        Ok(result)
    }

    fn read_reg(&mut self, reg: u32) -> Result<u32, Error> {
        self.connection
            .command(Command::ReadReg as u8, &reg.to_le_bytes()[..], 0)
    }

    fn write_reg(&mut self, addr: u32, value: u32, mask: Option<u32>) -> Result<(), Error> {
        let params = WriteRegParams {
            addr,
            value,
            mask: mask.unwrap_or(0xFFFFFFFF),
            delay_us: 0,
        };
        self.connection
            .command(Command::WriteReg as u8, bytes_of(&params), 0)?;
        Ok(())
    }

    /// The chip type that the flasher is connected to
    pub fn chip(&self) -> Chip {
        self.chip
    }

    /// The flash size of the board that the flasher is connected to
    pub fn flash_size(&self) -> FlashSize {
        self.flash_size
    }

    /// Load an elf image to ram and execute it
    ///
    /// Note that this will not touch the flash on the device
    pub fn load_elf_to_ram(&mut self, elf_data: &[u8]) -> Result<(), Error> {
        let image = FirmwareImage::from_data(elf_data).map_err(|_| Error::InvalidElf)?;

        if image.rom_segments(self.chip).next().is_some() {
            return Err(Error::ElfNotRamLoadable);
        }

        for segment in image.ram_segments(self.chip) {
            let padding = 4 - segment.data.len() % 4;
            let block_count =
                (segment.data.len() + padding + MAX_RAM_BLOCK_SIZE - 1) / MAX_RAM_BLOCK_SIZE;
            self.begin_command(
                Command::MemBegin,
                segment.data.len() as u32,
                block_count as u32,
                MAX_RAM_BLOCK_SIZE as u32,
                segment.addr,
            )?;

            for (i, block) in segment.data.chunks(MAX_RAM_BLOCK_SIZE).enumerate() {
                let block_padding = if i == block_count - 1 { padding } else { 0 };
                self.block_command(Command::MemData, &block, block_padding, 0, i as u32)?;
            }
        }

        self.mem_finish(image.entry())?;

        Ok(())
    }

    /// Load an elf image to flash and execute it
    pub fn load_elf_to_flash(&mut self, elf_data: &[u8]) -> Result<(), Error> {
        self.enable_flash()?;
        let mut image = FirmwareImage::from_data(elf_data).map_err(|_| Error::InvalidElf)?;
        image.flash_size = self.flash_size();

        println!("Flashing");
        for segment in self.chip.get_flash_segments(&image) {
            let segment = segment?;
            let addr = segment.addr;
            let block_count = (segment.data.len() + FLASH_WRITE_SIZE - 1) / FLASH_WRITE_SIZE;
            print!("   0x{:05x?} ", segment.addr);
            let _ = stdout().flush();

            let erase_size = match self.chip {
                Chip::Esp32 => segment.data.len() as u32,
                Chip::Esp8266 => get_erase_size(addr as usize, segment.data.len()) as u32,
            };

            self.begin_command(
                Command::FlashBegin,
                erase_size,
                block_count as u32,
                FLASH_WRITE_SIZE as u32,
                addr,
            )?;

            for (i, block) in segment.data.chunks(FLASH_WRITE_SIZE).enumerate() {
                print!(".");
                let _ = stdout().flush();
                let block_padding = FLASH_WRITE_SIZE - block.len();
                self.block_command(Command::FlashData, &block, block_padding, 0xff, i as u32)?;
            }
            println!();
        }

        self.flash_finish(false)?;
        println!("Firmware flashed successfully");

        print!("Resetting device - ");
        match self.connection.reset() {
            Ok(_) => {
                println!("Success");
                return Ok(());
            },
            Err(e) => {
                println!("Error {}", e);
                return Err(e);
            }
        }
    }

    pub fn change_baud(&mut self, speed: BaudRate) -> Result<(), Error> {
        let new_speed = (speed.speed() as u32).to_le_bytes();
        let old_speed = 0u32.to_le_bytes();
        self.connection.command(
            Command::ChangeBaud as u8,
            &[new_speed, old_speed].concat()[..],
            0,
        )?;
        self.connection.set_baud(speed)?;
        std::thread::sleep(Duration::from_secs_f32(0.05));
        self.connection.flush()?;
        Ok(())
    }
}

fn get_erase_size(offset: usize, size: usize) -> usize {
    let sector_count = (size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
    let start_sector = offset / FLASH_SECTOR_SIZE;

    let head_sectors = usize::min(
        FLASH_SECTORS_PER_BLOCK - (start_sector % FLASH_SECTORS_PER_BLOCK),
        sector_count,
    );

    if sector_count < 2 * head_sectors {
        (sector_count + 1) / 2 * FLASH_SECTOR_SIZE
    } else {
        (sector_count - head_sectors) * FLASH_SECTOR_SIZE
    }
}

const CHECKSUM_INIT: u8 = 0xEF;

pub fn checksum(data: &[u8], mut checksum: u8) -> u8 {
    for byte in data.as_ref() {
        checksum ^= *byte;
    }

    checksum
}
