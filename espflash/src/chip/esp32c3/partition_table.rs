use std::io::Write;

use md5::{Context, Digest};

const MAX_PARTITION_LENGTH: usize = 0xC00;
const PARTITION_TABLE_SIZE: usize = 0x1000;

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Type {
    App = 0x00,
    Data = 0x01,
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
#[allow(dead_code)]
pub enum AppType {
    Factory = 0x00,
    Ota0 = 0x10,
    Ota1 = 0x11,
    Ota2 = 0x12,
    Ota3 = 0x13,
    Ota4 = 0x14,
    Ota5 = 0x15,
    Ota6 = 0x16,
    Ota7 = 0x17,
    Ota8 = 0x18,
    Ota9 = 0x19,
    Ota10 = 0x1a,
    Ota11 = 0x1b,
    Ota12 = 0x1c,
    Ota13 = 0x1d,
    Ota14 = 0x1e,
    Ota15 = 0x1f,
    Test = 0x20,
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
#[allow(dead_code)]
pub enum DataType {
    Ota = 0x00,
    Phy = 0x01,
    Nvs = 0x02,
    CoreDump = 0x03,
    NvsKeys = 0x04,
    EFuse = 0x05,
    EspHttpd = 0x80,
    Fat = 0x81,
    Spiffs = 0x82,
}

#[allow(dead_code)]
pub enum SubType {
    App(AppType),
    Data(DataType),
}

impl SubType {
    fn as_u8(&self) -> u8 {
        match self {
            SubType::App(ty) => *ty as u8,
            SubType::Data(ty) => *ty as u8,
        }
    }
}

pub struct PartitionTable {
    partitions: Vec<Partition>,
}

impl PartitionTable {
    /// Create a basic partition table with a single app entry
    pub fn basic(app_offset: u32, app_size: u32) -> Self {
        PartitionTable {
            partitions: vec![Partition::new(
                String::from("factory"),
                Type::App,
                SubType::App(AppType::Factory),
                app_offset,
                app_size,
                0,
            )],
        }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let table = PartitionTable::basic(0x10000, 0x3f0000);

        let mut result = Vec::with_capacity(PARTITION_TABLE_SIZE);
        table.save(&mut result).unwrap();
        result
    }

    pub fn save<W: Write>(&self, writer: &mut W) -> std::io::Result<()> {
        let mut hasher = HashWriter::new(writer);
        for partition in &self.partitions {
            partition.save(&mut hasher)?;
        }

        let (writer, hash) = hasher.compute();

        writer.write_all(&[
            0xEB, 0xEB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF,
        ])?;
        writer.write_all(&hash.0)?;

        let written = self.partitions.len() * PARTITION_SIZE + 32;
        for _ in 0..(MAX_PARTITION_LENGTH - written) {
            writer.write_all(&[0xFF])?;
        }

        Ok(())
    }
}

const PARTITION_SIZE: usize = 32;

struct Partition {
    name: String,
    ty: Type,
    sub_type: SubType,
    offset: u32,
    size: u32,
    flags: u32,
}

impl Partition {
    pub fn new(
        name: String,
        ty: Type,
        sub_type: SubType,
        offset: u32,
        size: u32,
        flags: u32,
    ) -> Self {
        Partition {
            name,
            ty,
            sub_type,
            offset,
            size,
            flags,
        }
    }

    pub fn save<W: Write>(&self, writer: &mut W) -> std::io::Result<()> {
        writer.write_all(&[0xAA, 0x50])?;
        writer.write_all(&[self.ty as u8, self.sub_type.as_u8()])?;
        writer.write_all(&self.offset.to_le_bytes())?;
        writer.write_all(&self.size.to_le_bytes())?;

        let mut name_bytes = [0u8; 16];
        for (source, dest) in self.name.bytes().take(16).zip(name_bytes.iter_mut()) {
            *dest = source;
        }
        writer.write_all(&name_bytes)?;
        writer.write_all(&self.flags.to_le_bytes())?;

        Ok(())
    }
}

struct HashWriter<W: Write> {
    inner: W,
    hasher: Context,
}

impl<W: Write> Write for HashWriter<W> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.hasher.write_all(buf)?;
        self.inner.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

impl<W: Write> HashWriter<W> {
    pub fn new(inner: W) -> Self {
        HashWriter {
            inner,
            hasher: Context::new(),
        }
    }

    pub fn compute(self) -> (W, Digest) {
        (self.inner, self.hasher.compute())
    }
}

#[test]
fn test_basic() {
    use std::fs::read;

    let expected = read("./tests/data/partitions.bin").unwrap();
    let table = PartitionTable::basic(0x10000, 0x3f0000);

    let result = table.to_bytes();

    assert_eq!(expected.len(), result.len());
    assert_eq!(expected, result.as_slice());
}
