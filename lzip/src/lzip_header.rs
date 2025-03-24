use crate::lzip_index::MIN_DICTIONARY_SIZE;

const LZIP_MAGIC: [u8; 4] = [0x4C, 0x5A, 0x49, 0x50]; // "LZIP"

#[derive(Debug, Clone, Default)]
pub struct LzipHeader {
    pub data: [u8; Self::SIZE], // 0-3 magic bytes
                                // 4 version
                                // 5 coded dictionary size
}

impl LzipHeader {
    pub const SIZE: usize = 6;

    pub fn check_magic(&self) -> bool {
        self.data[..4] == LZIP_MAGIC
    }

    pub fn version(&self) -> u8 {
        self.data[4]
    }
    pub fn check_version(&self) -> bool {
        self.data[4] == 1
    }

    pub fn dictionary_size(&self) -> u32 {
        let mut sz = 1 << (self.data[5] & 0x1F);
        if sz > MIN_DICTIONARY_SIZE {
            sz -= (sz / 16) * ((self.data[5] as u32 >> 5) & 7);
        }
        sz
    }
}
