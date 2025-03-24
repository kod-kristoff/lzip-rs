use crate::lzip_index::MIN_MEMBER_SIZE;

#[derive(Debug, Clone, Default)]
pub struct LzipTrailer {
    pub data: [u8; Self::SIZE], //  0-3  CRC32 of the uncompressed data
                                //  4-11 size of the uncompressed data
                                // 12-19 member size including header and trailer
}

impl LzipTrailer {
    pub const SIZE: usize = 20;

    pub fn data_crc(&self) -> u32 {
        let mut result = 0;
        let mut i = 3;
        while i >= 0 {
            result <<= 8;
            result += self.data[i] as u32;
            i -= 1;
        }
        result
    }
    pub fn data_size(&self) -> u64 {
        let mut result = 0;
        let mut i = 11;
        while i >= 4 {
            result <<= 8;
            result += self.data[i] as u64;
            i -= 1;
        }
        result
    }
    pub fn member_size(&self) -> u64 {
        let mut result = 0;
        let mut i = 19;
        while i >= 12 {
            result <<= 8;
            result += self.data[i] as u64;
            i -= 1;
        }
        result
    }

    /// check internal consistency
    pub fn check_consistency(&self) -> bool {
        let crc = self.data_crc();
        let dsize = self.data_size();
        if (crc == 0) != (dsize == 0) {
            return false;
        }
        let msize = self.member_size();
        if msize < MIN_MEMBER_SIZE {
            return false;
        }
        let mlimit = (9 * dsize + 7) / 8 + MIN_MEMBER_SIZE;
        if mlimit > dsize && msize > mlimit {
            return false;
        }
        let dlimit = 7090 * (msize - 26) - 1;
        if dlimit > msize && dsize > dlimit {
            return false;
        }
        true
    }
}
pub(crate) fn trailer_data_crc(data: &[u8]) -> u32 {
    let mut result = 0;
    let mut i = 3;
    while i >= 0 {
        result <<= 8;
        result += data[i] as u32;
        i -= 1;
    }
    result
}
pub(crate) fn trailer_data_size(data: &[u8]) -> u64 {
    let mut result = 0;
    let mut i = 11;
    while i >= 4 {
        result <<= 8;
        result += data[i] as u64;
        i -= 1;
    }
    result
}
pub(crate) fn trailer_member_size(data: &[u8]) -> u64 {
    debug_assert!(data.len() > 19);
    let mut result = 0;
    let mut i = 19;
    while i >= 12 {
        result <<= 8;
        result += data[i] as u64;
        i -= 1;
    }
    result
}
pub(crate) fn trailer_check_consistency(data: &[u8]) -> bool {
    let crc = trailer_data_crc(data);
    let dsize = trailer_data_size(data);
    if (crc == 0) != (dsize == 0) {
        return false;
    }
    let msize = trailer_member_size(data);
    if msize < MIN_MEMBER_SIZE {
        return false;
    }
    let mlimit = (9 * dsize + 7) / 8 + MIN_MEMBER_SIZE;
    if mlimit > dsize && msize > mlimit {
        return false;
    }
    let dlimit = 7090 * (msize - 26) - 1;
    if dlimit > msize && dsize > dlimit {
        return false;
    }
    true
}

#[derive(Debug, Clone)]
pub(crate) struct LzipTrailerRef<'a> {
    data: &'a [u8],
}

impl<'a> LzipTrailerRef<'a> {
    pub(crate) fn new(data: &'a [u8]) -> Self {
        debug_assert!(data.len() >= LzipTrailer::SIZE);
        Self { data }
    }

    pub(crate) fn member_size(&self) -> u64 {
        trailer_member_size(&self.data)
    }

    pub(crate) fn check_consistency(&self) -> bool {
        trailer_check_consistency(&self.data)
    }
}
