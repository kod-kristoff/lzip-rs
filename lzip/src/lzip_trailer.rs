use std::io;

use byteorder::{ByteOrder, LittleEndian};

use crate::{constants::MIN_MEMBER_SIZE, errors::LzipTrailerError};

#[derive(Debug)]
pub struct LzipTrailer {
    /// CRC32 of the uncompressed data
    data_crc: u32,
    /// size of the uncompressed data
    data_size: u64,
    /// member size including header and trailer
    member_size: u64,
}

impl LzipTrailer {
    pub const SIZE: u64 = 20;

    pub fn from_reader<R>(input: &mut R) -> Result<Self, LzipTrailerError>
    where
        R: io::BufRead,
    {
        let mut buffer = [0u8; Self::SIZE as usize];
        input
            .read_exact(&mut buffer)
            .map_err(LzipTrailerError::ErrorReading)?;

        let data_crc = LittleEndian::read_u32(&buffer[0..3]);
        let data_size = LittleEndian::read_u64(&buffer[4..11]);
        let member_size = LittleEndian::read_u64(&buffer[12..19]);
        if (data_crc == 0) != (data_size == 0) {
            return Err(LzipTrailerError::CrcAndDataSizeMismatch {
                crc: data_crc,
                size: data_size,
            });
        }
        if member_size < MIN_MEMBER_SIZE {
            return Err(LzipTrailerError::TooSmallMemberSize(member_size));
        }
        let mlimit = (9 * data_size + 7) / 8 + MIN_MEMBER_SIZE;
        if mlimit > data_size && member_size > mlimit {
            return Err(LzipTrailerError::TooSmallDataSizeAndTooLargeMemberSize {
                dsize: data_size,
                msize: member_size,
            });
        }
        let dlimit = 7090 * (member_size - 26) - 1;
        if dlimit > member_size && data_size > dlimit {
            return Err(LzipTrailerError::TooLargeDataSizeAndTooSmallMemberSize {
                dsize: data_size,
                msize: member_size,
            });
        }
        Ok(Self {
            data_crc,
            data_size,
            member_size,
        })
    }

    pub fn member_size(&self) -> u64 {
        self.member_size
    }
}
