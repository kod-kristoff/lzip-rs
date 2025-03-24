use std::io::{self, SeekFrom};

use crate::errors::LzipError;

mod errors;

pub fn list_files<R: io::BufRead + io::Seek>(input: &mut R) -> Result<(), LzipError>{
    let index = LzipIndex::from_reader(input)?;
    todo!()
}

pub struct LzipIndex {}

impl LzipIndex {
    pub fn from_reader<R>(input: &mut R) -> Result<Self, LzipError>
    where
        R: io::BufRead + io::Seek
    {
        let insize = input.seek(SeekFrom::End(0)).unwrap();

        input.rewind().unwrap();
        let header = LzipHeader::from_reader(input)?;

        let mut pos = insize;
        while pos 
        todo!()
    }
}
pub struct LzipHeader {
    version: u8,
    dictionary_size: u32,
}

impl LzipHeader {
    const LZIP_MAGIC: [u8;4] = [0x4C, 0x5A, 0x49, 0x50 ];   // "LZIP"
    pub fn from_reader<R>(input: &mut R) -> Result<Self, LzipError> 
    where
        R: io::BufRead 
    {
        let mut buffer =[0u8;6];
        input.read_exact(&mut buffer).map_err(LzipError::HeaderTooShort)?;

        if buffer[0..4] != Self::LZIP_MAGIC {
            return Err(LzipError::BadFormat);
        }
        let version = buffer[4];
        if version != 1 {
            return Err(LzipError::UnsupportedVersion(version));
        }
        let encoded_dict_size = buffer[5];
        let dictionary_size = decode_dict_size(encoded_dict_size)?;
        Ok(Self {version, dictionary_size})
    }
}

const MIN_DICTIONARY_BITS: u32 = 12;
const MIN_DICTIONARY_SIZE: u32 = 1 << MIN_DICTIONARY_BITS;
const MAX_DICTIONARY_BITS: u32 = 29;
const MAX_DICTIONARY_SIZE: u32 = 1 << MAX_DICTIONARY_BITS;

fn decode_dict_size(encoded_size: u8) -> Result<u32,LzipError> {
    let mut size = 1<< (encoded_size & 0x1F);
    if size < MIN_DICTIONARY_SIZE {
        return Err(LzipError::TooSmallEncodedDictionarySize(encoded_size));
    }
    if size > MIN_DICTIONARY_SIZE {
        size -= ( size / 16 ) * ((encoded_size as u32 >> 5) & 7);
    }
    if size > MAX_DICTIONARY_SIZE {
        return Err(LzipError::TooLargeEncodedDictionarySize(encoded_size));
    }
    Ok(size)
}
