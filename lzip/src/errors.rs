use std::fmt;
use std::io;

#[derive(Debug)]
pub enum LzipError {
    Header { pos: u64, error: LzipHeaderError },
    InputNotSeekable(io::Error),
    Trailer { pos: u64, error: LzipTrailerError },
    EmptyMemberNotAllowed,
    DataTooLong,
}

#[derive(Debug)]
pub enum LzipHeaderError {
    BadFormat,
    HeaderTooShort(io::Error),
    UnsupportedVersion(u8),
    TooSmallEncodedDictionarySize(u8),
    TooLargeEncodedDictionarySize(u8),
    InvalidMarkingData,
}

#[derive(Debug)]
pub enum LzipTrailerError {
    ErrorReading(io::Error),
    CrcAndDataSizeMismatch { crc: u32, size: u64 },
    TooSmallMemberSize(u64),
    TooSmallDataSizeAndTooLargeMemberSize { dsize: u64, msize: u64 },
    TooLargeDataSizeAndTooSmallMemberSize { dsize: u64, msize: u64 },
}
