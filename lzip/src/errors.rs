use std::io;
use std::fmt;

#[derive(Debug)]
pub enum LzipError {
    BadFormat,
    HeaderTooShort(io::Error),
    UnsupportedVersion(u8),
    TooSmallEncodedDictionarySize(u8),
    TooLargeEncodedDictionarySize(u8),
}


