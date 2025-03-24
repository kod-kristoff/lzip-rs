use core::fmt;
use std::io;

#[derive(Debug)]
pub enum LzipError {
    BadMagic,
    UnsupportedVersion { version: u8 },
    InvalidDictionarySize,
    IoError(io::Error),
    InputFileTooShort { size: u64 },
    InputFileTooLong { size: u64 },
    ErrorReadingMemberHeader,
    MarkingDataNotAllowed,
    ErrorReadingMemberTrailer,
}

impl fmt::Display for LzipError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadMagic => f.write_str("Bad magic number (file not in Lzip format)."),
            Self::UnsupportedVersion { version } => f.write_fmt(format_args!(
                "Version {} member format not supported.",
                version
            )),
            Self::InvalidDictionarySize => f.write_str("Invalid dictionary size in member header.")
            Self::IoError(_err) => f.write_str("IO error"),
            Self::InputFileTooShort { size: _ } => f.write_str("Input file is too short."),
            Self::InputFileTooLong { size: _ } => {
                f.write_str("Input file is too long (2^63 bytes or more).")
            }
            Self::ErrorReadingMemberHeader => f.write_str("Error reading member header: "),
            Self::MarkingDataNotAllowed => f.write_str("Marking data not allowed."),
        }
    }
}

impl std::error::Error for LzipError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::IoError(err) => Some(err),
            _ => None,
        }
    }
}

impl From<io::Error> for LzipError {
    fn from(value: io::Error) -> Self {
        LzipError::IoError(value)
    }
}
