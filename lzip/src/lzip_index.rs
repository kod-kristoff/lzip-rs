use std::io::{self, SeekFrom};

use crate::{
    constants::MIN_MEMBER_SIZE, errors::LzipError, lzip_header::LzipHeader,
    lzip_trailer::LzipTrailer,
};

pub struct LzipIndex {}

#[derive(Debug, Copy,Clone)]
/// command-line options
pub struct LzipIndexOptions {
    pub ignore_empty: bool,
    pub ignore_marking: bool,
    pub ignore_trailing: bool,
    pub loose_trailing: bool,
}

impl Default for LzipIndexOptions {
    fn default() -> Self {
        Self { ignore_empty: true, ignore_marking: true, ignore_trailing: true, loose_trailing: false }
    }
}

impl LzipIndex {
    pub fn from_reader<R>(input: &mut R) -> Result<Self, LzipError>
    where
        R: io::BufRead + io::Seek,
    {
        let insize = input
            .seek(SeekFrom::End(0))
            .map_err(LzipError::InputNotSeekable)?;
        let mut members = Vec::new();

        input.rewind().unwrap();
        let header =
            LzipHeader::from_reader(input).map_err(|error| LzipError::Header { pos: 0, error })?;

        let mut pos = insize;
        while pos >= MIN_MEMBER_SIZE {
            let read_pos = input
                .seek(SeekFrom::Start(pos - LzipTrailer::SIZE))
                .unwrap();
            let trailer = LzipTrailer::from_reader(input).map_err(|error| LzipError::Trailer {
                pos: read_pos,
                error,
            })?;
            if trailer.member_size() > pos {
                // bad trailer
                if members.is_empty() {
                    if skip_trailing_data(input, pos,)
                }
            }
        }
        todo!()
    }
}

fn skip_trailing_data()
