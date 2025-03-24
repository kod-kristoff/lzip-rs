use std::io::{self};

use lzip_index::LzipIndex;

use crate::errors::LzipError;

pub mod constants;
mod errors;
mod lzip_header;
mod lzip_index;
mod lzip_trailer;

pub fn list_files<R: io::BufRead + io::Seek>(input: &mut R) -> Result<(), LzipError> {
    let index = LzipIndex::from_reader(input)?;
    todo!()
}
