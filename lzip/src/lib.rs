use std::io::{self};

pub use lzip_index::{LzipIndex, LzipIndexOptions};

use crate::errors::LzipError;

pub mod constants;
mod errors;
mod lzip_header;
mod lzip_index;
mod lzip_trailer;

pub fn list_files<R: io::BufRead + io::Seek>(
    input: &mut R,
    opts: &LzipIndexOptions,
) -> Result<(), LzipError> {
    let index = LzipIndex::from_reader(input, opts)?;
    let udata_size = index.udata_size();
    let cdata_size = index.cdata_size();
    todo!()
}
