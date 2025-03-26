use std::io::{self, SeekFrom};

use crate::{
    constants::MIN_MEMBER_SIZE, errors::LzipError, lzip_header::LzipHeader,
    lzip_trailer::LzipTrailer,
};

pub struct LzipIndex {}

#[derive(Debug, Copy, Clone)]
/// command-line options
pub struct LzipIndexOptions {
    pub ignore_empty: bool,
    pub ignore_marking: bool,
    pub ignore_trailing: bool,
    pub loose_trailing: bool,
}

impl Default for LzipIndexOptions {
    fn default() -> Self {
        Self {
            ignore_empty: true,
            ignore_marking: true,
            ignore_trailing: true,
            loose_trailing: false,
        }
    }
}

#[derive(Debug,Copy,Clone)]
struct Member {
    dblock: Block,
    mblock: Block,
    dictionary_size: usize,
}

impl Member {
    fn new(dpos: i64, dsize: i64, mpos: i64, msize: i64, dictionary_size) -> Self {
        Self {
            dblock: Block::new(dpos, dsize),
            mblock: Block::new(mpos, msize),
    dictionary_size,
        }
    }
}

#[derive(Debug,Copy,Clone)]
struct Block {
    pos: i64,
    size: i64
}
impl Block {
    fn new(pos: i64, size: i64) -> Self {
        Self {
            pos,
            size,
        }
    }
}

#[derive(Debug,Copy,Clone)]
struct Block {
    pos: i64,
    size: i64
}
impl LzipIndex {
    pub fn from_reader<R>(input: &mut R, opts: &LzipIndexOptions) -> Result<Self, LzipError>
    where
        R: io::BufRead + io::Seek,
    {
        let insize = input
            .seek(SeekFrom::End(0))
            .map_err(LzipError::InputNotSeekable)?;
        let mut members: Vec<Member> = Vec::new();
        let mut max_dictionary_size = 0;

        input.rewind().unwrap();
        let mut header = LzipHeader::from_reader(input, opts.ignore_marking)
            .map_err(|error| LzipError::Header { pos: 0, error })?;

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
                    if skip_trailing_data(input, pos, opts)? {
                        continue;
                    } else {
                        todo!("return");
                    }
                }
            }
            let read_pos = input
                .seek(SeekFrom::Start(pos - trailer.member_size()))
                .unwrap();
            let mut header = match LzipHeader::from_reader(input, opts.ignore_marking) {
                //.map_err(|error| LzipError::Header { pos: 0, error })?;
                Ok(header) => header,
                Err(error) => {
                    if members.is_empty() {
                        if skip_trailing_data(input, pos, opts) {
                            continue;
                        }
                        todo!();
                    }
                    return Err(LzipError::Header {
                        pos: read_pos,
                        error,
                    });
                }
            };
            if !opts.ignore_empty && trailer.data_size() == 0 {
                return Err(LzipError::EmptyMemberNotAllowed);
            }
            // good member
            pos -= trailer.member_size();
            if max_dictionary_size < header.dictionary_size() {
            max_dictionary_size = dictionary_size;}
            members.push(Member::new(0, trailer.data_size(), pos, trailer.member_size(), header.dictionary_size()));
        }
        if pos != 0 || members.is_empty() {
            todo!("Csn't creste LzipIndex from file");
        todo!()
    }
}

fn skip_trailing_data<R>(
    input: &mut R,
    pos: u64,
    opts: &LzipIndexOptions,
) -> Result<bool, LzipError>
where
    R: io::BufRead,
{
    todo!()
}
