use std::io::{self, Read, Seek, SeekFrom};

use crate::{
    ClOptions, LzipError, LzipHeader, LzipTrailer, LzipTrailerRef,
    lzip_trailer::trailer_member_size,
};

pub const MIN_MEMBER_SIZE: u64 = 36;
pub const MIN_DICTIONARY_BITS: u32 = 12;
pub const MIN_DICTIONARY_SIZE: u32 = 1 << MIN_DICTIONARY_BITS;
pub const MAX_DICTIONARY_BITS: u32 = 29;
pub const MAX_DICTIONARY_SIZE: u32 = 1 << MAX_DICTIONARY_BITS;

#[derive(Debug, Clone)]
pub struct LzipIndex {
    member_vector: Vec<Member>,
}

#[derive(Debug, Clone)]
struct Member {
    /// data block
    dblock: Block,
    /// member block
    mblock: Block,
    dictionary_size: u32,
}

/// pos >= 0, size >= 0, pos + size <= INT64_MAX
#[derive(Debug, Clone)]
struct Block {
    pos: u64,
    size: u64,
}

impl LzipIndex {
    pub fn from_reader_with_options<R>(mut infd: R, cl_opts: &ClOptions) -> Result<Self, LzipError>
    where
        R: Read + Seek,
    {
        let insize = infd.seek(SeekFrom::End(0))?;
        let dictionary_size = 0;

        if insize < MIN_MEMBER_SIZE {
            return Err(LzipError::InputFileTooShort { size: insize });
        }
        if insize > i64::MAX as u64 {
            return Err(LzipError::InputFileTooLong { size: insize });
        }

        let mut member_vector = Vec::new();
        let mut header = LzipHeader::default();
        read_header(&mut infd, &mut header, 0, cl_opts.ignore_marking)?;
        check_header(&header)?;

        let mut pos = insize;
        while pos >= MIN_MEMBER_SIZE {
            let mut trailer = LzipTrailer::default();
            if seek_read(
                &mut infd,
                &mut trailer.data,
                LzipTrailer::SIZE,
                pos - LzipTrailer::SIZE as u64,
            )? != LzipTrailer::SIZE
            {
                return Err(LzipError::ErrorReadingMemberTrailer);
            }
            let member_size = trailer.member_size();
            if member_size > pos || !trailer.check_consistency() {
                if member_vector.is_empty() {
                    if Self::skip_trailing_data(&mut infd, &mut pos, cl_opts) {}
                }
            }
        }
        todo!()
    }
    // If successful, push last member and set pos to member header
    fn skip_trailing_data<R>(
        fd: &mut R,
        pos: &mut u64,
        cl_opts: &ClOptions,
    ) -> Result<bool, LzipError>
    where
        R: Read + Seek,
    {
        if *pos < MIN_MEMBER_SIZE {
            return Ok(false);
        }
        const BLOCK_SIZE: usize = 16384;
        const BUFFER_SIZE: usize = BLOCK_SIZE + LzipTrailer::SIZE - 1 + LzipHeader::SIZE;
        let mut buffer = [0u8; BUFFER_SIZE];
        let mut bsize = *pos as usize % BLOCK_SIZE; // total bytes in buffer
        if bsize <= BUFFER_SIZE - BLOCK_SIZE {
            bsize += BLOCK_SIZE;
        }
        let search_size = bsize; // bytes to search for trailer
        let rd_size = bsize; // bytes to read from file
        let ipos = *pos - rd_size as u64;

        loop {
            if seek_read(fd, &mut buffer, rd_size, ipos)? != rd_size {
                eprintln!("Error seeking member trailer:");
                return Ok(false);
            }
            let max_msb: u8 = ((ipos + search_size as u64) >> 56) as u8;
            let mut i = search_size;
            while i >= LzipTrailer::SIZE {
                if buffer[i - 1] <= max_msb {
                    // most significant byte of member_size
                    let trailer = LzipTrailerRef::new(&buffer[i - LzipTrailer::SIZE..]);
                    let member_size = trailer.member_size();
                    if member_size == 0 {
                        // skip trailing zeros
                        while i > LzipTrailer::SIZE && buffer[i - 9] == 0 {
                            i -= 1;
                        }
                        continue;
                    }
                    if member_size > ipos + i as u64 || !trailer.check_consistency() {
                        continue;
                    }
                }
                i -= 1;
            }
        }
    }
}

fn check_header(header: &LzipHeader) -> Result<(), LzipError> {
    if !header.check_magic() {
        return Err(LzipError::BadMagic);
    }
    if !header.check_version() {
        return Err(LzipError::UnsupportedVersion {
            version: header.version(),
        });
    }
    if !is_valid_ds(header.dictionary_size()) {
        return Err(LzipError::InvalidDictionarySize);
    }
    Ok(())
}

fn is_valid_ds(dictionary_size: u32) -> bool {
    dictionary_size >= MIN_DICTIONARY_SIZE && dictionary_size <= MAX_DICTIONARY_SIZE
}
fn read_header<R>(
    fd: &mut R,
    header: &mut LzipHeader,
    pos: u64,
    ignore_marking: bool,
) -> Result<(), LzipError>
where
    R: Read + Seek,
{
    if seek_read(fd, &mut header.data, LzipHeader::SIZE, pos)? != LzipHeader::SIZE {
        return Err(LzipError::ErrorReadingMemberHeader);
    }
    let mut byte = [0; 1];
    if !ignore_marking && readblock(fd, &mut byte[..], 1)? == 1 && byte[0] != 0 {
        return Err(LzipError::MarkingDataNotAllowed);
    }
    Ok(())
}
fn seek_read<R>(fd: &mut R, buf: &mut [u8], size: usize, pos: u64) -> io::Result<usize>
where
    R: Read + Seek,
{
    if fd.seek(SeekFrom::Start(pos))? == pos {
        readblock(fd, buf, size)
    } else {
        Ok(0)
    }
}
pub fn readblock<R: Read>(fd: &mut R, buf: &mut [u8], size: usize) -> io::Result<usize> {
    let mut sz = 0;
    while sz < size {
        let n = fd.read(buf)?;
        if n > 0 {
            sz += n;
        } else if n == 0 {
            break;
        }
    }
    Ok(sz)
}
