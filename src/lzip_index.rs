// /* Lzip - LZMA lossless data compressor
//    Copyright (C) 2008-2024 Antonio Diaz Diaz.

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 2 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */
// #ifndef INT64_MAX
// #define INT64_MAX 0x7FFFFFFFFFFFFFFFLL
// #endif

// class Block
//   {
//   long long pos_, size_;	// pos >= 0, size >= 0, pos + size <= INT64_MAX

// public:
//   Block( const long long p, const long long s ) : pos_( p ), size_( s ) {}

//   long long pos() const { return pos_; }
//   long long size() const { return size_; }
//   long long end() const { return pos_ + size_; }

//   void pos( const long long p ) { pos_ = p; }
//   void size( const long long s ) { size_ = s; }
//   };

use std::io;

use crate::lzip::ClOptions;

pub struct LzipIndex
  {
//   struct Member
//     {
//     Block dblock, mblock;		// data block, member block
//     unsigned dictionary_size;

//     Member( const long long dpos, const long long dsize,
//             const long long mpos, const long long msize,
//             const unsigned dict_size )
//       : dblock( dpos, dsize ), mblock( mpos, msize ),
//         dictionary_size( dict_size ) {}
//     };

//   std::vector< Member > member_vector;
//   std::string error_;
//   const long long insize;
//   int retval_;
//   unsigned dictionary_size_;	// largest dictionary size in the file
}
//   bool check_header( const Lzip_header & header );
//   void set_errno_error( const char * const msg );
//   void set_num_error( const char * const msg, unsigned long long num );
//   bool read_header( const int fd, Lzip_header & header, const long long pos,
//                     const bool ignore_marking );
//   bool skip_trailing_data( const int fd, unsigned long long & pos,
//                            const Cl_options & cl_opts );

// public:
//   Lzip_index( const int infd, const Cl_options & cl_opts );

//   long members() const { return member_vector.size(); }
//   const std::string & error() const { return error_; }
//   int retval() const { return retval_; }
//   unsigned dictionary_size() const { return dictionary_size_; }

//   long long udata_size() const
//     { if( member_vector.empty() ) return 0;
//       return member_vector.back().dblock.end(); }

//   long long cdata_size() const
//     { if( member_vector.empty() ) return 0;
//       return member_vector.back().mblock.end(); }

//   // total size including trailing data (if any)
//   long long file_size() const
//     { if( insize >= 0 ) return insize; else return 0; }

//   const Block & dblock( const long i ) const
//     { return member_vector[i].dblock; }
//   const Block & mblock( const long i ) const
//     { return member_vector[i].mblock; }
//   unsigned dictionary_size( const long i ) const
//     { return member_vector[i].dictionary_size; }
//   };
// /* Lzip - LZMA lossless data compressor
//    Copyright (C) 2008-2024 Antonio Diaz Diaz.

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 2 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */
// #define _FILE_OFFSET_BITS 64

// #include <algorithm>
// #include <cerrno>
// #include <cstdio>
// #include <cstring>
// #include <string>
// #include <vector>
// #include <stdint.h>
// #include <unistd.h>

// #include "lzip.h"
// #include "lzip_index.h"

// namespace {

// int seek_read( const int fd, uint8_t * const buf, const int size,
//                const long long pos )
//   {
//   if( lseek( fd, pos, SEEK_SET ) == pos )
//     return readblock( fd, buf, size );
//   return 0;
//   }

// } // end namespace

// bool Lzip_index::check_header( const Lzip_header & header )
//   {
//   if( !header.check_magic() )
//     { error_ = bad_magic_msg; retval_ = 2; return false; }
//   if( !header.check_version() )
//     { error_ = bad_version( header.version() ); retval_ = 2; return false; }
//   if( !isvalid_ds( header.dictionary_size() ) )
//     { error_ = bad_dict_msg; retval_ = 2; return false; }
//   return true;
//   }

// void Lzip_index::set_errno_error( const char * const msg )
//   {
//   error_ = msg; error_ += std::strerror( errno );
//   retval_ = 1;
//   }

// void Lzip_index::set_num_error( const char * const msg, unsigned long long num )
//   {
//   char buf[80];
//   snprintf( buf, sizeof buf, "%s%llu", msg, num );
//   error_ = buf;
//   retval_ = 2;
//   }

// bool Lzip_index::read_header( const int fd, Lzip_header & header,
//                               const long long pos, const bool ignore_marking )
//   {
//   if( seek_read( fd, header.data, header.size, pos ) != header.size )
//     { set_errno_error( "Error reading member header: " ); return false; }
//   uint8_t byte;
//   if( !ignore_marking && readblock( fd, &byte, 1 ) == 1 && byte != 0 )
//     { error_ = marking_msg; retval_ = 2; return false; }
//   return true;
//   }

// // If successful, push last member and set pos to member header.
// bool Lzip_index::skip_trailing_data( const int fd, unsigned long long & pos,
//                                      const Cl_options & cl_opts )
//   {
//   if( pos < min_member_size ) return false;
//   enum { block_size = 16384,
//          buffer_size = block_size + Lzip_trailer::size - 1 + Lzip_header::size };
//   uint8_t buffer[buffer_size];
//   int bsize = pos % block_size;			// total bytes in buffer
//   if( bsize <= buffer_size - block_size ) bsize += block_size;
//   int search_size = bsize;			// bytes to search for trailer
//   int rd_size = bsize;				// bytes to read from file
//   unsigned long long ipos = pos - rd_size;	// aligned to block_size

//   while( true )
//     {
//     if( seek_read( fd, buffer, rd_size, ipos ) != rd_size )
//       { set_errno_error( "Error seeking member trailer: " ); return false; }
//     const uint8_t max_msb = ( ipos + search_size ) >> 56;
//     for( int i = search_size; i >= Lzip_trailer::size; --i )
//       if( buffer[i-1] <= max_msb )	// most significant byte of member_size
//         {
//         const Lzip_trailer & trailer =
//           *(const Lzip_trailer *)( buffer + i - trailer.size );
//         const unsigned long long member_size = trailer.member_size();
//         if( member_size == 0 )			// skip trailing zeros
//           { while( i > trailer.size && buffer[i-9] == 0 ) --i; continue; }
//         if( member_size > ipos + i || !trailer.check_consistency() ) continue;
//         Lzip_header header;
//         if( !read_header( fd, header, ipos + i - member_size,
//                           cl_opts.ignore_marking ) ) return false;
//         if( !header.check() ) continue;
//         const Lzip_header & header2 = *(const Lzip_header *)( buffer + i );
//         const bool full_h2 = bsize - i >= header.size;
//         if( header2.check_prefix( bsize - i ) )	// last member
//           {
//           if( !full_h2 ) error_ = "Last member in input file is truncated.";
//           else if( check_header( header2 ) )
//             error_ = "Last member in input file is truncated or corrupt.";
//           retval_ = 2; return false;
//           }
//         if( !cl_opts.loose_trailing && full_h2 && header2.check_corrupt() )
//           { error_ = corrupt_mm_msg; retval_ = 2; return false; }
//         if( !cl_opts.ignore_trailing )
//           { error_ = trailing_msg; retval_ = 2; return false; }
//         const unsigned long long data_size = trailer.data_size();
//         if( !cl_opts.ignore_empty && data_size == 0 )
//           { error_ = empty_msg; retval_ = 2; return false; }
//         pos = ipos + i - member_size;			// good member
//         const unsigned dictionary_size = header.dictionary_size();
//         if( dictionary_size_ < dictionary_size )
//           dictionary_size_ = dictionary_size;
//         member_vector.push_back( Member( 0, data_size, pos, member_size,
//                                          dictionary_size ) );
//         return true;
//         }
//     if( ipos == 0 )
//       { set_num_error( "Bad trailer at pos ", pos - Lzip_trailer::size );
//         return false; }
//     bsize = buffer_size;
//     search_size = bsize - Lzip_header::size;
//     rd_size = block_size;
//     ipos -= rd_size;
//     std::memcpy( buffer + rd_size, buffer, buffer_size - rd_size );
//     }
//   }

impl LzipIndex {
    pub fn from_reader(infd: &mut dyn io::BufRead, cl_opts: ClOptions) -> Result<(),()> {
        let insize = reader.seek()
    }
}
// Lzip_index::Lzip_index( const int infd, const Cl_options & cl_opts )
//   : insize( lseek( infd, 0, SEEK_END ) ), retval_( 0 ), dictionary_size_( 0 )
//   {
//   if( insize < 0 )
//     { set_errno_error( "Input file is not seekable: " ); return; }
//   if( insize < min_member_size )
//     { error_ = "Input file is too short."; retval_ = 2; return; }
//   if( insize > INT64_MAX )
//     { error_ = "Input file is too long (2^63 bytes or more).";
//       retval_ = 2; return; }

//   Lzip_header header;
//   if( !read_header( infd, header, 0, cl_opts.ignore_marking ) ||
//       !check_header( header ) ) return;

//   unsigned long long pos = insize;	// always points to a header or to EOF
//   while( pos >= min_member_size )
//     {
//     Lzip_trailer trailer;
//     if( seek_read( infd, trailer.data, trailer.size, pos - trailer.size ) !=
//         trailer.size )
//       { set_errno_error( "Error reading member trailer: " ); break; }
//     const unsigned long long member_size = trailer.member_size();
//     if( member_size > pos || !trailer.check_consistency() )	// bad trailer
//       {
//       if( member_vector.empty() )
//         { if( skip_trailing_data( infd, pos, cl_opts ) ) continue; return; }
//       set_num_error( "Bad trailer at pos ", pos - trailer.size ); break;
//       }
//     if( !read_header( infd, header, pos - member_size, cl_opts.ignore_marking ) )
//       break;
//     if( !header.check() )				// bad header
//       {
//       if( member_vector.empty() )
//         { if( skip_trailing_data( infd, pos, cl_opts ) ) continue; return; }
//       set_num_error( "Bad header at pos ", pos - member_size ); break;
//       }
//     const unsigned long long data_size = trailer.data_size();
//     if( !cl_opts.ignore_empty && data_size == 0 )
//       { error_ = empty_msg; retval_ = 2; break; }
//     pos -= member_size;					// good member
//     const unsigned dictionary_size = header.dictionary_size();
//     if( dictionary_size_ < dictionary_size )
//       dictionary_size_ = dictionary_size;
//     member_vector.push_back( Member( 0, data_size, pos, member_size,
//                                      dictionary_size ) );
//     }
//   if( pos != 0 || member_vector.empty() || retval_ != 0 )
//     {
//     member_vector.clear();
//     if( retval_ == 0 ) { error_ = "Can't create file index."; retval_ = 2; }
//     return;
//     }
//   std::reverse( member_vector.begin(), member_vector.end() );
//   for( unsigned long i = 0; ; ++i )
//     {
//     const long long end = member_vector[i].dblock.end();
//     if( end < 0 || end > INT64_MAX )
//       {
//       member_vector.clear();
//       error_ = "Data in input file is too long (2^63 bytes or more).";
//       retval_ = 2; return;
//       }
//     if( i + 1 >= member_vector.size() ) break;
//     member_vector[i+1].dblock.pos( end );
//     }
//   }
