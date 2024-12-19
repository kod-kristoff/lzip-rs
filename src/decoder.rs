/* Lzip - LZMA lossless data compressor
   Copyright (C) 2008-2024 Antonio Diaz Diaz.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

class Range_decoder
  {
  enum { buffer_size = 16384 };
  unsigned long long partial_member_pos;
  uint8_t * const buffer;	// input buffer
  int pos;			// current pos in buffer
  int stream_pos;		// when reached, a new block must be read
  uint32_t code;
  uint32_t range;
  const int infd;		// input file descriptor
  bool at_stream_end;

  bool read_block();

  Range_decoder( const Range_decoder & );	// declared as private
  void operator=( const Range_decoder & );	// declared as private

public:
  explicit Range_decoder( const int ifd )
    :
    partial_member_pos( 0 ),
    buffer( new uint8_t[buffer_size] ),
    pos( 0 ),
    stream_pos( 0 ),
    code( 0 ),
    range( 0xFFFFFFFFU ),
    infd( ifd ),
    at_stream_end( false )
    {}

  ~Range_decoder() { delete[] buffer; }

  bool finished() { return pos >= stream_pos && !read_block(); }

  unsigned long long member_position() const
    { return partial_member_pos + pos; }

  void reset_member_position()
    { partial_member_pos = 0; partial_member_pos -= pos; }

  uint8_t get_byte()
    {
    // 0xFF avoids decoder error if member is truncated at EOS marker
    if( finished() ) return 0xFF;
    return buffer[pos++];
    }

  int read_data( uint8_t * const outbuf, const int size )
    {
    int sz = 0;
    while( sz < size && !finished() )
      {
      const int rd = std::min( size - sz, stream_pos - pos );
      std::memcpy( outbuf + sz, buffer + pos, rd );
      pos += rd;
      sz += rd;
      }
    return sz;
    }

  bool load( const bool ignore_marking = true )
    {
    code = 0;
    range = 0xFFFFFFFFU;
    // check and discard first byte of the LZMA stream
    if( get_byte() != 0 && !ignore_marking ) return false;
    for( int i = 0; i < 4; ++i ) code = ( code << 8 ) | get_byte();
    return true;
    }

  void normalize()
    {
    if( range <= 0x00FFFFFFU )
      { range <<= 8; code = ( code << 8 ) | get_byte(); }
    }

  unsigned decode( const int num_bits )
    {
    unsigned symbol = 0;
    for( int i = num_bits; i > 0; --i )
      {
      normalize();
      range >>= 1;
//      symbol <<= 1;
//      if( code >= range ) { code -= range; symbol |= 1; }
      const bool bit = ( code >= range );
      symbol <<= 1; symbol += bit;
      code -= range & ( 0U - bit );
      }
    return symbol;
    }

  bool decode_bit( Bit_model & bm )
    {
    normalize();
    const uint32_t bound = ( range >> bit_model_total_bits ) * bm.probability;
    if( code < bound )
      {
      range = bound;
      bm.probability +=
        ( bit_model_total - bm.probability ) >> bit_model_move_bits;
      return 0;
      }
    else
      {
      code -= bound;
      range -= bound;
      bm.probability -= bm.probability >> bit_model_move_bits;
      return 1;
      }
    }

  void decode_symbol_bit( Bit_model & bm, unsigned & symbol )
    {
    normalize();
    symbol <<= 1;
    const uint32_t bound = ( range >> bit_model_total_bits ) * bm.probability;
    if( code < bound )
      {
      range = bound;
      bm.probability +=
        ( bit_model_total - bm.probability ) >> bit_model_move_bits;
      }
    else
      {
      code -= bound;
      range -= bound;
      bm.probability -= bm.probability >> bit_model_move_bits;
      symbol |= 1;
      }
    }

  void decode_symbol_bit_reversed( Bit_model & bm, unsigned & model,
                                   unsigned & symbol, const int i )
    {
    normalize();
    model <<= 1;
    const uint32_t bound = ( range >> bit_model_total_bits ) * bm.probability;
    if( code < bound )
      {
      range = bound;
      bm.probability +=
        ( bit_model_total - bm.probability ) >> bit_model_move_bits;
      }
    else
      {
      code -= bound;
      range -= bound;
      bm.probability -= bm.probability >> bit_model_move_bits;
      model |= 1;
      symbol |= 1 << i;
      }
    }

  unsigned decode_tree6( Bit_model bm[] )
    {
    unsigned symbol = 1;
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    return symbol & 0x3F;
    }

  unsigned decode_tree8( Bit_model bm[] )
    {
    unsigned symbol = 1;
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    return symbol & 0xFF;
    }

  unsigned decode_tree_reversed( Bit_model bm[], const int num_bits )
    {
    unsigned model = 1;
    unsigned symbol = 0;
    for( int i = 0; i < num_bits; ++i )
      decode_symbol_bit_reversed( bm[model], model, symbol, i );
    return symbol;
    }

  unsigned decode_tree_reversed4( Bit_model bm[] )
    {
    unsigned model = 1;
    unsigned symbol = 0;
    decode_symbol_bit_reversed( bm[model], model, symbol, 0 );
    decode_symbol_bit_reversed( bm[model], model, symbol, 1 );
    decode_symbol_bit_reversed( bm[model], model, symbol, 2 );
    decode_symbol_bit_reversed( bm[model], model, symbol, 3 );
    return symbol;
    }

  unsigned decode_matched( Bit_model bm[], unsigned match_byte )
    {
    Bit_model * const bm1 = bm + 0x100;
    unsigned symbol = 1;
    while( symbol < 0x100 )
      {
      const unsigned match_bit = ( match_byte <<= 1 ) & 0x100;
      const bool bit = decode_bit( bm1[symbol+match_bit] );
      symbol <<= 1; symbol |= bit;
      if( match_bit >> 8 != bit )
        {
        while( symbol < 0x100 ) decode_symbol_bit( bm[symbol], symbol );
        break;
        }
      }
    return symbol & 0xFF;
    }

  unsigned decode_len( Len_model & lm, const int pos_state )
    {
    Bit_model * bm;
    unsigned mask, offset, symbol = 1;

    if( decode_bit( lm.choice1 ) == 0 )
      { bm = lm.bm_low[pos_state]; mask = 7; offset = 0; goto len3; }
    if( decode_bit( lm.choice2 ) == 0 )
      { bm = lm.bm_mid[pos_state]; mask = 7; offset = len_low_symbols; goto len3; }
    bm = lm.bm_high; mask = 0xFF; offset = len_low_symbols + len_mid_symbols;
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
len3:
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    decode_symbol_bit( bm[symbol], symbol );
    return ( symbol & mask ) + min_match_len + offset;
    }
  };


class LZ_decoder
  {
  unsigned long long partial_data_pos;
  Range_decoder & rdec;
  const unsigned dictionary_size;
  uint8_t * const buffer;	// output buffer
  unsigned pos;			// current pos in buffer
  unsigned stream_pos;		// first byte not yet written to file
  uint32_t crc_;
  const int outfd;		// output file descriptor
  bool pos_wrapped;

  void flush_data();
  int check_trailer( const Pretty_print & pp, const bool ignore_empty ) const;

  uint8_t peek_prev() const
    { return buffer[((pos > 0) ? pos : dictionary_size)-1]; }

  uint8_t peek( const unsigned distance ) const
    {
    const unsigned i = ( ( pos > distance ) ? 0 : dictionary_size ) +
                       pos - distance - 1;
    return buffer[i];
    }

  void put_byte( const uint8_t b )
    {
    buffer[pos] = b;
    if( ++pos >= dictionary_size ) flush_data();
    }

  void copy_block( const unsigned distance, unsigned len )
    {
    unsigned lpos = pos, i = lpos - distance - 1;
    bool fast, fast2;
    if( lpos > distance )
      {
      fast = ( len < dictionary_size - lpos );
      fast2 = ( fast && len <= lpos - i );
      }
    else
      {
      i += dictionary_size;
      fast = ( len < dictionary_size - i );	// (i == pos) may happen
      fast2 = ( fast && len <= i - lpos );
      }
    if( fast )					// no wrap
      {
      pos += len;
      if( fast2 )				// no wrap, no overlap
        std::memcpy( buffer + lpos, buffer + i, len );
      else
        for( ; len > 0; --len ) buffer[lpos++] = buffer[i++];
      }
    else for( ; len > 0; --len )
      {
      buffer[pos] = buffer[i];
      if( ++pos >= dictionary_size ) flush_data();
      if( ++i >= dictionary_size ) i = 0;
      }
    }

  LZ_decoder( const LZ_decoder & );		// declared as private
  void operator=( const LZ_decoder & );		// declared as private

public:
  LZ_decoder( Range_decoder & rde, const unsigned dict_size, const int ofd )
    :
    partial_data_pos( 0 ),
    rdec( rde ),
    dictionary_size( dict_size ),
    buffer( new uint8_t[dictionary_size] ),
    pos( 0 ),
    stream_pos( 0 ),
    crc_( 0xFFFFFFFFU ),
    outfd( ofd ),
    pos_wrapped( false )
    // prev_byte of first byte; also for peek( 0 ) on corrupt file
    { buffer[dictionary_size-1] = 0; }

  ~LZ_decoder() { delete[] buffer; }

  unsigned crc() const { return crc_ ^ 0xFFFFFFFFU; }
  unsigned long long data_position() const { return partial_data_pos + pos; }

  int decode_member( const Cl_options & cl_opts, const Pretty_print & pp );
  };
/* Lzip - LZMA lossless data compressor
   Copyright (C) 2008-2024 Antonio Diaz Diaz.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define _FILE_OFFSET_BITS 64

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <stdint.h>
#include <unistd.h>

#include "lzip.h"
#include "decoder.h"


/* Return the number of bytes really read.
   If (value returned < size) and (errno == 0), means EOF was reached.
*/
int readblock( const int fd, uint8_t * const buf, const int size )
  {
  int sz = 0;
  errno = 0;
  while( sz < size )
    {
    const int n = read( fd, buf + sz, size - sz );
    if( n > 0 ) sz += n;
    else if( n == 0 ) break;				// EOF
    else if( errno != EINTR ) break;
    errno = 0;
    }
  return sz;
  }


/* Return the number of bytes really written.
   If (value returned < size), it is always an error.
*/
int writeblock( const int fd, const uint8_t * const buf, const int size )
  {
  int sz = 0;
  errno = 0;
  while( sz < size )
    {
    const int n = write( fd, buf + sz, size - sz );
    if( n > 0 ) sz += n;
    else if( n < 0 && errno != EINTR ) break;
    errno = 0;
    }
  return sz;
  }


bool Range_decoder::read_block()
  {
  if( !at_stream_end )
    {
    stream_pos = readblock( infd, buffer, buffer_size );
    if( stream_pos != buffer_size && errno ) throw Error( "Read error" );
    at_stream_end = ( stream_pos < buffer_size );
    partial_member_pos += pos;
    pos = 0;
    show_dprogress();
    }
  return pos < stream_pos;
  }


void LZ_decoder::flush_data()
  {
  if( pos > stream_pos )
    {
    const int size = pos - stream_pos;
    crc32.update_buf( crc_, buffer + stream_pos, size );
    if( outfd >= 0 && writeblock( outfd, buffer + stream_pos, size ) != size )
      throw Error( "Write error" );
    if( pos >= dictionary_size )
      { partial_data_pos += pos; pos = 0; pos_wrapped = true; }
    stream_pos = pos;
    }
  }


int LZ_decoder::check_trailer( const Pretty_print & pp,
                               const bool ignore_empty ) const
  {
  Lzip_trailer trailer;
  int size = rdec.read_data( trailer.data, trailer.size );
  bool error = false;

  if( size < trailer.size )
    {
    error = true;
    if( verbosity >= 0 )
      { pp();
        std::fprintf( stderr, "Trailer truncated at trailer position %d;"
                              " some checks may fail.\n", size ); }
    while( size < trailer.size ) trailer.data[size++] = 0;
    }

  const unsigned td_crc = trailer.data_crc();
  if( td_crc != crc() )
    {
    error = true;
    if( verbosity >= 0 )
      { pp();
        std::fprintf( stderr, "CRC mismatch; stored %08X, computed %08X\n",
                      td_crc, crc() ); }
    }
  const unsigned long long data_size = data_position();
  const unsigned long long td_size = trailer.data_size();
  if( td_size != data_size )
    {
    error = true;
    if( verbosity >= 0 )
      { pp();
        std::fprintf( stderr, "Data size mismatch; stored %llu (0x%llX), computed %llu (0x%llX)\n",
                      td_size, td_size, data_size, data_size ); }
    }
  const unsigned long long member_size = rdec.member_position();
  const unsigned long long tm_size = trailer.member_size();
  if( tm_size != member_size )
    {
    error = true;
    if( verbosity >= 0 )
      { pp();
        std::fprintf( stderr, "Member size mismatch; stored %llu (0x%llX), computed %llu (0x%llX)\n",
                      tm_size, tm_size, member_size, member_size ); }
    }
  if( error ) return 3;
  if( !ignore_empty && data_size == 0 ) return 5;
  if( verbosity >= 2 )
    {
    if( verbosity >= 4 ) show_header( dictionary_size );
    if( data_size == 0 || member_size == 0 )
      std::fputs( "no data compressed. ", stderr );
    else
      std::fprintf( stderr, "%6.3f:1, %5.2f%% ratio, %5.2f%% saved. ",
                    (double)data_size / member_size,
                    ( 100.0 * member_size ) / data_size,
                    100.0 - ( ( 100.0 * member_size ) / data_size ) );
    if( verbosity >= 4 ) std::fprintf( stderr, "CRC %08X, ", td_crc );
    if( verbosity >= 3 )
      std::fprintf( stderr, "%9llu out, %8llu in. ", data_size, member_size );
    }
  return 0;
  }


/* Return value: 0 = OK, 1 = decoder error, 2 = unexpected EOF,
                 3 = trailer error, 4 = unknown marker found,
                 5 = empty member found, 6 = marked member found. */
int LZ_decoder::decode_member( const Cl_options & cl_opts,
                               const Pretty_print & pp )
  {
  Bit_model bm_literal[1<<literal_context_bits][0x300];
  Bit_model bm_match[State::states][pos_states];
  Bit_model bm_rep[State::states];
  Bit_model bm_rep0[State::states];
  Bit_model bm_rep1[State::states];
  Bit_model bm_rep2[State::states];
  Bit_model bm_len[State::states][pos_states];
  Bit_model bm_dis_slot[len_states][1<<dis_slot_bits];
  Bit_model bm_dis[modeled_distances-end_dis_model+1];
  Bit_model bm_align[dis_align_size];
  Len_model match_len_model;
  Len_model rep_len_model;
  unsigned rep0 = 0;		// rep[0-3] latest four distances
  unsigned rep1 = 0;		// used for efficient coding of
  unsigned rep2 = 0;		// repeated distances
  unsigned rep3 = 0;
  State state;

  if( !rdec.load( cl_opts.ignore_marking ) ) return 6;
  while( !rdec.finished() )
    {
    const int pos_state = data_position() & pos_state_mask;
    if( rdec.decode_bit( bm_match[state()][pos_state] ) == 0 )	// 1st bit
      {
      // literal byte
      Bit_model * const bm = bm_literal[get_lit_state(peek_prev())];
      if( state.is_char_set_char() )
        put_byte( rdec.decode_tree8( bm ) );
      else
        put_byte( rdec.decode_matched( bm, peek( rep0 ) ) );
      continue;
      }
    // match or repeated match
    int len;
    if( rdec.decode_bit( bm_rep[state()] ) != 0 )		// 2nd bit
      {
      if( rdec.decode_bit( bm_rep0[state()] ) == 0 )		// 3rd bit
        {
        if( rdec.decode_bit( bm_len[state()][pos_state] ) == 0 ) // 4th bit
          { state.set_short_rep(); put_byte( peek( rep0 ) ); continue; }
        }
      else
        {
        unsigned distance;
        if( rdec.decode_bit( bm_rep1[state()] ) == 0 )		// 4th bit
          distance = rep1;
        else
          {
          if( rdec.decode_bit( bm_rep2[state()] ) == 0 )	// 5th bit
            distance = rep2;
          else
            { distance = rep3; rep3 = rep2; }
          rep2 = rep1;
          }
        rep1 = rep0;
        rep0 = distance;
        }
      state.set_rep();
      len = rdec.decode_len( rep_len_model, pos_state );
      }
    else					// match
      {
      len = rdec.decode_len( match_len_model, pos_state );
      unsigned distance = rdec.decode_tree6( bm_dis_slot[get_len_state(len)] );
      if( distance >= start_dis_model )
        {
        const unsigned dis_slot = distance;
        const int direct_bits = ( dis_slot >> 1 ) - 1;
        distance = ( 2 | ( dis_slot & 1 ) ) << direct_bits;
        if( dis_slot < end_dis_model )
          distance += rdec.decode_tree_reversed(
                      bm_dis + ( distance - dis_slot ), direct_bits );
        else
          {
          distance +=
            rdec.decode( direct_bits - dis_align_bits ) << dis_align_bits;
          distance += rdec.decode_tree_reversed4( bm_align );
          if( distance == 0xFFFFFFFFU )		// marker found
            {
            rdec.normalize();
            flush_data();
            if( len == min_match_len )		// End Of Stream marker
              return check_trailer( pp, cl_opts.ignore_empty );
            if( len == min_match_len + 1 )	// Sync Flush marker
              { rdec.load(); continue; }
            if( verbosity >= 0 )
              {
              pp();
              std::fprintf( stderr, "Unsupported marker code '%d'\n", len );
              }
            return 4;
            }
          }
        }
      rep3 = rep2; rep2 = rep1; rep1 = rep0; rep0 = distance;
      state.set_match();
      if( rep0 >= dictionary_size || ( rep0 >= pos && !pos_wrapped ) )
        { flush_data(); return 1; }
      }
    copy_block( rep0, len );
    }
  flush_data();
  return 2;
  }
