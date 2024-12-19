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

enum { price_shift_bits = 6,
       price_step_bits = 2,
       price_step = 1 << price_step_bits };

class Dis_slots
  {
  uint8_t data[1<<10];

public:
  void init()
    {
    for( int slot = 0; slot < 4; ++slot ) data[slot] = slot;
    for( int i = 4, size = 2, slot = 4; slot < 20; slot += 2 )
      {
      std::memset( &data[i], slot, size );
      std::memset( &data[i+size], slot + 1, size );
      size <<= 1;
      i += size;
      }
    }

  uint8_t operator[]( const int dis ) const { return data[dis]; }
  };

extern Dis_slots dis_slots;

inline uint8_t get_slot( const unsigned dis )
  {
  if( dis < (1 << 10) ) return dis_slots[dis];
  if( dis < (1 << 19) ) return dis_slots[dis>> 9] + 18;
  if( dis < (1 << 28) ) return dis_slots[dis>>18] + 36;
  return dis_slots[dis>>27] + 54;
  }


class Prob_prices
  {
  short data[bit_model_total >> price_step_bits];

public:
  void init()
    {
    for( int i = 0; i < bit_model_total >> price_step_bits; ++i )
      {
      unsigned val = ( i * price_step ) + ( price_step / 2 );
      int bits = 0;				// base 2 logarithm of val
      for( int j = 0; j < price_shift_bits; ++j )
        {
        val = val * val;
        bits <<= 1;
        while( val >= 1 << 16 ) { val >>= 1; ++bits; }
        }
      bits += 15;				// remaining bits in val
      data[i] = ( bit_model_total_bits << price_shift_bits ) - bits;
      }
    }

  int operator[]( const int probability ) const
    { return data[probability >> price_step_bits]; }
  };

extern Prob_prices prob_prices;


inline int price0( const Bit_model bm )
  { return prob_prices[bm.probability]; }

inline int price1( const Bit_model bm )
  { return prob_prices[bit_model_total - bm.probability]; }

inline int price_bit( const Bit_model bm, const bool bit )
  { return bit ? price1( bm ) : price0( bm ); }


inline int price_symbol3( const Bit_model bm[], int symbol )
  {
  bool bit = symbol & 1;
  symbol |= 8; symbol >>= 1;
  int price = price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  return price + price_bit( bm[1], symbol & 1 );
  }


inline int price_symbol6( const Bit_model bm[], unsigned symbol )
  {
  bool bit = symbol & 1;
  symbol |= 64; symbol >>= 1;
  int price = price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  return price + price_bit( bm[1], symbol & 1 );
  }


inline int price_symbol8( const Bit_model bm[], int symbol )
  {
  bool bit = symbol & 1;
  symbol |= 0x100; symbol >>= 1;
  int price = price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  bit = symbol & 1; symbol >>= 1; price += price_bit( bm[symbol], bit );
  return price + price_bit( bm[1], symbol & 1 );
  }


inline int price_symbol_reversed( const Bit_model bm[], int symbol,
                                  const int num_bits )
  {
  int price = 0;
  int model = 1;
  for( int i = num_bits; i > 0; --i )
    {
    const bool bit = symbol & 1;
    symbol >>= 1;
    price += price_bit( bm[model], bit );
    model <<= 1; model |= bit;
    }
  return price;
  }


inline int price_matched( const Bit_model bm[], unsigned symbol,
                          unsigned match_byte )
  {
  int price = 0;
  unsigned mask = 0x100;
  symbol |= mask;
  while( true )
    {
    const unsigned match_bit = ( match_byte <<= 1 ) & mask;
    const bool bit = ( symbol <<= 1 ) & 0x100;
    price += price_bit( bm[(symbol>>9)+match_bit+mask], bit );
    if( symbol >= 0x10000 ) return price;
    mask &= ~(match_bit ^ symbol);	// if( match_bit != bit ) mask = 0;
    }
  }


class Matchfinder_base
  {
  bool read_block();
  void normalize_pos();

  Matchfinder_base( const Matchfinder_base & );	// declared as private
  void operator=( const Matchfinder_base & );	// declared as private

protected:
  unsigned long long partial_data_pos;
  uint8_t * buffer;		// input buffer
  int32_t * prev_positions;	// 1 + last seen position of key. else 0
  int32_t * pos_array;		// may be tree or chain
  const int before_size;	// bytes to keep in buffer before dictionary
  int buffer_size;
  int dictionary_size;		// bytes to keep in buffer before pos
  int pos;			// current pos in buffer
  int cyclic_pos;		// cycles through [0, dictionary_size]
  int stream_pos;		// first byte not yet read from file
  int pos_limit;		// when reached, a new block must be read
  int key4_mask;
  const int num_prev_positions23;
  int num_prev_positions;	// size of prev_positions
  int pos_array_size;
  const int infd;		// input file descriptor
  bool at_stream_end;		// stream_pos shows real end of file

  Matchfinder_base( const int before_size_,
                    const int dict_size, const int after_size,
                    const int dict_factor, const int num_prev_positions23_,
                    const int pos_array_factor, const int ifd );

  ~Matchfinder_base()
    { delete[] prev_positions; std::free( buffer ); }

public:
  uint8_t peek( const int distance ) const { return buffer[pos-distance]; }
  int available_bytes() const { return stream_pos - pos; }
  unsigned long long data_position() const { return partial_data_pos + pos; }
  bool data_finished() const { return at_stream_end && pos >= stream_pos; }
  const uint8_t * ptr_to_current_pos() const { return buffer + pos; }

  int true_match_len( const int index, const int distance ) const
    {
    const uint8_t * const data = buffer + pos;
    int i = index;
    const int len_limit = std::min( available_bytes(), (int)max_match_len );
    while( i < len_limit && data[i-distance] == data[i] ) ++i;
    return i;
    }

  void move_pos()
    {
    if( ++cyclic_pos > dictionary_size ) cyclic_pos = 0;
    if( ++pos >= pos_limit ) normalize_pos();
    }

  void reset();
  };


class Range_encoder
  {
  enum { buffer_size = 65536 };
  uint64_t low;
  unsigned long long partial_member_pos;
  uint8_t * const buffer;	// output buffer
  int pos;			// current pos in buffer
  uint32_t range;
  unsigned ff_count;
  const int outfd;		// output file descriptor
  uint8_t cache;
  Lzip_header header;

  void shift_low()
    {
    if( low >> 24 != 0xFF )
      {
      const bool carry = ( low > 0xFFFFFFFFU );
      put_byte( cache + carry );
      for( ; ff_count > 0; --ff_count ) put_byte( 0xFF + carry );
      cache = low >> 24;
      }
    else ++ff_count;
    low = ( low & 0x00FFFFFFU ) << 8;
    }

  Range_encoder( const Range_encoder & );	// declared as private
  void operator=( const Range_encoder & );	// declared as private

public:
  void reset( const unsigned dictionary_size )
    {
    low = 0;
    partial_member_pos = 0;
    pos = 0;
    range = 0xFFFFFFFFU;
    ff_count = 0;
    cache = 0;
    header.dictionary_size( dictionary_size );
    for( int i = 0; i < header.size; ++i ) put_byte( header.data[i] );
    }

  Range_encoder( const unsigned dictionary_size, const int ofd )
    :
    buffer( new uint8_t[buffer_size] ), outfd( ofd )
    {
    header.set_magic();
    reset( dictionary_size );
    }

  ~Range_encoder() { delete[] buffer; }

  unsigned long long member_position() const
    { return partial_member_pos + pos + ff_count; }

  void flush() { for( int i = 0; i < 5; ++i ) shift_low(); }
  void flush_data();

  void put_byte( const uint8_t b )
    {
    buffer[pos] = b;
    if( ++pos >= buffer_size ) flush_data();
    }

  void encode( const int symbol, const int num_bits )
    {
    for( unsigned mask = 1 << ( num_bits - 1 ); mask > 0; mask >>= 1 )
      {
      range >>= 1;
      if( symbol & mask ) low += range;
      if( range <= 0x00FFFFFFU ) { range <<= 8; shift_low(); }
      }
    }

  void encode_bit( Bit_model & bm, const bool bit )
    {
    const uint32_t bound = ( range >> bit_model_total_bits ) * bm.probability;
    if( !bit )
      {
      range = bound;
      bm.probability +=
        ( bit_model_total - bm.probability ) >> bit_model_move_bits;
      }
    else
      {
      low += bound;
      range -= bound;
      bm.probability -= bm.probability >> bit_model_move_bits;
      }
    if( range <= 0x00FFFFFFU ) { range <<= 8; shift_low(); }
    }

  void encode_tree3( Bit_model bm[], const int symbol )
    {
    bool bit = ( symbol >> 2 ) & 1;
    encode_bit( bm[1], bit );
    int model = 2 | bit;
    bit = ( symbol >> 1 ) & 1;
    encode_bit( bm[model], bit ); model <<= 1; model |= bit;
    encode_bit( bm[model], symbol & 1 );
    }

  void encode_tree6( Bit_model bm[], const unsigned symbol )
    {
    bool bit = ( symbol >> 5 ) & 1;
    encode_bit( bm[1], bit );
    int model = 2 | bit;
    bit = ( symbol >> 4 ) & 1;
    encode_bit( bm[model], bit ); model <<= 1; model |= bit;
    bit = ( symbol >> 3 ) & 1;
    encode_bit( bm[model], bit ); model <<= 1; model |= bit;
    bit = ( symbol >> 2 ) & 1;
    encode_bit( bm[model], bit ); model <<= 1; model |= bit;
    bit = ( symbol >> 1 ) & 1;
    encode_bit( bm[model], bit ); model <<= 1; model |= bit;
    encode_bit( bm[model], symbol & 1 );
    }

  void encode_tree8( Bit_model bm[], const int symbol )
    {
    int model = 1;
    for( int i = 7; i >= 0; --i )
      {
      const bool bit = ( symbol >> i ) & 1;
      encode_bit( bm[model], bit );
      model <<= 1; model |= bit;
      }
    }

  void encode_tree_reversed( Bit_model bm[], int symbol, const int num_bits )
    {
    int model = 1;
    for( int i = num_bits; i > 0; --i )
      {
      const bool bit = symbol & 1;
      symbol >>= 1;
      encode_bit( bm[model], bit );
      model <<= 1; model |= bit;
      }
    }

  void encode_matched( Bit_model bm[], unsigned symbol, unsigned match_byte )
    {
    unsigned mask = 0x100;
    symbol |= mask;
    while( true )
      {
      const unsigned match_bit = ( match_byte <<= 1 ) & mask;
      const bool bit = ( symbol <<= 1 ) & 0x100;
      encode_bit( bm[(symbol>>9)+match_bit+mask], bit );
      if( symbol >= 0x10000 ) break;
      mask &= ~(match_bit ^ symbol);	// if( match_bit != bit ) mask = 0;
      }
    }

  void encode_len( Len_model & lm, int symbol, const int pos_state )
    {
    bool bit = ( ( symbol -= min_match_len ) >= len_low_symbols );
    encode_bit( lm.choice1, bit );
    if( !bit )
      encode_tree3( lm.bm_low[pos_state], symbol );
    else
      {
      bit = ( ( symbol -= len_low_symbols ) >= len_mid_symbols );
      encode_bit( lm.choice2, bit );
      if( !bit )
        encode_tree3( lm.bm_mid[pos_state], symbol );
      else
        encode_tree8( lm.bm_high, symbol - len_mid_symbols );
      }
    }
  };


class LZ_encoder_base : public Matchfinder_base
  {
protected:
  enum { max_marker_size = 16,
         num_rep_distances = 4 };	// must be 4

  uint32_t crc_;

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
  Range_encoder renc;

  LZ_encoder_base( const int before_size, const int dict_size,
                   const int after_size, const int dict_factor,
                   const int num_prev_positions23,
                   const int pos_array_factor,
                   const int ifd, const int outfd )
    :
    Matchfinder_base( before_size, dict_size, after_size, dict_factor,
                      num_prev_positions23, pos_array_factor, ifd ),
    crc_( 0xFFFFFFFFU ),
    renc( dictionary_size, outfd )
    {}

  unsigned crc() const { return crc_ ^ 0xFFFFFFFFU; }

  int price_literal( const uint8_t prev_byte, const uint8_t symbol ) const
    { return price_symbol8( bm_literal[get_lit_state(prev_byte)], symbol ); }

  int price_matched( const uint8_t prev_byte, const uint8_t symbol,
                     const uint8_t match_byte ) const
    { return ::price_matched( bm_literal[get_lit_state(prev_byte)], symbol,
                              match_byte ); }

  void encode_literal( const uint8_t prev_byte, const uint8_t symbol )
    { renc.encode_tree8( bm_literal[get_lit_state(prev_byte)], symbol ); }

  void encode_matched( const uint8_t prev_byte, const uint8_t symbol,
                       const uint8_t match_byte )
    { renc.encode_matched( bm_literal[get_lit_state(prev_byte)], symbol,
                           match_byte ); }

  void encode_pair( const unsigned dis, const int len, const int pos_state )
    {
    renc.encode_len( match_len_model, len, pos_state );
    const unsigned dis_slot = get_slot( dis );
    renc.encode_tree6( bm_dis_slot[get_len_state(len)], dis_slot );

    if( dis_slot >= start_dis_model )
      {
      const int direct_bits = ( dis_slot >> 1 ) - 1;
      const unsigned base = ( 2 | ( dis_slot & 1 ) ) << direct_bits;
      const unsigned direct_dis = dis - base;

      if( dis_slot < end_dis_model )
        renc.encode_tree_reversed( bm_dis + ( base - dis_slot ),
                                   direct_dis, direct_bits );
      else
        {
        renc.encode( direct_dis >> dis_align_bits, direct_bits - dis_align_bits );
        renc.encode_tree_reversed( bm_align, direct_dis, dis_align_bits );
        }
      }
    }

  void full_flush( const State state );

public:
  virtual ~LZ_encoder_base() {}

  unsigned long long member_position() const { return renc.member_position(); }
  virtual void reset();

  virtual bool encode_member( const unsigned long long member_size ) = 0;
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
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <vector>
#include <stdint.h>

#include "lzip.h"
#include "encoder_base.h"


Dis_slots dis_slots;
Prob_prices prob_prices;


bool Matchfinder_base::read_block()
  {
  if( !at_stream_end && stream_pos < buffer_size )
    {
    const int size = buffer_size - stream_pos;
    const int rd = readblock( infd, buffer + stream_pos, size );
    stream_pos += rd;
    if( rd != size && errno ) throw Error( "Read error" );
    if( rd < size ) { at_stream_end = true; pos_limit = buffer_size; }
    }
  return pos < stream_pos;
  }


void Matchfinder_base::normalize_pos()
  {
  if( pos > stream_pos )
    internal_error( "pos > stream_pos in normalize_pos." );
  if( !at_stream_end )
    {
    // offset is int32_t for the std::min below
    const int32_t offset = pos - before_size - dictionary_size;
    const int size = stream_pos - offset;
    std::memmove( buffer, buffer + offset, size );
    partial_data_pos += offset;
    pos -= offset;		// pos = before_size + dictionary_size
    stream_pos -= offset;
    for( int i = 0; i < num_prev_positions; ++i )
      prev_positions[i] -= std::min( prev_positions[i], offset );
    for( int i = 0; i < pos_array_size; ++i )
      pos_array[i] -= std::min( pos_array[i], offset );
    read_block();
    }
  }


Matchfinder_base::Matchfinder_base( const int before_size_,
                    const int dict_size, const int after_size,
                    const int dict_factor, const int num_prev_positions23_,
                    const int pos_array_factor, const int ifd )
  :
  partial_data_pos( 0 ),
  before_size( before_size_ ),
  pos( 0 ),
  cyclic_pos( 0 ),
  stream_pos( 0 ),
  num_prev_positions23( num_prev_positions23_ ),
  infd( ifd ),
  at_stream_end( false )
  {
  const int buffer_size_limit =
    ( dict_factor * dict_size ) + before_size + after_size;
  buffer_size = std::max( 65536, dict_size );
  buffer = (uint8_t *)std::malloc( buffer_size );
  if( !buffer ) throw std::bad_alloc();
  if( read_block() && !at_stream_end && buffer_size < buffer_size_limit )
    {
    uint8_t * const tmp = (uint8_t *)std::realloc( buffer, buffer_size_limit );
    if( !tmp ) { std::free( buffer ); throw std::bad_alloc(); }
    buffer = tmp;
    buffer_size = buffer_size_limit;
    read_block();
    }
  if( at_stream_end && stream_pos < dict_size )
    dictionary_size = std::max( (int)min_dictionary_size, stream_pos );
  else
    dictionary_size = dict_size;
  pos_limit = buffer_size;
  if( !at_stream_end ) pos_limit -= after_size;
  unsigned size = 1 << std::max( 16, real_bits( dictionary_size - 1 ) - 2 );
  if( dictionary_size > 1 << 26 ) size >>= 1;		// 64 MiB
  key4_mask = size - 1;			// increases with dictionary size
  size += num_prev_positions23;
  num_prev_positions = size;

  pos_array_size = pos_array_factor * ( dictionary_size + 1 );
  size += pos_array_size;
  if( size * sizeof prev_positions[0] <= size ) prev_positions = 0;
  else prev_positions = new( std::nothrow ) int32_t[size];
  if( !prev_positions ) { std::free( buffer ); throw std::bad_alloc(); }
  pos_array = prev_positions + num_prev_positions;
  for( int i = 0; i < num_prev_positions; ++i ) prev_positions[i] = 0;
  }


void Matchfinder_base::reset()
  {
  if( stream_pos > pos )
    std::memmove( buffer, buffer + pos, stream_pos - pos );
  partial_data_pos = 0;
  stream_pos -= pos;
  pos = 0;
  cyclic_pos = 0;
  read_block();
  if( at_stream_end && stream_pos < dictionary_size )
    {
    dictionary_size = std::max( (int)min_dictionary_size, stream_pos );
    int size = 1 << std::max( 16, real_bits( dictionary_size - 1 ) - 2 );
    if( dictionary_size > 1 << 26 ) size >>= 1;		// 64 MiB
    key4_mask = size - 1;
    size += num_prev_positions23;
    num_prev_positions = size;
    pos_array = prev_positions + num_prev_positions;
    }
  for( int i = 0; i < num_prev_positions; ++i ) prev_positions[i] = 0;
  }


void Range_encoder::flush_data()
  {
  if( pos > 0 )
    {
    if( outfd >= 0 && writeblock( outfd, buffer, pos ) != pos )
      throw Error( "Write error" );
    partial_member_pos += pos;
    pos = 0;
    show_cprogress();
    }
  }


// End Of Stream marker => (dis == 0xFFFFFFFFU, len == min_match_len)
void LZ_encoder_base::full_flush( const State state )
  {
  const int pos_state = data_position() & pos_state_mask;
  renc.encode_bit( bm_match[state()][pos_state], 1 );
  renc.encode_bit( bm_rep[state()], 0 );
  encode_pair( 0xFFFFFFFFU, min_match_len, pos_state );
  renc.flush();
  Lzip_trailer trailer;
  trailer.data_crc( crc() );
  trailer.data_size( data_position() );
  trailer.member_size( renc.member_position() + trailer.size );
  for( int i = 0; i < trailer.size; ++i ) renc.put_byte( trailer.data[i] );
  renc.flush_data();
  }


void LZ_encoder_base::reset()
  {
  Matchfinder_base::reset();
  crc_ = 0xFFFFFFFFU;
  bm_literal[0][0].reset( (1 << literal_context_bits) * 0x300 );
  bm_match[0][0].reset( State::states * pos_states );
  bm_rep[0].reset( State::states );
  bm_rep0[0].reset( State::states );
  bm_rep1[0].reset( State::states );
  bm_rep2[0].reset( State::states );
  bm_len[0][0].reset( State::states * pos_states );
  bm_dis_slot[0][0].reset( len_states * (1 << dis_slot_bits) );
  bm_dis[0].reset( modeled_distances - end_dis_model + 1 );
  bm_align[0].reset( dis_align_size );
  match_len_model.reset();
  rep_len_model.reset();
  renc.reset( dictionary_size );
  }
