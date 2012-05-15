#include <VLIB/Platform/video_utils.h>
#include <VLIB/video_packetizer.h>

#include <VP_Os/vp_os_assert.h>

#include <VP_Os/vp_os_types.h>

#include <VLIB/UVLC/uvlc.h>

#ifndef HAS_UVLC_ENCODE

#define PACK_BITS( bits_out, length_out, bits_in, length_in ) \
  bits_out  <<= length_in;                                    \
  length_out += length_in;                                    \
  bits_out   |= bits_in;

void uvlc_encode( video_stream_t* const stream, int32_t level, int32_t run, int32_t not_last )
{
  int32_t sign, length, data, value_code, value_length;

  /// Encode number of zeros
  data = run;

  length      = 0;
  value_code  = 1;

  if( data > 0 )
  {
    length = 32 - clz(data);      // compute number of bits used in run ( = length of run )
    data  -= 1 << ( length - 1 ); // compute value of run
  }

  value_length  = length + 1;

  length -= 1;
  if( length > 0 )
  {
    PACK_BITS( value_code, value_length, data, length );
  }

  /// Encode level
  data = level;

  // sign handling
  sign = 0;
  if( data < 0 )
  {
    data = -data;
    sign = 1;
  }

  // TODO Check saturation & if level == -128
  length = 32 - clz(data);  // number of bits used in level ( = length of level )
  if( length > 1 )
  {
    data   -= 1 << (length - 1);
    length += 1;
  }

  PACK_BITS( value_code, value_length, 1, length );

  VP_OS_ASSERT( length != 2 );

  length -= 2;
  if(length > 0)
  {
    PACK_BITS( value_code, value_length, data, length );
  }

  PACK_BITS( value_code, value_length, sign, 1 );

  /// Encode last
  // add sequence for end of block if required
  if( not_last == 0 )
  {
    PACK_BITS( value_code, value_length, 0x5, 3 );
  }

  /// Write output
  video_write_data( stream, value_code, value_length );
}

#endif // HAS_UVLC_ENCODE

C_RESULT uvlc_decode( video_stream_t* const stream, int32_t* run, int32_t* level, int32_t* last)
{
  uint32_t stream_code, stream_length;
  int32_t r = 0, z, sign;

  stream_code = stream_length = 0;

  // Peek 32 bits from stream because we know our datas fit in
  video_peek_data( stream, &stream_code, 32 );


  /// Decode number of zeros
  z = clz(stream_code);

  stream_code  <<= z + 1; // Skip all zeros & 1
  stream_length += z + 1;

  if( z > 1 )
  {
    r = stream_code >> (32 - (z-1));

    stream_code   <<= (z-1);
    stream_length  += (z-1);

    *run = r + (1 << (z-1));
  }
  else
  {
    *run = z;
  }


  /// Decode level / last
  z = clz(stream_code);

  stream_code  <<= z + 1; // Skip all zeros & 1
  stream_length += z + 1;

  if( z == 1 )
  {
    *run  = 0;
    *last = 1;
  }
  else
  {
    if( z == 0 )
    {
      z = 1;
      r = 1;
    }

    stream_length  += z;

    stream_code >>= (32 - z);
    sign = stream_code & 1;

    if( z != 0 )
    {
      r  = stream_code >> 1;
      r += 1 << (z-1);
    }

    *level = sign ? -r : r;
    *last  = 0;
  }

  // Do the real Read in stream to consume what we used
  video_read_data( stream, &stream_code, stream_length );

  return C_OK;
}
