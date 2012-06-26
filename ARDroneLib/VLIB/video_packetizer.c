#include <VLIB/video_codec.h>
#include <VLIB/video_packetizer.h>
#include <VLIB/Platform/video_utils.h>
#include <VLIB/Platform/video_config.h>

#include <VP_Os/vp_os_malloc.h>

C_RESULT video_packetizer_init( video_controller_t* controller )
{
  // Internal buffer configuration
  controller->in_stream.bytes     = vp_os_malloc( DEFAULT_INTERNAL_STREAM_SIZE );
  controller->in_stream.used      = 0;
  controller->in_stream.size      = DEFAULT_INTERNAL_STREAM_SIZE;
  controller->in_stream.index     = 0;
  controller->in_stream.length    = 32;
  controller->in_stream.code      = 0;
  controller->in_stream.endianess = VIDEO_STREAM_LITTLE_ENDIAN;

  return C_OK;
}

C_RESULT video_packetizer_close( video_controller_t* controller )
{
    if(controller->in_stream.bytes != NULL)
    {    
  vp_os_free( controller->in_stream.bytes );
  controller->in_stream.bytes   = NULL;
    }
    
  controller->in_stream.used    = 0;
  controller->in_stream.size    = 0;
  controller->in_stream.index   = 0;
  controller->in_stream.length  = 0;
  controller->in_stream.code    = 0;

  return C_OK;
}

C_RESULT video_cache_stream( video_controller_t* controller, video_stream_t* in )
{
  video_codec_t* video_codec = controller->video_codec;

  return video_codec->cache_stream( controller, in );
}

#ifndef HAS_VIDEO_WRITE_DATA

// Fill stream->code from right to left with data in parameters (code & length)
// New bits are always always inserted at the rigth of stream->code (least significant bits)
// This way old bits are put in most significant bits
//            31  ....   0    (length-1)    ....    0
//  stream <= ------------ <= -----------------------
//            stream->bits            code
void video_write_data( video_stream_t* const stream, uint32_t code, int32_t length )
{
  while( length > stream->length )
  {
    // code's length is bigger than number of our free bits
    // we put as many bits in cache as possible
    stream->code <<= stream->length;
    stream->code  |= code >> (length - stream->length);

    length -= stream->length;  // Compute number of bits left
    code   &= (1 << length) - 1; // We keep only bits we didn't push in cache

    stream->bytes[stream->index] = stream->code;
    stream->index++;
    stream->used += 4;

    stream->code    = 0;
    stream->length  = 32;
  }

  if( length > 0 )
  {
    // In this case, previous loop ended with case length < stream->length
    stream->code   <<= length;
    stream->code    |= code;

    stream->length -= length;
  }
}

#endif

C_RESULT video_stuff8( video_stream_t* const stream )
{
  uint32_t length8;

  length8  = (stream->length & ~7); // TODO: Check if generated code use bic on arm

  stream->code    <<= ( stream->length - length8 );
  stream->length    = length8;

  return C_OK;
}

// Fill code from right to left with length bits from stream->code
// Next bits in stream->code to take are always at the left (most significant bits)
// This way new bits are put in least significant bits
//  (length-1)    ....    0    31  ....   0
//  ----------------------- <= ------------ <= stream
//         code                stream->bits 
C_RESULT video_read_data( video_stream_t* const stream, uint32_t* code, int32_t length )
{
  uint32_t out_code = *code;

  while( length > (32 - stream->length) )
  {
    /// We need more bits than available in current read bits

    out_code = (out_code << (32 - stream->length)) | (stream->code >> stream->length);
    length  -= (32 - stream->length);

    stream->code    = stream->bytes[stream->index];
    stream->length  = 0;
    stream->index++;
  }

  if( length > 0 )
  {
    out_code  = (out_code << length) | (stream->code >> ( 32 - length ));

    stream->code  <<= length;
    stream->length += length;
  }

  *code = out_code;

  return C_OK;
}

C_RESULT video_peek_data( const video_stream_t* const stream, uint32_t* code, int32_t length )
{
  uint32_t out_code = *code;
  uint32_t stream_code = stream->code;
  uint32_t stream_length = stream->length;

  while( length > (32 - (int32_t)stream_length) )
  {
    /// We need more bits than available in current read bits

    out_code = (out_code << (32 - stream_length)) | (stream_code >> stream_length);
    length  -= (32 - stream_length);

    stream_code    = stream->bytes[stream->index];
    stream_length  = 0;
  }

  if( length > 0 )
  {
    out_code  = (out_code << length) | (stream_code >> ( 32 - length ));
  }

  *code = out_code;

  return C_OK;
}

C_RESULT video_align8( video_stream_t* const stream )
{
  uint32_t length8, length = stream->length;

  if( length > 0 )
  {
    // Do alignment only when stream->length > 0
    length8  = ( length & ~7); // TODO: Check if generated code use bic on arm
    if( length8 != length )
    {
      length8 += 0x08;
      stream->code    <<= ( length8 - length );
      stream->length    = length8;
    }
  }

  return C_OK;
}
