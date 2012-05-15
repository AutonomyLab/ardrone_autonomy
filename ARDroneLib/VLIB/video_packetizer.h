#ifndef _VIDEO_PACKETIZER_H_
#define _VIDEO_PACKETIZER_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_controller.h>

C_RESULT video_packetizer_init( video_controller_t* controller );
C_RESULT video_packetizer_close( video_controller_t* controller );


/// Write to a stream

// Fills a stream with length bits from code
void     video_write_data( video_stream_t* const stream, uint32_t code, int32_t length );

// Updates stream as its length attributes is a multiple of 8
// Updates is done by adding bits
C_RESULT video_stuff8( video_stream_t* const stream );


/// Read from a stream

// Takes length bit from stream and updates it
C_RESULT video_read_data( video_stream_t* const stream, uint32_t* code, int32_t length );

// Takes length bit from stream without updating it
C_RESULT video_peek_data( const video_stream_t* const stream, uint32_t* code, int32_t length );

// Updates stream as its length attributes is a multiple of 8
// Updates is done by skipping bits
C_RESULT video_align8( video_stream_t* const stream );


/// Copy a stream

// Flush content of stream in into stream out
C_RESULT video_cache_stream( video_controller_t* controller, video_stream_t* in );

#endif // _VIDEO_PACKETIZER_H_
