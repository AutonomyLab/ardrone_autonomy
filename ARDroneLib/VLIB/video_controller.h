#ifndef _VIDEO_CONTROLLER_H_
#define _VIDEO_CONTROLLER_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_picture.h>

#include <VLIB/video_picture.h>
#include <VLIB/video_gob.h>

enum {
  VIDEO_ENCODE  = 1,
  VIDEO_DECODE  = 2
};

enum {
  VIDEO_PICTURE_INTRA = 0,  // Picture is a reference frame
  VIDEO_PICTURE_INTER = 1,  // Picture is encoded using motion estimation / compensation
  VIDEO_PICTURE_PB    = 2,  // Picture is encoded using a PB frame
  VIDEO_PICTURE_B     = 3,
  VIDEO_PICTURE_EI    = 4,
  VIDEO_PICTURE_EP    = 5,
};

enum {
  VIDEO_STREAM_LITTLE_ENDIAN  = 1,
  VIDEO_STREAM_BIG_ENDIAN     = 2
};

typedef struct _video_controller_t  video_controller_t;
typedef struct _video_codec_t       video_codec_t;
typedef struct _video_stream_t      video_stream_t;

struct _video_stream_t {
  int32_t   length;     // Number of bits used in code (TODO why is it signed?)
  uint32_t  code;       // Currently read/write data
  uint32_t  used;       // Number of bytes used in stream
  uint32_t* bytes;      // Must be aligned on a 4-bytes boundary
  uint32_t  index;      // Position of next dword available for reading/writing
  uint32_t  size;       // Max size (in bytes, times of 4) of this stream
  uint32_t  endianess;  // Endianess of the stream
};

typedef C_RESULT (*encode_blockline_fc)( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete );
typedef C_RESULT (*decode_blockline_fc)( video_controller_t* controller, vp_api_picture_t* blockline, bool_t* got_image );
typedef C_RESULT (*update_fc)( video_controller_t* controller );
typedef C_RESULT (*cache_stream_fc)( video_controller_t* controller, video_stream_t* in );

struct _video_controller_t {
  // Configuration Data
  uint32_t        mode;           // encoding or decoding
  bool_t          use_me;         // use motion estimation / compensation
  bool_t          do_azq;
  int32_t         aq, bq;
  uint32_t        target_bitrate; // Target bitrate in bit/s
  uint32_t        target_size;    // Target size per image

  // External & internal buffer used by packetizer layer
  // video_stream_t* ex_stream;      // External buffer
  video_stream_t  in_stream;      // Internal buffer

  // Internal statistics
  uint32_t  num_frames;           // Frame index
  int32_t   current_bits;         // Number of bits in the buffer
  int32_t   output_bits;          // Number of bits occupied by the previous encoded picture
  int32_t   original_framerate;   // Frame rate of the original video sequence in pictures per second
  int32_t   target_framerate;     // Target frame rate in pictures per second (original_framerate / target_framerate must be an int)

  // Video Data for currently processed picture
  uint32_t  picture_type;
  int32_t   width;                // Size of picture currently decoded
  int32_t   height;
  bool_t    resolution_changed;   // if current frame resolution differs from previous one, this flag is set to TRUE
  int32_t   num_blockline;        // Number of blocklines per picture
  int32_t   mb_blockline;         // Number of macroblocks per blockline for this picture
  int32_t   blockline;            // Current blockline in picture
  bool_t    picture_complete;     // tells if picture is complete

  int32_t   quant;
  int32_t   dquant;
  int32_t   Qp;
  int32_t   invQp;

  video_gob_t*  gobs;             // Description of the picture as an array of gob
  int16_t*      cache;            // Cache that holds data for the whole picture (used internally by gobs)

  video_macroblock_t* cache_mbs;  // Array of macroblocks describing blockline_cache (used for decoding)
  int16_t*      blockline_cache;  // Cache used to hold intermediate results (for hardware DCT for example)

  // Codec specific functions
  uint32_t        codec_type;
  video_codec_t*  video_codec;
};

C_RESULT video_controller_update( video_controller_t* controller, bool_t complete );

C_RESULT video_controller_set_mode( video_controller_t* controller, uint32_t mode );

C_RESULT video_controller_cleanup( video_controller_t* controller );

// Configuration api
// video_controller_set_bitrate allows you to set the target bitrate
C_RESULT video_controller_set_bitrate( video_controller_t* controller, uint32_t target );

// video_controller_set_target_size allows you yo set a target size for each picture
C_RESULT video_controller_set_target_size( video_controller_t* controller, uint32_t target );

// Set format for picture to be decoded
// This function resize internal buffers if needed
C_RESULT video_controller_set_format( video_controller_t* controller, int32_t width, int32_t height );

// Set picture type ( INTRA or INTER )
C_RESULT  video_controller_set_picture_type( video_controller_t* controller, uint32_t type );

// Set motion estimation usage
C_RESULT  video_controller_set_motion_estimation( video_controller_t* controller, bool_t use_me );

static INLINE uint8_t* video_controller_get_stream_ptr( video_controller_t* controller ) {
  return (uint8_t*)&controller->in_stream.bytes[0];
}

static INLINE uint32_t video_controller_get_stream_size( video_controller_t* controller ) {
  return controller->in_stream.used;
}

#endif // _VIDEO_CONTROLLER_H_
