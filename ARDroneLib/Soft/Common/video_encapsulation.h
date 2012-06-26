#ifndef _PARROT_VIDEO_ENCAPSULATION_H_
#define _PARROT_VIDEO_ENCAPSULATION_H_

#include <VP_Os/vp_os_types.h>

typedef enum { 
	CODEC_UNKNNOWN=0,
	CODEC_VLIB,
	CODEC_P264,
	CODEC_MPEG4_VISUAL,
	CODEC_MPEG4_AVC
}parrot_video_encapsulation_codecs_t;


typedef enum {
	FRAME_TYPE_UNKNNOWN=0,
	FRAME_TYPE_IDR_FRAME, /* headers followed by I-frame */
	FRAME_TYPE_I_FRAME,
	FRAME_TYPE_P_FRAME,
	FRAME_TYPE_HEADERS
}parrot_video_encapsulation_frametypes_t;

typedef enum {
  PAVE_CTRL_FRAME_DATA           =0,            /* The PaVE is followed by video data */
  PAVE_CTRL_FRAME_ADVERTISEMENT  =(1<<0),     /* The PaVE is not followed by any data. Used to announce a frame which will be sent on the other socket later. */
  PAVE_CTRL_LAST_FRAME_IN_STREAM =(1<<1),    /* Announces the position of the last frame in the current stream */
}parrot_video_encapsulation_control_t;

/* Please keep this structure size a multiple of 8 or 16 */
/*
 * Please keep 16-bit words aligned on 16-bit boundaries
 * and keep 32-bit words aligned on 32-bit boundaries
 */
 
/*

*/

#define PAVE_CURRENT_VERSION (2)

typedef struct {
  /*00*/ uint8_t  signature[4];
  /*04*/ uint8_t  version;
  /*05*/ uint8_t  video_codec;
  /*06*/ uint16_t header_size;
  /*08*/ uint32_t payload_size;             /* Amount of data following this PaVE */
  /*12*/ uint16_t encoded_stream_width;     /* ex: 640 */
  /*14*/ uint16_t encoded_stream_height;    /* ex: 368 */
  /*16*/ uint16_t display_width;            /* ex: 640 */
  /*18*/ uint16_t display_height;           /* ex: 360 */
  /*20*/ uint32_t frame_number;             /* frame position inside the current stream */
  /*24*/ uint32_t timestamp;                /* in milliseconds */
  /*28*/ uint8_t  total_chuncks;            /* number of UDP packets containing the current decodable payload */
  /*29*/ uint8_t  chunck_index ;            /* position of the packet - first chunk is #0 */
  /*30*/ uint8_t  frame_type;               /* I-frame, P-frame */
  /*31*/ uint8_t  control;                  /* Special commands like end-of-stream or advertised frames */
  /*32*/ uint32_t stream_byte_position_lw;  /* Byte position of the current payload in the encoded stream  - lower 32-bit word */
  /*36*/ uint32_t stream_byte_position_uw;  /* Byte position of the current payload in the encoded stream  - upper 32-bit word */
  /*40*/ uint16_t stream_id;                /* This ID indentifies packets that should be recorded together */
  /*42*/ uint8_t  total_slices;             /* number of slices composing the current frame */
  /*43*/ uint8_t  slice_index ;             /* position of the current slice in the frame */
  /*44*/ uint8_t  header1_size;             /* H.264 only : size of SPS inside payload - no SPS present if value is zero */
  /*45*/ uint8_t  header2_size;             /* H.264 only : size of PPS inside payload - no PPS present if value is zero */
  /*46*/ uint8_t  reserved2[2];             /* Padding to align on 48 bytes */
  /*48*/ uint32_t advertised_size;          /* Size of frames announced as advertised frames */
  /*52*/ uint8_t  reserved3[12];            /* Padding to align on 64 bytes */
} __attribute__ ((packed)) parrot_video_encapsulation_t;

/* PaVE signature represented as a 32-bit integer in little and big endian */
#define PAVE_INT32LE_SIGNATURE (0x45566150)
#define PAVE_INT32BE_SIGNATURE (0x50615645)

#define PAVE_CHECK(x) ( (*((uint32_t*)(x)))==PAVE_INT32LE_SIGNATURE )

typedef enum
{
	PAVE_STREAM_ID_SUFFIX_MP4_360p = 0,
	PAVE_STREAM_ID_SUFFIX_H264_360p = 1,
	PAVE_STREAM_ID_SUFFIX_H264_720p = 2
}parrot_video_encapsulation_stream_id_suffixes_t;

C_RESULT init_parrot_video_encapsulation_header(parrot_video_encapsulation_t * header);
int pave_is_same_frame(parrot_video_encapsulation_t * header1 , parrot_video_encapsulation_t * header2 );
void dumpPave (parrot_video_encapsulation_t *PaVE);
#endif
