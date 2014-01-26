/*
 *  video_stage_decoder.h
 *  ARDroneLib
 *
 *  Created by n.brulez on 02/08/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#if defined (FFMPEG_SUPPORT) || defined (ITTIAM_SUPPORT)

#ifndef __VIDEO_STAGE_DECODER_H__
#define __VIDEO_STAGE_DECODER_H__

#ifdef FFMPEG_SUPPORT
# define mp4h264_config_t ffmpeg_stage_decoding_config_t
#elif defined (ITTIAM_SUPPORT)
# define mp4h264_config_t ittiam_stage_decoding_config_t
#else
# error Either FFMPEG_SUPPORT or ITTIAM_SUPPORT must be defined
#endif

#ifdef ITTIAM_SUPPORT
# include <ardrone_tool/Video/video_stage_ittiam_decoder.h>
#endif

#ifdef FFMPEG_SUPPORT
# include <ardrone_tool/Video/video_stage_ffmpeg_decoder.h>
#endif

#include <VLIB/Stages/vlib_stage_decode.h>

typedef struct _video_decoder_config_t
{
  // Input data : dst_picture->format
  // Output : all others
  vp_api_picture_t *src_picture;
  vp_api_picture_t *dst_picture;
  uint32_t num_frames;
  uint32_t num_picture_decoded;
  uint32_t rowstride;
  uint32_t bpp;

  // Internal datas
  bool_t vlibMustChangeFormat;
  vlib_stage_decoding_config_t *vlibConf;
  vp_api_io_data_t *vlibOut;
  mp4h264_config_t *mp4h264Conf;
  vp_api_io_data_t *mp4h264Out;
} video_decoder_config_t;

C_RESULT video_stage_decoder_open (video_decoder_config_t *cfg);
C_RESULT video_stage_decoder_transform (video_decoder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_stage_decoder_close (video_decoder_config_t *cfg);

extern const vp_api_stage_funcs_t video_decoding_funcs;

#endif // __VIDEO_STAGE_DECODER_H__

#endif // FFMPEG_SUPPORT
