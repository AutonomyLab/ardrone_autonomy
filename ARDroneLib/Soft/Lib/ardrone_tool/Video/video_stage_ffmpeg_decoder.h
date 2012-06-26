/*
 *  video_stage_ffmpeg_decoder.h
 *  ARDroneLib
 *
 *  Created by f.dhaeyer on 09/12/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */
#ifdef FFMPEG_SUPPORT

#ifndef _VIDEO_STAGE_FFMPEG_DECODER_H_
#define _VIDEO_STAGE_FFMPEG_DECODER_H_
#include <VP_Api/vp_api.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

typedef struct _ffmpeg_picture_
{
  enum PixelFormat format;
  uint32_t width;
  uint32_t height;
} ffmpeg_picture;

typedef struct _ffmpeg_stage_decoding_config_t
{
  ffmpeg_picture dst_picture;
  ffmpeg_picture src_picture;

  uint32_t num_picture_decoded;

  /* Alloced data pointers are saved into cfg to avoid memory leaks */  
  AVCodec *pCodecMP4;
  AVCodec *pCodecH264;
  AVCodecContext *pCodecCtxMP4;
  AVCodecContext *pCodecCtxH264;
  AVFrame *pFrame;
  AVFrame *pFrameOutput;
  uint8_t **bufferArray; // out->buffers
  uint8_t *buffer; // out->buffers[0]
  struct SwsContext *img_convert_ctx;
} ffmpeg_stage_decoding_config_t;

C_RESULT ffmpeg_stage_decoding_open(ffmpeg_stage_decoding_config_t *cfg);
C_RESULT ffmpeg_stage_decoding_transform(ffmpeg_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT ffmpeg_stage_decoding_close(ffmpeg_stage_decoding_config_t *cfg);

extern const vp_api_stage_funcs_t ffmpeg_decoding_funcs;

#endif // _VIDEO_STAGE_FFMPEG_DECODER_H_

#endif // FFMPEG_SUPPORT
