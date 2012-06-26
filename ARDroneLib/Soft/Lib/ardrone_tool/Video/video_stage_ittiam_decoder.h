//
//  ittiam_stage_decode.h
//  ARDroneEngine
//
//  Created by Mykonos on 08/07/11.
//  Copyright 2011 PARROT SA. All rights reserved.
//

#ifdef ITTIAM_SUPPORT

#ifndef ITTIAM_STAGE_DECODE_H_
#define ITTIAM_STAGE_DECODE_H_

#include <VP_Api/vp_api.h>

//ITTIAM Common inlcudes
#include <datatypedef.h>
#include <it_mem.h>
#include <it_memory.h>

// H264 include
#include <ih264d_cxa8.h>

//MPEG4 include
#include <imp4d_cxa8.h>


#include <VP_Api/vp_api_picture.h>

typedef struct _ittiam_picture_ {
    enum PixelFormat format;
    uint32_t width;
    uint32_t height;
} ittiam_picture;

typedef struct _ittiam_stage_decoding_config_t {
    ittiam_picture dst_picture;
    ittiam_picture src_picture;

    uint32_t num_picture_decoded;
} ittiam_stage_decoding_config_t;


C_RESULT ittiam_stage_decoding_open(ittiam_stage_decoding_config_t *cfg);
C_RESULT ittiam_stage_decoding_transform(ittiam_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT ittiam_stage_decoding_close(ittiam_stage_decoding_config_t *cfg);

extern const vp_api_stage_funcs_t ittiam_decoding_funcs;

#endif // _ITTIAM_STAGE_DECODE_H_

#endif
