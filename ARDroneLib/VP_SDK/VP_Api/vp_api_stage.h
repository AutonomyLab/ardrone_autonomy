/**
 *  @file     vp_api_stage.h
 *  @brief    VP Api. Stages definition
 *  @author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  @author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  @author   Thomas Landais <thomas.landais@parrot.fr>
 *  @author   Julien Floret <julien.floret.ext@parrot.com>
 *  @version  2.0
 *  @date     first release 16/03/2007
 *  @date     modification  19/03/2007
 */

#ifndef _VP_API_STAGE_H_
#define _VP_API_STAGE_H_

///////////////////////////////////////////////
// INCLUDES

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_config.h>
#include <VP_Api/vp_api_supervisor.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct _vp_api_io_data_;


///////////////////////////////////////////////
// TYPEDEFS


/**
 * @typedef C_RESULT (*vp_api_stage_handle_msg_t)(void *cfg, PIPELINE_MSG msg_id, void *callback, void *param)
Functions prototypes to open, close and update a stage
 */
typedef C_RESULT
(*vp_api_stage_handle_msg_t)(void *cfg, PIPELINE_MSG msg_id, void *callback, void *param);

typedef C_RESULT
(*vp_api_stage_open_t)(void *cfg);

typedef C_RESULT
(*vp_api_stage_transform_t)(void *cfg, struct _vp_api_io_data_ *in, struct _vp_api_io_data_ *out);

typedef C_RESULT
(*vp_api_stage_close_t)(void *cfg);


/**
 * @struct  _vp_api_stage_funcs_
 * @brief   A structure that contains all functions pointers to handle a stage
 */
typedef struct _vp_api_stage_funcs_
{
  vp_api_stage_handle_msg_t   handle_msg;
  vp_api_stage_open_t         open;
  vp_api_stage_transform_t    transform;
  vp_api_stage_close_t        close;
} vp_api_stage_funcs_t;


/**
 * @brief Definition of ready to use stages for vp products
 */

C_RESULT vp_api_stage_empty_transform (void *cfg, struct _vp_api_io_data_ *in, struct _vp_api_io_data_ *out);

extern const vp_api_stage_funcs_t vp_stages_input_file_funcs;
extern const vp_api_stage_funcs_t vp_stages_output_file_funcs;

extern const vp_api_stage_funcs_t vp_stages_input_buffer_funcs;
extern const vp_api_stage_funcs_t vp_stages_output_buffer_funcs;

extern const vp_api_stage_funcs_t vp_stages_frame_pipe_sender_funcs;
extern const vp_api_stage_funcs_t vp_stages_frame_pipe_receiver_funcs;
extern const vp_api_stage_funcs_t vp_stages_frame_pipe_fetch_funcs;

extern const vp_api_stage_funcs_t vp_stages_video_mixer_funcs;

extern const vp_api_stage_funcs_t vp_stages_buffer_to_picture_funcs;
extern const vp_api_stage_funcs_t vp_stages_picture_to_buffer_funcs;

extern const vp_api_stage_funcs_t vp_stages_decoder_filter_funcs;
extern const vp_api_stage_funcs_t vp_stages_decoder_ffmpeg_funcs;

extern const vp_api_stage_funcs_t vp_stages_input_com_funcs;
extern const vp_api_stage_funcs_t vp_stages_output_com_funcs;

extern const vp_api_stage_funcs_t vp_stages_output_console_funcs;

extern const vp_api_stage_funcs_t vp_stages_output_lcd_funcs;

extern const vp_api_stage_funcs_t vp_stages_yuv2rgb_funcs;

#ifdef USE_PVSP
extern const vp_api_stage_funcs_t vp_stages_pvsp_funcs;
#endif // USE_PVSP

#ifdef __cplusplus
}
#endif


#endif // ! _VP_API_STAGE_H_
