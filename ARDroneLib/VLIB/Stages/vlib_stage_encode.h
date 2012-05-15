/**
 *  \brief    VLIB Stage
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 05/01/2009
 */

#ifndef _VLIB_STAGE_ENCODE_H_
#define _VLIB_STAGE_ENCODE_H_

#include <VP_Api/vp_api.h>

#include <VLIB/video_codec.h>

///////////////////////////////////////////////
// DECLARATIONS

enum {
 VLIB_ENCODER
};

extern const vp_api_stage_funcs_t vlib_encoding_funcs;


///////////////////////////////////////////////
// TYPEDEFS

typedef struct _vlib_stage_encoding_config_t
{
  uint32_t          subsampl;
  uint32_t			target_size;

  uint32_t          width;
  uint32_t          height;

  video_controller_t controller;
  vp_api_picture_t*  picture;

  uint32_t          current_size;

  bool_t            block_mode_enable;

  uint32_t        codec_type;

} vlib_stage_encoding_config_t;


///////////////////////////////////////////////
// FUNCTIONS

/**
 * @fn      vlib_stage_encoding_open
 * @param   vlib_stage_encoding_config_t *cfg
 * @todo    Open the encoding vlib stage
 * @return  VP_SUCCESS
 */
C_RESULT vlib_stage_encoding_open(vlib_stage_encoding_config_t *cfg);


/**
 * @fn      Transform the input com stage
 * @param   vlib_stage_encoding_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    Encode picture in config to fill output data stream
 * @return  VP_SUCCESS
 */
C_RESULT vlib_stage_encoding_transform(vlib_stage_encoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      vlib_stage_encoding_close
 * @param   vlib_stage_encoding_config_t *cfg
 * @todo    Close the encoding stage
 * @return  VP_SUCCESS
 */
C_RESULT vlib_stage_encoding_close(vlib_stage_encoding_config_t *cfg);

#endif // _vlib_STAGE_ENCODE_H_
