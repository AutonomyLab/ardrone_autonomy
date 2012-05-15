/**
 *  \brief    VP Stages. Buffer stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

#ifndef _VP_STAGES_IO_BUFFER_H_
#define _VP_STAGES_IO_BUFFER_H_

/** 
 * @defgroup VP_SDK
 * @{ */

/** 
 * @defgroup VP_Stages
 * @{ */

/** 
 * @defgroup vp_stages_io_buffer input/output buffers stage
 * @{ */


///////////////////////////////////////////////
// INCLUDE

#include <VP_Api/vp_api.h>


///////////////////////////////////////////////
// TYPEDEFS

/**
 * \typedef enumerate all the camera resolution supported in camif
 */

typedef struct _vp_stages_input_buffer_config_
{
  int8_t  *buffer;
  uint32_t total_size;
  uint32_t send_size;

  // private
  uint32_t remaining_size;
} vp_stages_input_buffer_config_t;

typedef struct _vp_stages_output_buffer_config_ vp_stages_output_buffer_config_t;



///////////////////////////////////////////////
// FUNCTIONS


/**
 * @fn      Open the input buffer stage
 * @param   vp_stages_input_buffer_config_t *cfg : configuration of buffer parameters
 * @todo    modify the return
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_buffer_stage_open(vp_stages_input_buffer_config_t *cfg);

/**
 * @fn      Transform the buffer input stage
 * @brief   Select the right indexbuffer of cfg and increment the size and the data pointer to the output
 * @param   vp_stages_input_buffer_config_t *cfg : data cfg buffer is send to out buffer
 * @param   vp_api_io_data_t *in : not used here
 * @param   vp_api_io_data_t *out : used to send the data
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_stages_input_buffer_stage_transform(vp_stages_input_buffer_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Close the input buffer stage
 * @brief   Nothing to do
 * @param   vp_stages_input_buffer_config_t *cfg
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_buffer_stage_close(vp_stages_input_buffer_config_t *cfg);

/**
 * @fn      Open the output buffer stage
 * @brief   Nothing to do
 * @param   vp_stages_input_buffer_config_t *cfg
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_buffer_stage_open(vp_stages_output_buffer_config_t *cfg);

/**
 * @fn      Transform the output buffer stage
 * @param   vp_stages_input_buffer_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_stages_output_buffer_stage_transform(vp_stages_output_buffer_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Close the output buffer stage
 * @brief   Nothing to do
 * @param   vp_stages_input_buffer_config_t *cfg
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_buffer_stage_close(vp_stages_output_buffer_config_t *cfg);

// vp_stages_io_buffer
/** @} */
// VP_Stages
/** @} */
// VP_SDK
/** @} */

#endif // ! _VP_STAGES_IO_BUFFER_H_
