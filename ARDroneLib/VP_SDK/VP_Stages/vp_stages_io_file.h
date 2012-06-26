/**
 *  \brief    VP Stages. File stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

///////////////////////////////////////////////
// INCLUDES

#ifndef _VP_STAGES_IO_FILE_H_
#define _VP_STAGES_IO_FILE_H_
#include <VP_Api/vp_api_picture.h>

/**
 * @defgroup VP_SDK
 * @{ */

/**
 * @defgroup VP_Stages
 * @{ */

/**
 * @defgroup vp_stages_io_file input/output file stage
 * @{ */

#include <VP_Api/vp_api.h>
#include <stdio.h>

///////////////////////////////////////////////
// TYEPDEFS

typedef struct _vp_stages_input_file_config_
{
  char *name;                          // initialise with input file
  bool_t loop;                         // to be initialised to 1
  bool_t moving_picture;               // =0:one still picture  =1:several frames
  FILE *f;                             //  initialised in open stage
  int width,height;                    // for input files, the resolution to use
  int nb_buffers;
  uint8_t ** buffers;
  uint8_t * data;
  uint32_t buffer_size;                // initialised in open stage
  vp_api_picture_t vp_api_picture;     // output image
  int preload;                         // if not 0, number of bytes to load from file to memory
  int current_buffer;

} vp_stages_input_file_config_t;

typedef struct _vp_stages_output_file_config_
{
  char *name;
  FILE *f;
  uint32_t flush_every_nb;

} vp_stages_output_file_config_t;


///////////////////////////////////////////////
// FUNCTIONS

/**
 * @fn      Open the input file stage
 * @param   vp_stages_input_file_config_t *cfg
 * @todo    A COMMENTER + Verification de l'open
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_file_stage_open(vp_stages_input_file_config_t *cfg);

/**
 * @fn      Transform the input file stage
 * @param   vp_stages_input_file_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_file_stage_transform(vp_stages_input_file_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Close the input file stage
 * @param   vp_stages_input_file_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_file_stage_close(vp_stages_input_file_config_t *cfg);


/**
 * @fn      Open the output file stage
 * @param   vp_stages_output_file_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_file_stage_open(vp_stages_output_file_config_t *cfg);

/**
 * @fn      Open the output file stage
 * @param   vp_stages_output_file_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_file_stage_transform(vp_stages_output_file_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Open the output file stage
 * @param   vp_stages_output_file_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_file_stage_close(vp_stages_output_file_config_t *cfg);

// vp_stages_io_file
/** @} */
// VP_Stages
/** @} */
// VP_SDK
/** @} */

#endif // ! _VP_STAGES_IO_FILE_H_
