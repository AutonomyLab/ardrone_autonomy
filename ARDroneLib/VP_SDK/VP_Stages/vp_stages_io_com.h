/**
 *  \brief    VP Stages. Com stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

#ifndef _VP_STAGES_IO_COM_H_
#define _VP_STAGES_IO_COM_H_

/** 
 * @defgroup VP_SDK
 * @{ */

/** 
 * @defgroup VP_Stages
 * @{ */

/** 
 * @defgroup vp_stages_io_com input/output communcation stage
 * @{ */


#include <VP_Api/vp_api.h>
#include <VP_Com/vp_com.h>

///////////////////////////////////////////////
// TYPEDEFS

typedef struct _vp_stages_input_com_config_
{
  vp_com_t*             com;

  vp_com_config_t*      config;
  vp_com_connection_t*  connection;

  vp_com_socket_t       socket;
  VP_COM_SOCKET_OPTIONS sockopt;

  uint32_t              buffer_size;

  // Private Datas
  vp_com_socket_t       socket_client; // Socket used for read / write
  Read                  read;

} vp_stages_input_com_config_t;

typedef struct _vp_stages_output_com_config_
{
  vp_com_t*             com;

  vp_com_config_t*      config;
  vp_com_connection_t*  connection;

  vp_com_socket_t       socket;
  VP_COM_SOCKET_OPTIONS sockopt;

  uint32_t              buffer_size;

  // Private Datas
  vp_com_socket_t       socket_client; // Socket used for read / write
  Write                 write;

} vp_stages_output_com_config_t;


///////////////////////////////////////////////
// FUNCTIONS

/**
 * @fn      Open the input com stage
 * @param   vp_stages_input_com_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_com_stage_open(vp_stages_input_com_config_t *cfg);


/**
 * @fn      Transform the input com stage
 * @param   vp_stages_input_com_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_com_stage_transform(vp_stages_input_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Close the input com stage
 * @param   vp_stages_input_com_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_input_com_stage_close(vp_stages_input_com_config_t *cfg);


/**
 * @fn      Open the output com stage
 * @param   vp_stages_output_com_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_com_stage_open(vp_stages_output_com_config_t *cfg);

/**
 * @fn      Transform the output com stage
 * @param   vp_stages_output_com_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_com_stage_transform(vp_stages_output_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Close the output com stage
 * @param   vp_stages_output_com_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_com_stage_close(vp_stages_output_com_config_t *cfg);

// vp_stages_io_com
/** @} */
// VP_Stages
/** @} */
// VP_SDK
/** @} */


#endif // ! _VP_STAGES_IO_COM_H_
