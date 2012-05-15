/**
 *  \brief    VP Api. Composite Stage Declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \todo     This stage need to be tested
 *  \version  1.0
 *  \date     first release 21/03/2007
 */

#ifndef _VP_API_IO_MULTI_STAGE_H_
#define _VP_API_IO_MULTI_STAGE_H_

#include <VP_Api/vp_api.h>


///////////////////////////////////////////////
// DEFINES

#define VP_API_EXECUTE_ALL_STAGES -1
#define VP_API_EXECUTE_NO_STAGE   -2

///////////////////////////////////////////////
// TYPEDEFS

typedef struct _vp_api_io_multi_stage_config_
{
  int32_t             activ_stage; // if activ_stage == -1 then all stages are executed
  uint32_t            nb_stages;
  vp_api_io_stage_t*  stages;
} vp_api_io_multi_stage_config_t;


///////////////////////////////////////////////
// FUNCTIONS

/**
 * @fn      Open the multi stage
 * @param   vp_api_io_multi_stage_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_api_multi_stage_open(vp_api_io_multi_stage_config_t *cfg);


/**
 * @fn      Transform the multi stage
 * @param   vp_api_io_multi_stage_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_api_multi_stage_transform(vp_api_io_multi_stage_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);


/**
 * @fn      Close the multi stage
 * @param   vp_api_io_multi_stage_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_api_multi_stage_close(vp_api_io_multi_stage_config_t *cfg);


#endif // _VP_API_IO_MULTI_STAGE_H_
