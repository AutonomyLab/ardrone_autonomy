/**
 *  \brief    VP Stages. Console stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

///////////////////////////////////////////////
// INCLUDES

#ifndef _VP_STAGES_IO_CONSOLE_H_
#define _VP_STAGES_IO_CONSOLE_H_

/** 
 * @defgroup VP_SDK
 * @{ */

/** 
 * @defgroup VP_Stages
 * @{ */

/** 
 * @defgroup vp_stages_io_console input/output console stage
 * @{ */


///////////////////////////////////////////////
// INCLUDE

#include <VP_Api/vp_api.h>


///////////////////////////////////////////////
// FUNCTIONS



/**
 * @fn      Open the output console stage
 * @brief   Nothing to do
 * @param   void *cfg
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_console_stage_open(void *cfg);


/**
 * @fn      Open the output console stage
 * @brief   Nothing to do
 * @param   void *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_console_stage_transform(void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Close the output console stage
 * @brief   Nothing to do
 * @param   void *cfg
 * @return  VP_SUCCESS
 */
C_RESULT
vp_stages_output_console_stage_close(void *cfg);

// vp_stages_io_console
/** @} */
// VP_Stages
/** @} */
// VP_SDK
/** @} */

#endif // _VP_STAGES_IO_CONSOLE_H_
