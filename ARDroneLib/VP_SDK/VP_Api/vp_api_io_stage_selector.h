/**
 *  \brief    VP Api. Stage selector for a multi stage
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  1.0
 *  \date     first release 21/03/2007
 */

#ifndef _VP_API_IO_STAGE_SELECTOR_H_
#define _VP_API_IO_STAGE_SELECTOR_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_io_multi_stage.h>

///////////////////////////////////////////////
// TYPEDEFS

typedef struct _vp_api_io_stage_selector_
{
  int32_t* activ_stage; // Pointer on the activ_stage of targeted multi_stage
} vp_api_io_stage_selector_t;


///////////////////////////////////////////////
// FUNCTIONS

//
// No implementation!
// If someone wants to use a stage selector, he needs to implement it,
// because it depends on its needs.
//

#endif // _VP_API_IO_STAGE_SELECTOR_H_
