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

#include <VP_Stages/vp_stages_io_console.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_print.h>


C_RESULT
vp_stages_output_console_stage_open(void *cfg)
{
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_console_stage_transform(void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  out->buffers = in->buffers;
  out->size = in->size;
  if(in->status == VP_API_STATUS_PROCESSING)
  {
    int i;

    for(i = 0;i < out->size;i++)
      PRINT("%c",in->buffers[in->indexBuffer][i]);
  }

  out->status = in->status;

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_console_stage_close(void *cfg)
{
  return (VP_SUCCESS);
}
