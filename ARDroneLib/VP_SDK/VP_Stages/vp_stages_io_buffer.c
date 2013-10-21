/**
 *  \brief    VP Stages. Buffer stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

///////////////////////////////////////////////
// INCLUDES


#include <VP_Stages/vp_stages_io_buffer.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>


C_RESULT
vp_stages_input_buffer_stage_open(vp_stages_input_buffer_config_t *cfg)
{
  VP_OS_ASSERT(cfg->buffer);
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_input_buffer_stage_transform(vp_stages_input_buffer_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
    {
      out->numBuffers = 1;
      out->size = 0;
      out->buffers = &cfg->buffer;
      out->indexBuffer = 0;
      cfg->remaining_size = cfg->total_size;
    }

  if(cfg->remaining_size != 0)
    {
      out->status = VP_API_STATUS_PROCESSING;
      out->buffers[out->indexBuffer] += out->size;
      out->size = (cfg->remaining_size < cfg->send_size ? cfg->remaining_size : cfg->send_size);
      cfg->remaining_size -= out->size;
    }
  else
    {
      out->status = VP_API_STATUS_ENDED;
    }

  vp_os_mutex_unlock(&out->lock);

  DEBUG_PRINT_SDK("vp_stages_input_buffer : size=%d buffer=%08X remaining=%d\n", (int)out->size, (int)out->buffers[out->indexBuffer], (int)cfg->remaining_size);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_input_buffer_stage_close(vp_stages_input_buffer_config_t *cfg)
{
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_buffer_stage_open(vp_stages_output_buffer_config_t *cfg)
{
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_buffer_stage_transform(vp_stages_output_buffer_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
    {
      out->numBuffers = 1;
      out->size = in->size;
      out->buffers = (uint8_t **)vp_os_malloc(sizeof(uint8_t *)+out->size*sizeof(uint8_t));
      out->buffers[0] = (uint8_t *)(out->buffers+1);
      out->indexBuffer = 0;
      // out->lineSize not used
      out->status = VP_API_STATUS_PROCESSING;
    }

  // \todo test
  if(in->status == VP_API_STATUS_PROCESSING)
    {
      PRINT("One frame.\n");
      vp_os_memcpy(out->buffers[0], &in->buffers[in->indexBuffer][0], in->size*sizeof(uint8_t));
    }

  if(in->status == VP_API_STATUS_ENDED)
  {
	  vp_os_free(out->buffers);
	  out->buffers = NULL;
  }

  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_buffer_stage_close(vp_stages_output_buffer_config_t *cfg)
{
  return (VP_SUCCESS);
}
