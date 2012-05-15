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

#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Os/vp_os_malloc.h>

C_RESULT
vp_stages_input_file_stage_open(vp_stages_input_file_config_t *cfg)
{
  cfg->f = fopen(cfg->name, "rb");

  if(cfg->f == NULL)
  {
    PRINT("Missing input file\n");
    return (VP_FAILURE);
  }
  return (VP_SUCCESS);
}

C_RESULT
vp_stages_input_file_stage_transform(vp_stages_input_file_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);
  uint32_t UI32_i=0;
  char c;
  uint32_t y_size, c_size;
  if( out->status == VP_API_STATUS_INIT )
  {
    out->numBuffers =  1;
    out->size = cfg->buffer_size;
    out->buffers = (int8_t **) vp_os_malloc (sizeof(int8_t *)+out->size*sizeof(int8_t));
    out->buffers[0] = (int8_t *)(out->buffers+1);
    out->indexBuffer = 0;
    // out->lineSize not used
    out->status = VP_API_STATUS_PROCESSING;
  }

  // work and update status
  if(out->size < (int32_t)cfg->buffer_size || feof(cfg->f))
  {
    if (cfg->loop)
    {
      rewind(cfg->f);
    }
    else
    {
      //vp_os_free(out->buffers);
      out->status = VP_API_STATUS_ENDED;
    }
  }
  else
  {
    if(out->status == VP_API_STATUS_PROCESSING)
      out->size = fread(out->buffers[0], sizeof(int8_t), cfg->buffer_size*sizeof(int8_t), cfg->f);

    if(out->size <= 0)
    {
      if (cfg->loop)
      {
        rewind(cfg->f);
        out->size = fread(out->buffers[0], sizeof(int8_t), cfg->buffer_size*sizeof(int8_t), cfg->f);
      }
      else
      {
        vp_os_free(out->buffers);
        out->status = VP_API_STATUS_ENDED;
      }
    }

    if(ferror(cfg->f))
    {
      PRINT("ferror\n");
      out->status = VP_API_STATUS_ERROR;
    }
  }

  vp_os_mutex_unlock(&out->lock);
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_input_file_stage_close(vp_stages_input_file_config_t *cfg)
{
  fclose(cfg->f);
  return (VP_SUCCESS);
}

C_RESULT
vp_stages_output_file_stage_open(vp_stages_output_file_config_t *cfg)
{
  VP_OS_ASSERT(cfg->flush_every_nb >= 0);
  cfg->f = fopen(cfg->name, "wb");
  return (VP_SUCCESS);
}

#define RATIO 1

C_RESULT
vp_stages_output_file_stage_transform(vp_stages_output_file_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  static int total_size = 0;

  vp_os_mutex_lock(&out->lock);

  if(in->status == VP_API_STATUS_PROCESSING && in->size > 0)
    fwrite(in->buffers[in->indexBuffer], sizeof(int8_t), in->size*sizeof(int8_t), cfg->f);

  fflush(cfg->f);
  total_size += in->size;
  out->status = in->status;
  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}

C_RESULT
vp_stages_output_file_stage_close(vp_stages_output_file_config_t *cfg)
{
  fclose(cfg->f);
  return (VP_SUCCESS);
}
