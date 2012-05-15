#include <stdlib.h>
#include <ctype.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Os/vp_os_print.h>


#define NB_STAGES 3


static PIPELINE_HANDLE pipeline_handle;


C_RESULT
my_transform_open(void *cfg)
{
  return (SUCCESS);
}

C_RESULT
my_transform_transform(void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  int i;

  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
    {
      out->numBuffers = 1;
      out->size = in->size; // to be cleaned ...
      out->buffers = (int8_t **)malloc(sizeof(int8_t *)+out->size*sizeof(int8_t));
      out->buffers[0] = (int8_t *)(out->buffers+1);
      out->indexBuffer = 0;
      // out->lineSize not used
      out->status = VP_API_STATUS_PROCESSING;
    }

  if(in->status == VP_API_STATUS_ENDED)
    {
      free(out->buffers);
    }
  else if(in->status == VP_API_STATUS_PROCESSING)
    {
      // uppercase
      for(i = 0 ; i < in->size ; i++)
	{
	  out->buffers[0][i] = toupper(in->buffers[in->indexBuffer][i]);
	}
    }

  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return (SUCCESS);
}

C_RESULT
my_transform_close(void *cfg)
{
  return (SUCCESS);
}


const vp_api_stage_funcs_t my_transform_funcs =
{
  NULL,
  (vp_api_stage_open_t)my_transform_open,
  (vp_api_stage_transform_t)my_transform_transform,
  (vp_api_stage_close_t)my_transform_close
};


int
main(int argc, char **argv)
{
  vp_api_io_pipeline_t pipeline;
  vp_api_io_data_t out;
  vp_api_io_stage_t stages[NB_STAGES];

  vp_stages_input_file_config_t ifc;
  vp_stages_output_file_config_t ofc;

  ifc.name = "toto.in";
  ifc.buffer_size = 512;

  ofc.name = "toto.out";

  stages[0].type = VP_API_INPUT_FILE;
  stages[0].cfg = (void *)&ifc;
  stages[0].funcs = vp_stages_input_file_funcs;

  stages[1].type = VP_API_FILTER_ENCODER;
  stages[1].cfg = NULL;
  stages[1].funcs = my_transform_funcs;

  stages[2].type = VP_API_OUTPUT_FILE;
  stages[2].cfg = (void *)&ofc;
  stages[2].funcs = vp_stages_output_file_funcs;

  pipeline.nb_stages = NB_STAGES;
  pipeline.stages = &stages[0];

  vp_api_open(&pipeline, &pipeline_handle);

  out.status = VP_API_STATUS_PROCESSING;
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING));

  // \todo tests

  vp_api_close(&pipeline, &pipeline_handle);
  return EXIT_SUCCESS;
}
