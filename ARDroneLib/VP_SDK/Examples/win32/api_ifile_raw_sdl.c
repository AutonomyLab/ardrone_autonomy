#include <stdlib.h>
#include <ctype.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_configs.h>
#include <VP_Stages/vp_stages_io_console.h>
#include <VP_Stages/vp_stages_o_sdl.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>

#include <MJPEG/mjpeg.h>
#include <MJPEG/dct.h>

#define ACQ_WIDTH  (176*2)
#define ACQ_HEIGHT (144*2)

#define NB_STAGES 3


PIPELINE_HANDLE pipeline_handle;

PROTO_THREAD_ROUTINE(escaper, nomParams);
PROTO_THREAD_ROUTINE(app, nomParams);

BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(escaper, 20)
  THREAD_TABLE_ENTRY(app, 20)
END_THREAD_TABLE


///*******************************************************************************************************************///

typedef struct _buffer_to_picture_config_t
{
  vp_api_picture_t* picture;

} buffer_to_picture_config_t;

C_RESULT
buffer_to_picture_open(buffer_to_picture_config_t *cfg)
{
  cfg->picture->format        = PIX_FMT_YUV420P;

  cfg->picture->width         = ACQ_WIDTH;
  cfg->picture->height        = ACQ_HEIGHT;
  cfg->picture->framerate     = 15;

  cfg->picture->y_line_size   = ACQ_WIDTH;
  cfg->picture->cb_line_size  = ACQ_WIDTH / 2;
  cfg->picture->cr_line_size  = ACQ_WIDTH / 2;

  cfg->picture->y_pad         = 0;
  cfg->picture->c_pad         = 0;

  return (SUCCESS);
}

C_RESULT
buffer_to_picture_transform(buffer_to_picture_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);


  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;
    out->size         = (ACQ_WIDTH*ACQ_HEIGHT*3)/2;
    out->buffers      = (int8_t **) cfg->picture;
    out->indexBuffer  = 0;
    out->status       = VP_API_STATUS_PROCESSING;
  }

  if(out->status == VP_API_STATUS_ENDED)
  {
  }

  if(out->status == VP_API_STATUS_PROCESSING)
  {
    cfg->picture->y_buf   = in->buffers[0];
    cfg->picture->cb_buf  = in->buffers[0] + ACQ_WIDTH*ACQ_HEIGHT;
    cfg->picture->cr_buf  = in->buffers[0] + ACQ_WIDTH*ACQ_HEIGHT + ACQ_WIDTH*ACQ_HEIGHT/4;
  }

  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return (SUCCESS);
}

C_RESULT
buffer_to_picture_close(buffer_to_picture_config_t *cfg)
{
  return (SUCCESS);
}

const vp_api_stage_funcs_t buffer_to_picture_funcs =
{
  NULL,
  (vp_api_stage_open_t)buffer_to_picture_open,
  (vp_api_stage_transform_t)buffer_to_picture_transform,
  (vp_api_stage_close_t)buffer_to_picture_close
};


int
main(int argc, char **argv)
{
  if(argc != 2)
  {
    PRINT("You must specify a filename.\n");
    return EXIT_FAILURE;
  }

  START_THREAD(escaper, NO_PARAM);
  START_THREAD(app, argv);

  JOIN_THREAD(escaper);
  JOIN_THREAD(app);

  return EXIT_SUCCESS;
}


PROTO_THREAD_ROUTINE(app,argv)
{
  vp_api_picture_t picture;

  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_stages_input_file_config_t     ifc;
  vp_stages_output_sdl_config_t     osc;
  buffer_to_picture_config_t        bpc;

  vp_os_memset(&ifc,0,sizeof(vp_stages_input_file_config_t));

  ifc.name            = ((char**)argv)[1];
  ifc.buffer_size     = (ACQ_WIDTH*ACQ_HEIGHT*3)/2;

  osc.width           = ACQ_WIDTH;
  osc.height          = ACQ_HEIGHT;
  osc.bpp             = 16;
  osc.window_width    = ACQ_WIDTH;
  osc.window_height   = ACQ_HEIGHT;
  osc.pic_width       = ACQ_WIDTH;
  osc.pic_height      = ACQ_HEIGHT;
  osc.y_size          = ACQ_WIDTH*ACQ_HEIGHT;
  osc.c_size          = (ACQ_WIDTH*ACQ_HEIGHT) >> 2;

  bpc.picture         = &picture;

  stages[0].type      = VP_API_INPUT_FILE;
  stages[0].cfg       = (void *)&ifc;
  stages[0].funcs     = vp_stages_input_file_funcs;

  stages[1].type      = VP_API_FILTER_DECODER;
  stages[1].cfg       = (void *)&bpc;
  stages[1].funcs     = buffer_to_picture_funcs;

  stages[2].type      = VP_API_OUTPUT_SDL;
  stages[2].cfg       = (void *)&osc;
  stages[2].funcs     = vp_stages_output_sdl_funcs;

  pipeline.nb_stages  = NB_STAGES;
  pipeline.stages     = &stages[0];

  vp_api_open(&pipeline, &pipeline_handle);
  out.status = VP_API_STATUS_PROCESSING;
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING))
  {
    vp_os_delay( 100 );
  }

  vp_api_close(&pipeline, &pipeline_handle);

  return EXIT_SUCCESS;
}
