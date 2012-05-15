#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_configs.h>
#include <VP_Stages/vp_stages_io_console.h>
#include <VP_Stages/vp_stages_o_sdl.h>
#include <VP_Stages/vp_stages_io_com.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>


vp_stages_output_sdl_config_t     osc;

#define ACQ_WIDTH  (osc.width)
#define ACQ_HEIGHT (osc.height)

#define NB_STAGES 3


PIPELINE_HANDLE pipeline_handle;


// used by buffer_to_picture
static vp_api_picture_t picture;


PROTO_THREAD_ROUTINE(escaper,nomParams);
PROTO_THREAD_ROUTINE(app,nomParams);


BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(escaper,20)
  THREAD_TABLE_ENTRY(app,20)
END_THREAD_TABLE


C_RESULT
buffer_to_picture_open(int *cfg)
{
  picture.format = PIX_FMT_YUV420P;

  picture.width = ACQ_WIDTH;
  picture.height = ACQ_HEIGHT;
  picture.framerate = 15;

  picture.y_line_size = ACQ_WIDTH;
  picture.cb_line_size = 0; //ACQ_WIDTH/2;
  picture.cr_line_size = 0; //ACQ_WIDTH/2;

  return (SUCCESS);
}

C_RESULT
buffer_to_picture_transform(int *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);


  if(out->status == VP_API_STATUS_INIT)
    {
      out->numBuffers = 1;
      out->size = ACQ_WIDTH*ACQ_HEIGHT; //(ACQ_WIDTH*ACQ_HEIGHT*3)/2;
      out->buffers = (int8_t **)(int8_t *)&picture;
      out->indexBuffer = 0;
      out->status = VP_API_STATUS_PROCESSING;
    }

  if(out->status == VP_API_STATUS_ENDED)
    {
    }

  if(out->status == VP_API_STATUS_PROCESSING)
    {
      picture.y_buf = in->buffers[in->indexBuffer];
      picture.cb_buf = in->buffers[in->indexBuffer]+ACQ_WIDTH*ACQ_HEIGHT;
      picture.cr_buf = in->buffers[in->indexBuffer]+(ACQ_WIDTH*ACQ_HEIGHT*5)/4;
    }

  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return (SUCCESS);
}

C_RESULT
buffer_to_picture_close(int *cfg)
{
  return (SUCCESS);
}


const vp_api_stage_funcs_t buffer_to_picture_funcs =
{
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t)buffer_to_picture_open,
  (vp_api_stage_transform_t)buffer_to_picture_transform,
  (vp_api_stage_close_t)buffer_to_picture_close
};


int
main(int argc, char **argv)
{
  START_THREAD(escaper, NO_PARAM);
  START_THREAD(app, argv);

  JOIN_THREAD(escaper);
  JOIN_THREAD(app);

  return EXIT_SUCCESS;
}


PROTO_THREAD_ROUTINE(app,argv)
{
  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_com_t                          com;
  vp_stages_input_com_config_t      icc;
  vp_com_serial_config_t            config;


  vp_os_memset(&icc,0,sizeof(vp_stages_input_com_config_t));
  vp_os_memset(&com, 0, sizeof(vp_com_t));
  vp_stages_fill_default_config(UART1_COM_CONFIG, &config, sizeof(config));
  vp_stages_fill_default_config(SDL_RAW_QCIF_CONFIG, &osc, sizeof(osc));

  com.type                = VP_COM_SERIAL;
  icc.com                 = &com;
  icc.socket.type         = VP_COM_CLIENT;
  icc.config              = (vp_com_config_t *)&config;
  icc.buffer_size         = 176*144; //ACQ_WIDTH*ACQ_HEIGHT;


  stages[0].type    = VP_API_INPUT_SOCKET;
  stages[0].cfg     = (void *)&icc;
  stages[0].funcs   = vp_stages_input_com_funcs;

  stages[1].type    = VP_API_FILTER_DECODER;
  stages[1].cfg     = (void *)NULL;
  stages[1].funcs   = buffer_to_picture_funcs;

  stages[2].type    = VP_API_OUTPUT_SDL;
  stages[2].cfg     = (void *)&osc;
  stages[2].funcs   = vp_stages_output_sdl_funcs;

  pipeline.nb_stages = NB_STAGES;
  pipeline.stages = &stages[0];


  vp_api_open(&pipeline, &pipeline_handle);
  out.status = VP_API_STATUS_PROCESSING;
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING))
    {
      // no delay here
    }

  vp_api_close(&pipeline, &pipeline_handle);

  return EXIT_SUCCESS;
}
