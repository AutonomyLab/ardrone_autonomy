#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Stages/vp_stages_configs.h>
#include <VP_Stages/vp_stages_io_console.h>
#include <VP_Stages/vp_stages_o_sdl.h>
#include <VP_Stages/vp_stages_io_com.h>
#include <VP_Stages/vp_stages_io_ffmpeg.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>


#define NB_STAGES 3


PIPELINE_HANDLE pipeline_handle;


PROTO_THREAD_ROUTINE(escaper,nomParams);
PROTO_THREAD_ROUTINE(app,nomParams);


BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(escaper,20)
  THREAD_TABLE_ENTRY(app,20)
END_THREAD_TABLE


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
  vp_stages_decoder_ffmpeg_config_t dfc;
  vp_stages_output_sdl_config_t     osc;


  vp_os_memset(&icc,0,sizeof(vp_stages_input_com_config_t));
  vp_os_memset(&com, 0, sizeof(vp_com_t));
  vp_stages_fill_default_config(UART1_COM_CONFIG, &config, sizeof(config));
  vp_stages_fill_default_config(DECODER_MPEG4_CONFIG, &dfc, sizeof(dfc));
  vp_stages_fill_default_config(SDL_DECODING_CONFIG, &osc, sizeof(osc));


  com.type                = VP_COM_SERIAL;
  icc.com                 = &com;
  icc.socket.type         = VP_COM_CLIENT;
  icc.config              = (vp_com_config_t *)&config;
  icc.buffer_size         = 1024;


  stages[0].type    = VP_API_INPUT_SOCKET;
  stages[0].cfg     = (void *)&icc;
  stages[0].funcs   = vp_stages_input_com_funcs;

  stages[1].type    = VP_API_FILTER_DECODER;
  stages[1].cfg     = (void *)&dfc;
  stages[1].funcs   = vp_stages_decoder_ffmpeg_funcs;

  stages[2].type    = VP_API_OUTPUT_SDL;
  stages[2].cfg     = (void *)&osc;
  stages[2].funcs   = vp_stages_output_sdl_funcs;

  pipeline.nb_stages = NB_STAGES;
  pipeline.stages = &stages[0];


  vp_api_open(&pipeline, &pipeline_handle);
  out.status = VP_API_STATUS_PROCESSING;
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING))
    {
      //vp_os_delay( 200 );
    }

  vp_api_close(&pipeline, &pipeline_handle);

  return EXIT_SUCCESS;
}
