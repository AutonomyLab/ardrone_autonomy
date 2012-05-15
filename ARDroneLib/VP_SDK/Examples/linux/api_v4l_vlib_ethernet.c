#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Stages/vp_stages_configs.h>
#include <VP_Stages/vp_stages_i_v4l.h>
#include <VP_Stages/vp_stages_io_com.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VLIB/Stages/vlib_stage_encode.h>


#define VIDEO_DEVICE_PATH "/dev/video0"

#undef BLOCK_MODE
#undef ACQ_WIDTH
#undef ACQ_HEIGHT

#define ACQ_WIDTH 320
#define ACQ_HEIGHT 240

#define COM_VIDEO()             wired_com()
#define COM_CONFIG_VIDEO()      wired_config()
#define COM_CONNECTION_VIDEO()  wired_connection()


PROTO_THREAD_ROUTINE(app,nomParams);

BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(app,20)
END_THREAD_TABLE


#define NB_STAGES_MAX 10


static vp_stages_input_v4l_config_t         ivc;
static vp_stages_buffer_to_picture_config_t bpc;
static vlib_stage_encoding_config_t         vec;
static vp_stages_output_com_config_t        occ;

static PIPELINE_HANDLE pipeline_handle;

static vp_api_picture_t picture;


vp_com_t* wired_com(void)
{
  static vp_com_t com = {
    VP_COM_WIRED,
    FALSE,
    0,
    { { 0 } },
    NULL,
    NULL,
    0,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
  };

  return &com;
}

vp_com_config_t* wired_config(void)
{
  static vp_com_wired_config_t config = {
    "eth0",
    "172.20.22.245",
    "255.255.255.0",
    "172.20.22.255"
  };

  return (vp_com_config_t*) &config;
}

vp_com_connection_t* wired_connection(void)
{
  static vp_com_wired_connection_t connection = {
    0
  };

  return (vp_com_connection_t*) (void*) &connection;
}


int main(int argc, char **argv)
{
  if(argc < 2)
    {
      PRINT("You need to specify video device path\n");
      PRINT("Ex : %s /dev/video0\n", argv[0]);
      return EXIT_FAILURE;
    }

  START_THREAD(app, argv);
  JOIN_THREAD(app);

  return EXIT_SUCCESS;
}


PROTO_THREAD_ROUTINE(app,argv)
{
  vp_api_io_pipeline_t pipeline;
  vp_api_io_data_t out;
  vp_api_io_stage_t stages[NB_STAGES_MAX];

  uint16_t time1,time2;
  uint16_t i;

  time1 = clock();

  vp_os_memset(&ivc,0,sizeof(ivc));
  vp_os_memset(&bpc,0,sizeof(bpc));
  vp_os_memset(&vec,0,sizeof(vec));
  vp_os_memset(&occ,0,sizeof(occ));

  ivc.device = ((char**)argv)[1];
  ivc.width = ACQ_WIDTH;
  ivc.height = ACQ_HEIGHT;
  ivc.vp_api_picture = &picture;

  vec.width                               = ACQ_WIDTH;
  vec.height                              = ACQ_HEIGHT;
  vec.block_mode_enable                   = FALSE;
  vec.picture                             = &picture;

  occ.com                           = COM_VIDEO();
  occ.config                        = COM_CONFIG_VIDEO();
  occ.connection                    = COM_CONNECTION_VIDEO();
  occ.socket.type                   = VP_COM_SERVER;
  occ.socket.protocol               = VP_COM_TCP;
  occ.socket.port                   = 5555;
  occ.buffer_size                   = 1500;

  pipeline.nb_stages = 0;

  stages[pipeline.nb_stages].type    = VP_API_INPUT_FILE;
  stages[pipeline.nb_stages].cfg     = (void *)&ivc;
  stages[pipeline.nb_stages++].funcs = vp_stages_input_v4l_funcs;
  
  stages[pipeline.nb_stages].type    = VP_API_FILTER_ENCODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vec;
  stages[pipeline.nb_stages++].funcs = vlib_encoding_funcs;

  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SOCKET;
  stages[pipeline.nb_stages].cfg     = (void *)&occ;
  stages[pipeline.nb_stages++].funcs = vp_stages_output_com_funcs;

  pipeline.stages = &stages[0];

  vp_api_open(&pipeline, &pipeline_handle);
 
  out.status = VP_API_STATUS_PROCESSING;

  /////////////////////////////////////////////////////////////////////////////////////////
  i = 0;
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING))
    {
      i++;
      PRINT("image %d \n",i);
      //vp_os_delay(50);
    }
  /////////////////////////////////////////////////////////////////////////////////////////

  time2 = clock();
  printf("temps ecoule : %d \n",time2-time1);
    
  vp_api_close(&pipeline, &pipeline_handle);

  return EXIT_SUCCESS;
}

