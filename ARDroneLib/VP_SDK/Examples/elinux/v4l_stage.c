#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Stages/vp_stages_configs.h>
#include <VP_Stages/vp_stages_V4L2_i_camif.h>
#include <VP_Stages/vp_stages_io_com.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VLIB/Stages/vlib_stage_encode.h>
#include <VP_Stages/blockline_to_buffer_stage.h>


#include "Video/ov7720.h"
#include "Video/cresyn.h"

#include <sys/time.h>
#include <unistd.h>

 
//#define ACQ_WIDTH 176
//#define ACQ_HEIGHT 144

#define ACQ_WIDTH 640
#define ACQ_HEIGHT 480

#define CRESYN_CAM
//#define OV_CAM
//#define BLOCK_MODE
//#define BUFFER
//#define ENCODE 
//#define FILE_OUTPUT
//#define ETHERNET_OUTPUT



#define NB_STAGES_MAX 10


static vp_stages_input_camif_config_t  ivc;
static vlib_stage_encoding_config_t  vec;
static vp_stages_output_file_config_t  ofc;
static vp_stages_output_com_config_t   occ;
static blockline_to_buffer_config_t bbc;

static PIPELINE_HANDLE pipeline_handle;


C_RESULT
buffer_to_picture_close(int *cfg)
{
  return (SUCCESS);
}

const vp_api_stage_funcs_t V4L2_funcs =
{
  (vp_api_stage_handle_msg_t)vp_stages_input_camif_stage_handle_msg,
  (vp_api_stage_open_t)vp_stages_input_camif_stage_open,
  (vp_api_stage_transform_t)vp_stages_input_camif_stage_transform,
  (vp_api_stage_close_t)vp_stages_input_camif_stage_close
};

//// COM
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
    "172.20.2.190",
    "255.255.255.0",
    "172.20.2.255"
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
  vp_api_picture_t  video_picture;
  vp_api_picture_t  buffer_blockline;
#ifdef BLOCK_MODE
  vp_api_picture_t  video_blockline;
#endif
  vp_api_io_pipeline_t pipeline;
  vp_api_io_data_t out;
  vp_api_io_stage_t stages[NB_STAGES_MAX];

  vp_os_memset(&ivc,0,sizeof(ivc));
  vp_os_memset(&ofc,0,sizeof(ofc));
  vp_os_memset(&occ,0,sizeof(occ));


  // CAMIF config
  ivc.camera = "/dev/video1";
  ivc.i2c = "/dev/i2c-0";
#ifdef OV_CAM
  ivc.camera_configure = &camera_configure_ov77xx;
#else
#ifdef CRESYN_CAM
  ivc.camera_configure = &camera_configure_cresyn;
#else  
#error no cam selected
#endif
#endif

  ivc.resolution = VGA;
  ivc.nb_buffers = 8;
  ivc.format                              = V4L2_PIX_FMT_YUV420;
  ivc.y_pad                               = 0;
  ivc.c_pad                               = 0;
  ivc.offset_delta                        = 0;
#ifdef BLOCK_MODE
  ivc.vp_api_blockline                    = &video_blockline;
#endif
  ivc.vp_api_picture                      = &video_picture;
  ivc.use_chrominances                    = 1;
  ivc.framerate				  = 15;

  // BUFFER
#ifdef BLOCK_MODE
  bbc.picture                    = &video_blockline;
#endif

  // ENCODER
  vec.width                               = 320;
  vec.height                              = 240;
#ifdef BLOCK_MODE
  vec.block_mode_enable                   = TRUE;
#else
  vec.block_mode_enable                   = FALSE;
#endif
  vec.picture                             = &video_picture;
 
  // OUTPUT FILE config
  ofc.name = "/tmp/out.yuv";
  
  // COM CONFIG
  occ.com                            = wired_com();
  occ.config                         = wired_config();
  occ.connection                     = wired_connection();
  occ.socket.type                    = VP_COM_SERVER;
  occ.socket.protocol                = VP_COM_TCP;
  occ.socket.port                    = 5555;
  occ.buffer_size                    = 640*480*3/2;

  // build pipeline
  pipeline.nb_stages = 0;

  stages[pipeline.nb_stages].type    = VP_API_INPUT_CAMIF;
  stages[pipeline.nb_stages].cfg     = (void *)&ivc;
  stages[pipeline.nb_stages++].funcs = V4L2_funcs;

#ifdef BLOCK_MODE
#ifdef BUFFER
  stages[pipeline.nb_stages].type    = VP_API_INPUT_CAMIF;
  stages[pipeline.nb_stages].cfg     = (void *)&bbc;
  stages[pipeline.nb_stages++].funcs =  blockline_to_buffer_funcs;
#endif
#endif

#ifdef ENCODE 
  stages[pipeline.nb_stages].type    = VP_API_FILTER_ENCODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vec;
  stages[pipeline.nb_stages++].funcs = vlib_encoding_funcs;
#endif

#ifdef FILE_OUTPUT
  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_FILE;
  stages[pipeline.nb_stages].cfg     = (void *)&ofc;
  stages[pipeline.nb_stages++].funcs = vp_stages_output_file_funcs;
#endif

#ifdef ETHERNET_OUTPUT
  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SOCKET;
  stages[pipeline.nb_stages].cfg     = (void *)&occ;
  stages[pipeline.nb_stages++].funcs = vp_stages_output_com_funcs;
#endif

  pipeline.stages = &stages[0];

  // launch pipelines
  vp_api_open(&pipeline, &pipeline_handle);
  
  out.status = VP_API_STATUS_PROCESSING;

  /////////////////////////////////////////////////////////////////////////////////////////
  
  uint16_t i = 0;

  struct timeval time1,time2;

  printf("Pipeline launched....\n");
  gettimeofday(&time1,NULL);
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING))
    {
      i++;
      //printf("image %d \n",i);
      if (i>50)
        break;
      //vp_os_delay(20);
    }   
  /////////////////////////////////////////////////////////////////////////////////////////
 
  gettimeofday(&time2,NULL);
  int seconde = time2.tv_sec-time1.tv_sec;
  int ms = time2.tv_usec-time1.tv_usec;
  if (ms<0)
  {
     seconde--;
     ms += 1000000;
  }
  ms = ms/1000;
  float fps = ((float)i*1000)/(float)(seconde*1000+ms);
  printf("temps ecoule : %d.%d / nombre image : %d average fps : %f\n",seconde,ms,i,fps);
  vp_api_close(&pipeline, &pipeline_handle);

  return EXIT_SUCCESS;
}

