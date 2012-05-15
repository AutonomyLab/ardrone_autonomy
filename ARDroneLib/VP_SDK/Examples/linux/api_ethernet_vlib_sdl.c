#include <stdlib.h>
#include <ctype.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Stages/vp_stages_configs.h>
#include <VP_Stages/vp_stages_io_console.h>
#include <VP_Stages/vp_stages_o_sdl.h>
#include <VP_Stages/vp_stages_io_console.h>
#include <VP_Stages/vp_stages_io_com.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VLIB/Stages/vlib_stage_decode.h>


#define SERVER_HOST "172.20.22.245"

#undef BLOCK_MODE
#undef ACQ_WIDTH
#undef ACQ_HEIGHT

#define NB_STAGES_MAX 10

#define ACQ_WIDTH     320
#define ACQ_HEIGHT    240


#define COM_VIDEO()             wired_com()
#define COM_CONFIG_VIDEO()      wired_config()
#define COM_CONNECTION_VIDEO()  wired_connection()
#define COM_CONFIG_SOCKET_VIDEO(socket, type, opt, serverhost)  wired_config_socket(socket, type, opt, serverhost)


PROTO_THREAD_ROUTINE(escaper,nomParams);
PROTO_THREAD_ROUTINE(app,nomParams);

BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(escaper,20)
  THREAD_TABLE_ENTRY(app,10)
END_THREAD_TABLE


static vp_stages_input_com_config_t  icc;
static vlib_stage_decoding_config_t  vdc;
static vp_stages_output_sdl_config_t osc;

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
    "eth1",
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

void wired_config_socket(vp_com_socket_t* socket, VP_COM_SOCKET_TYPE type, int32_t port, const char* serverhost)
{
  vp_os_memset(socket, 0, sizeof(vp_com_socket_t));

  socket->type           = type;
  socket->protocol       = VP_COM_TCP;
  socket->port           = port;

  if(serverhost && socket->type == VP_COM_CLIENT)
    strcpy(socket->serverHost, serverhost);
}


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
  vp_api_io_stage_t       stages[NB_STAGES_MAX];

  vp_os_memset( &icc,     0, sizeof(icc) );
  vp_os_memset( &vdc,     0, sizeof(vdc) );
  vp_os_memset( &osc,     0, sizeof(osc) );

  icc.com               = COM_VIDEO();
  icc.buffer_size       = 1024;
  icc.socket.protocol   = VP_COM_TCP;
  COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, 5555, SERVER_HOST);

  /// Picture configuration
  picture.format        = PIX_FMT_YUV420P;
  picture.width         = ACQ_WIDTH;
  picture.height        = ACQ_HEIGHT;
  picture.framerate     = 30;
  picture.y_buf         = vp_os_malloc( ACQ_WIDTH*ACQ_HEIGHT );
  picture.cr_buf        = vp_os_malloc( ACQ_WIDTH*ACQ_HEIGHT/4 );
  picture.cb_buf        = vp_os_malloc( ACQ_WIDTH*ACQ_HEIGHT/4 );
  picture.y_line_size   = ACQ_WIDTH;
  picture.cb_line_size  = ACQ_WIDTH / 2;
  picture.cr_line_size  = ACQ_WIDTH / 2;
  picture.y_pad         = 0;
  picture.c_pad         = 0;

  vdc.width           = ACQ_WIDTH;
  vdc.height          = ACQ_HEIGHT;
  vdc.picture         = &picture;
  vdc.luma_only       = FALSE;
  vdc.block_mode_enable = FALSE;

  osc.width           = ACQ_WIDTH;
  osc.height          = ACQ_HEIGHT;
  osc.bpp             = 16;
  osc.window_width    = ACQ_WIDTH;
  osc.window_height   = ACQ_HEIGHT;
  osc.pic_width       = ACQ_WIDTH;
  osc.pic_height      = ACQ_HEIGHT;
  osc.y_size          = ACQ_WIDTH*ACQ_HEIGHT;
  osc.c_size          = (ACQ_WIDTH*ACQ_HEIGHT) >> 2;

  pipeline.nb_stages                 = 0;

  stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
  stages[pipeline.nb_stages].cfg     = (void *)&icc;
  stages[pipeline.nb_stages++].funcs = vp_stages_input_com_funcs;

  stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vdc;
  stages[pipeline.nb_stages++].funcs = vlib_decoding_funcs;

  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SDL;
  stages[pipeline.nb_stages].cfg     = (void *)&osc;
  stages[pipeline.nb_stages++].funcs = vp_stages_output_sdl_funcs;

  pipeline.stages                    = &stages[0];

  vp_api_open(&pipeline, &pipeline_handle);
  out.status = VP_API_STATUS_PROCESSING;
  while(SUCCEED(vp_api_run(&pipeline, &out)) && (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING))
    {
    }

  vp_api_close(&pipeline, &pipeline_handle);

  return EXIT_SUCCESS;
}

