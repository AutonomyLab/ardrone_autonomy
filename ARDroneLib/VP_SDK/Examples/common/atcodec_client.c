
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifndef AT_MESSAGES_HEADER
# define AT_MESSAGES_HEADER <ATcodec/ATcodec_Messages_ex.h>
#endif // > AT_MESSAGES_HEADER


#include <ATcodec/ATcodec_api.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Com/vp_com.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Os/vp_os_thread.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <Examples/common/common.h>
#include <Examples/common/atcodec_client.h>
#include <VP_Stages/vp_stages_io_com.h>


#ifndef AT_MESSAGES_HEADER
#error You need to define AT_MESSAGES_HEADER
#endif


AT_CODEC_MSG_IDS ids;

static vp_stages_input_com_config_t icc;
static Write atcodec_write;
static Read atcodec_read;

static vp_com_t com;

#ifdef NO_COM
  static vp_com_serial_config_t          config;
#else // ! NO_COM
#ifdef USE_WIFI
  static vp_com_wifi_connection_t        connection;
  static vp_com_wifi_config_t            config;
#endif // ! USE_WIFI
#endif // > NO_COM

AT_CODEC_ERROR_CODE atresu_fake(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  PRINT("FAKE received !\n");
  return AT_CODEC_GENERAL_OK;
}

AT_CODEC_ERROR_CODE atresu_ok(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  PRINT("OK received !\n");
  return AT_CODEC_GENERAL_OK;
}

AT_CODEC_ERROR_CODE atresu_error(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  PRINT("ERROR received !\n");
  return AT_CODEC_GENERAL_OK;
}

AT_CODEC_ERROR_CODE atresu_cgmi(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  char str[1024];

  //int len =
  ATcodec_Memory_Raw_Get_Int(mem);

  ATcodec_Memory_Get_String(mem, &str[0]);

  PRINT("CGMI received : \"%s\" !\n", &str[0]);

  return AT_CODEC_GENERAL_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_Client_init(void)
{
  static int foo = 1;

  vp_os_memset(&com, 0, sizeof(vp_com_t));
  icc.com = &com;

  if(foo)
    {

# undef ATCODEC_DEFINE_AT_CMD
# define ATCODEC_DEFINE_AT_CMD(ID,Str,From,Cb,Prio) \
    if((ids.ID = ATcodec_Add_Defined_Message(Str)) == -1) \
      { \
        fprintf(stderr, "Error Add_Hashed \"%s\" library\n", Str); \
        return AT_CODEC_INIT_ERROR; \
      }

# undef ATCODEC_DEFINE_AT_RESU
# define ATCODEC_DEFINE_AT_RESU(ID,Str,From,Cb) \
    if((ids.ID = ATcodec_Add_Hashed_Message(Str,ids.From,Cb,0)) == -1) \
      { \
        fprintf(stderr, "Error Add_Defined \"%s\" library\n", Str); \
        return AT_CODEC_INIT_ERROR; \
      }

# include AT_MESSAGES_HEADER

  //ATcodec_Print_Tree();

  //vp_delay(100);

#ifdef NO_COM
#ifdef __linux__
  strcpy(config.itfName, "/dev/ttyUSB0");
#else
  strcpy(config.itfName, "/dev/ser0");
#endif
  config.initial_baudrate = VP_COM_BAUDRATE_115200;
  config.baudrate = VP_COM_BAUDRATE_115200;
  config.sync = 0; //1;

  com.type                = VP_COM_SERIAL;
  icc.socket.type         = VP_COM_CLIENT;
  icc.config              = (vp_com_config_t *)&config;
  icc.buffer_size         = 16;
#else // ! NO_COM
#ifdef USE_WIFI
  strcpy(connection.networkName,"linksys");

  strcpy(config.itfName,    "rausb0");
  strcpy(config.localHost,  "192.168.1.57");
  strcpy(config.netmask,    "255.255.255.0");
  strcpy(config.broadcast,  "192.168.1.255");
  strcpy(config.gateway,    "192.168.1.1");
  strcpy(config.server,     "192.168.1.1");
  strcpy(config.passkey,    "9F1C3EE11CBA230B27BF1C1B6F");
  config.infrastructure       = 1;
  config.secure               = 1;

  com.type                          = VP_COM_WIFI;
  icc.config                        = (vp_com_config_t*)&config;
  icc.connection                    = (vp_com_connection_t*)&connection;
  icc.socket.type                   = VP_COM_CLIENT;
  icc.socket.protocol               = VP_COM_TCP;
  icc.socket.port                   = 5555;
  icc.buffer_size                   = 6400;

  strcpy(icc.socket_client.serverHost,"192.168.1.23");
#endif // ! USE_WIFI
#endif // > NO_COM

  if(FAILED(vp_com_init(icc.com)))
  {
    PRINT("VP_Com : Failed to init\n");
    vp_com_shutdown(icc.com);
    return AT_CODEC_OPEN_ERROR;
  }

  if(FAILED(vp_com_local_config(icc.com, icc.config)))
  {
    PRINT("VP_Com : Failed to configure\n");
    vp_com_shutdown(icc.com);
    return AT_CODEC_OPEN_ERROR;
  }

  PRINT("com_init OK\n");

  foo = 0;

    }

  return AT_CODEC_INIT_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_Client_shutdown(void)
{
  ATcodec_Shutdown_Library();

  return AT_CODEC_SHUTDOWN_OK;
}

int rfcomm_socket;

AT_CODEC_ERROR_CODE AT_CODEC_Client_open(void)
{
  static int foo = 1;

  printf("ATcodec_open\n");

  if(foo)
    {
/*   if(FAILED(vp_com_connect(icc.connection, 1))) */
/*     return AT_CODEC_OPEN_ERROR; */

  if(FAILED(vp_com_open(icc.com, &icc.socket_client, &atcodec_read,&atcodec_write)))
    return AT_CODEC_OPEN_ERROR;

  foo = 0;

    }

  return AT_CODEC_OPEN_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_Client_close(void)
{
  printf("ATcodec_close\n");

  vp_com_close(icc.com, &icc.socket_client);

  return AT_CODEC_CLOSE_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_Client_write(int8_t *buffer, int32_t *len)
{
  //printf("ATcodec_write\n");

  if(FAILED(atcodec_write(&icc.socket_client, buffer, len)))
     return AT_CODEC_WRITE_ERROR;

  return AT_CODEC_WRITE_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_Client_read(int8_t *buffer, int32_t *len)
{
/*   printf("< ATcodec_read >\n"); */

  if(FAILED(atcodec_read(&icc.socket_client, buffer, len)))
       return AT_CODEC_READ_ERROR;

  if(*len)
    {
  buffer[*len] = '\0';
/*   PRINT("len=%d AT_CODEC_Client_read : \"", *len); */
/*   for( ; *buffer ; buffer++) */
/*     { */
/*       if(*buffer == '\r') */
/* 	{ */
/* 	  PRINT("<CR>"); */
/* 	} */
/*       else if(*buffer == '\n') */
/* 	{ */
/* 	  PRINT("<LF>"); */
/* 	} */
/*       else if(*buffer == '\0') */
/* 	{ */
/* 	  PRINT("<\\0>"); */
/* 	} */
/*       else */
/* 	{ */
/* 	  PRINT("%c", *buffer); */
/* 	} */
/*     } */
/*   PRINT("\"\n"); */
    }

  return AT_CODEC_READ_OK;
}
