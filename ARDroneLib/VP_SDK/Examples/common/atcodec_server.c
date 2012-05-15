
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
#include <VP_Stages/vp_stages_io_com.h>


#ifndef AT_MESSAGES_HEADER
#error You need to define AT_MESSAGES_HEADER
#endif


typedef struct _AT_CODEC_MSG_IDS_
{
# define ATCODEC_DEFINE_AT_CMD(ID,Str,From,Cb,Prio) AT_CODEC_MSG_ID ID;
# define ATCODEC_DEFINE_AT_RESU(ID,Str,From,Cb) AT_CODEC_MSG_ID ID;
# include AT_MESSAGES_HEADER
}
AT_CODEC_MSG_IDS;


static AT_CODEC_MSG_IDS ids;


static vp_stages_output_com_config_t occ;

static Write atcodec_write;
static Read atcodec_read;

static vp_com_t com;

#ifdef NO_COM
  static vp_com_serial_config_t    config;
#else // ! NO_COM
#ifdef USE_WIFI
  vp_com_wifi_connection_t         connection;
  vp_com_wifi_config_t             config;
#endif // ! USE_WIFI
#endif // > NO_COM



AT_CODEC_ERROR_CODE at_pm_qw(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  static int foo = 0;
  int motor = ATcodec_Memory_Raw_Get_Int(mem);

  PRINT("Motor selected : %d\n", motor);

  if(foo)
    *id = ids.AT_MSG_ATRESU_OK;
  else
    *id = ids.AT_MSG_ATRESU_ERROR;

  foo = 1-foo;

  return AT_CODEC_GENERAL_OK;
}

AT_CODEC_ERROR_CODE at_pm_exe(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  static int foo = 1;
  int speed = ATcodec_Memory_Raw_Get_Int(mem);
  int timestamp = ATcodec_Memory_Raw_Get_Int(mem);
  PRINT("Motor command : speed=%d , timestamp=%d\n", speed, timestamp);

  if(foo)
    *id = ids.AT_MSG_ATRESU_OK;
  else
    *id = ids.AT_MSG_ATRESU_ERROR;

  foo = 1-foo;

  return AT_CODEC_GENERAL_OK;
}

AT_CODEC_ERROR_CODE at_cgmi(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *id)
{
  PRINT("CGMI command\n");

  ATcodec_Memory_Put_String(output, "By Aurelien Morelle");

  *id = ids.AT_MSG_ATRESU_CGMI;

  return AT_CODEC_GENERAL_OK;
}


AT_CODEC_ERROR_CODE AT_CODEC_init(void)
{
  vp_os_memset(&com, 0, sizeof(vp_com_t));

  occ.com = &com;

# undef ATCODEC_DEFINE_AT_CMD
# define ATCODEC_DEFINE_AT_CMD(ID,Str,From,Cb,Prio) \
    if((ids.ID = ATcodec_Add_Hashed_Message(Str,From,Cb,Prio)) == -1) \
      { \
        PRINT("Error Add_Hashed \"%s\" library\n", Str); \
        return AT_CODEC_INIT_ERROR; \
      }

# undef ATCODEC_DEFINE_AT_RESU
# define ATCODEC_DEFINE_AT_RESU(ID,Str,From,Cb) \
    if((ids.ID = ATcodec_Add_Defined_Message(Str)) == -1) \
      { \
        PRINT("Error Add_Defined \"%s\" library\n", Str); \
        return AT_CODEC_INIT_ERROR; \
      }

# include AT_MESSAGES_HEADER

  //ATcodec_Print_Tree();

#ifdef NO_COM
#ifdef __linux__
  strcpy(config.itfName, "/dev/ttyS0");
#else
  strcpy(config.itfName, "/dev/ser1");
#endif
  config.initial_baudrate = VP_COM_BAUDRATE_115200;
  config.baudrate = VP_COM_BAUDRATE_115200;
  config.sync = 0; //1;

  com.type                          = VP_COM_SERIAL;

  occ.socket.type                   = VP_COM_SERVER;
  occ.config                        = (vp_com_config_t *)&config;
  occ.buffer_size                   = 16;
#else // ! NO_COM
#ifdef USE_WIFI
  vp_os_memset(&connection,0,sizeof(vp_com_connection_t));
  strcpy(connection.networkName,"linksys");

  strcpy(config.itfName,    "wl0");
  strcpy(config.localHost,  "192.168.1.23");
  strcpy(config.netmask,    "255.255.255.0");
  strcpy(config.broadcast,  "192.168.1.255");
  strcpy(config.gateway,    "192.168.1.1");
  strcpy(config.server,     "192.168.1.1");
  strcpy(config.passkey,    "9F1C3EE11CBA230B27BF1C1B6F");
  config.infrastructure = 1;
  config.secure = 1;

  com.type                          = VP_COM_WIFI;

  occ.config                        = (vp_com_config_t*)&config;
  occ.connection                    = (vp_com_connection_t*)&connection;
  occ.socket.type                   = VP_COM_SERVER;
  occ.socket.protocol               = VP_COM_TCP;
  occ.socket.port                   = 5555;
  occ.buffer_size                   = 128;
#endif // ! USE_WIFI
#endif // > NO_COM


  if(FAILED(vp_com_init(occ.com)))
  {
    PRINT("VP_Com : Failed to init\n");
    vp_com_shutdown(occ.com);
    return AT_CODEC_OPEN_ERROR;
  }

  if(FAILED(vp_com_local_config(occ.com, occ.config)))
  {
    PRINT("VP_Com : Failed to configure\n");
    vp_com_shutdown(occ.com);
    return AT_CODEC_OPEN_ERROR;
  }

  PRINT("com_init OK\n");

  return AT_CODEC_INIT_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_shutdown(void)
{
  ATcodec_Shutdown_Library();

  return AT_CODEC_SHUTDOWN_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_open(void)
{
  PRINT("ATcodec_open\n");

  if(FAILED(vp_com_open(occ.com, &occ.socket, &atcodec_read,&atcodec_write)))
    return AT_CODEC_OPEN_ERROR;

  if(FAILED(vp_com_wait_connections(occ.com, &occ.socket, &occ.socket_client, 1)))
    return AT_CODEC_OPEN_ERROR;

  PRINT("Connection RFCOMM incoming\n");

  return AT_CODEC_OPEN_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_close(void)
{
  PRINT("ATcodec_close\n");

  vp_com_close(occ.com, &occ.socket);
  vp_com_close(occ.com, &occ.socket_client);

  PRINT("socket closed\n");

  return AT_CODEC_CLOSE_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_write(int8_t *buffer, int32_t *len)
{
/*   int i; */

  if(FAILED(atcodec_write(&occ.socket_client, buffer, len)))
     return AT_CODEC_WRITE_ERROR;

/*   if(*len) */
/*     { */
/*       PRINT("ATcodec_write < "); */
/*       for(i = 0 ; i < *len ; i++) */
/* 	{ */
/* 	  if(buffer[i] == '\r') */
/* 	    PRINT("_CR_"); */
/* 	  else if(buffer[i] == '\n') */
/* 	    PRINT("_LF_"); */
/* 	  else */
/* 	    PRINT("%c", buffer[i]); */
/* 	} */
/*       PRINT(" > (%d)\n", *len); */
/*     } */

  return AT_CODEC_WRITE_OK;
}

AT_CODEC_ERROR_CODE AT_CODEC_read(int8_t *buffer, int32_t *len)
{
/*   int i; */

  if(FAILED(atcodec_read(&occ.socket_client, buffer, len)))
     return AT_CODEC_READ_ERROR;

  if(*len)
    {
      buffer[*len] = '\0';
/*       PRINT("ATcodec_read < "); */
/*       for(i = 0 ; i < *len ; i++) */
/* 	{ */
/* 	  if(buffer[i] == '\r') */
/* 	    PRINT("_CR_"); */
/* 	  else if(buffer[i] == '\n') */
/* 	    PRINT("_LF_"); */
/* 	  else if(buffer[i] == '\0') */
/* 	    PRINT("_\\0_"); */
/* 	  else */
/* 	    PRINT("%c", buffer[i]); */
/* 	} */
/*       PRINT(" > (%d)\n", *len); */
    }

  return AT_CODEC_READ_OK;
}
