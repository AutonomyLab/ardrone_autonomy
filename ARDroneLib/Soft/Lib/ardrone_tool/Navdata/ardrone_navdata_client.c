#include <errno.h>
#include <string.h>

#include <config.h>

#include <VP_Os/vp_os_print.h>
#include <VP_Com/vp_com.h>

#include <ardrone_api.h>
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Com/config_com.h>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#endif

// PROTO_THREAD_ROUTINE( navdata_update , nomParams );

static bool_t navdata_thread_in_pause = TRUE;
static bool_t bContinue = TRUE;
static uint32_t num_retries = 0; 
static vp_os_cond_t navdata_client_condition;
static vp_os_mutex_t navdata_client_mutex;

static vp_com_socket_t navdata_socket;
static Read navdata_read      = NULL;
static Write navdata_write    = NULL;

static uint8_t navdata_buffer[NAVDATA_MAX_SIZE];
navdata_unpacked_t navdata_unpacked;

C_RESULT ardrone_navdata_client_init(void)
{
  C_RESULT res;

  COM_CONFIG_SOCKET_NAVDATA(&navdata_socket, VP_COM_CLIENT, NAVDATA_PORT, wifi_ardrone_ip);
  navdata_socket.protocol = VP_COM_UDP;
  navdata_socket.is_multicast = 1;      // enable multicast for Navdata
  navdata_socket.multicast_base_addr = MULTICAST_BASE_ADDR;

  vp_os_mutex_init(&navdata_client_mutex);
  vp_os_cond_init(&navdata_client_condition, &navdata_client_mutex);
	
  res = C_OK;

  return res;
}

C_RESULT ardrone_navdata_client_suspend(void)
{
	vp_os_mutex_lock(&navdata_client_mutex);
	navdata_thread_in_pause = TRUE;
	vp_os_mutex_unlock(&navdata_client_mutex);	
	
	return C_OK;
}

C_RESULT ardrone_navdata_client_resume(void)
{
	vp_os_mutex_lock(&navdata_client_mutex);
	vp_os_cond_signal(&navdata_client_condition);
	navdata_thread_in_pause = FALSE;
	vp_os_mutex_unlock(&navdata_client_mutex);	

	return C_OK;
}

C_RESULT ardrone_navdata_open_server(void)
{
  // Flag value :
  // 1 -> Unicast
  // 2 -> Multicast
  int32_t flag = 1, len = sizeof(flag);

  if( navdata_write != NULL )
  {
    if (navdata_socket.is_multicast == 1)
      flag = 2;

    navdata_write(&navdata_socket, (const int8_t*) &flag, &len);
  }

  return C_OK;
}

DEFINE_THREAD_ROUTINE( navdata_update, nomParams )
{
  C_RESULT res;
  int32_t  i, size;
  uint32_t cks, navdata_cks, sequence = NAVDATA_SEQUENCE_DEFAULT-1;
  struct timeval tv;
#ifdef _WIN32
  int timeout_for_windows=1000/*milliseconds*/;
#endif


  navdata_t* navdata = (navdata_t*) &navdata_buffer[0];

  tv.tv_sec   = 1/*second*/;
  tv.tv_usec  = 0;

  res = C_OK;

  if( VP_FAILED(vp_com_open(COM_NAVDATA(), &navdata_socket, &navdata_read, &navdata_write)) )
  {
    printf("VP_Com : Failed to open socket for navdata\n");
    res = C_FAIL;
  }

  if( VP_SUCCEEDED(res) )
  {
    PRINT("Thread navdata_update in progress...\n");

#ifdef _WIN32
	setsockopt((int32_t)navdata_socket.priv, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_for_windows, sizeof(timeout_for_windows));
	/* Added by Stephane to force the drone start sending data. */
	if(navdata_write)
	{	int sizeinit = 5; navdata_write( (void*)&navdata_socket, (int8_t*)"Init", &sizeinit ); }
#else
	setsockopt((int32_t)navdata_socket.priv, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif


    i = 0;
    while( ardrone_navdata_handler_table[i].init != NULL )
    {
      // if init failed for an handler we set its process function to null
      // We keep its release function for cleanup
      if( VP_FAILED( ardrone_navdata_handler_table[i].init(ardrone_navdata_handler_table[i].data) ) )
        ardrone_navdata_handler_table[i].process = NULL;

      i ++;
    }

   navdata_thread_in_pause = FALSE;
    while( VP_SUCCEEDED(res) 
           && !ardrone_tool_exit() 
           && bContinue )
    {
	  if(navdata_thread_in_pause)
	  {
		vp_os_mutex_lock(&navdata_client_mutex);
		num_retries = NAVDATA_MAX_RETRIES + 1;
		vp_os_cond_wait(&navdata_client_condition);
		vp_os_mutex_unlock(&navdata_client_mutex);
	  }
	
		if( navdata_read == NULL )
      {
        res = C_FAIL;
        continue;
      }

      size = NAVDATA_MAX_SIZE;
      navdata->header = 0; // Soft reset
      res = navdata_read( (void*)&navdata_socket, (int8_t*)&navdata_buffer[0], &size );

#ifdef _WIN32	
	  if( size <= 0 )
#else
	  if( size == 0 )
#endif
		{
			// timeout
			PRINT("Timeout when reading navdatas - resending a navdata request on port %i\n",NAVDATA_PORT);
			/* Resend a request to the drone to get navdatas */
			ardrone_navdata_open_server();
			sequence = NAVDATA_SEQUENCE_DEFAULT-1;
			num_retries++;
		} 
		else
         num_retries = 0;

      if( VP_SUCCEEDED( res ) )
      {
        if( navdata->header == NAVDATA_HEADER )
        {
          if( ardrone_get_mask_from_state(navdata->ardrone_state, ARDRONE_COM_WATCHDOG_MASK) )
          {
            // reset sequence number because of com watchdog
            // This code is mandatory because we can have a com watchdog without detecting it on mobile side :
            //        Reconnection is fast enough (less than one second)
            sequence = NAVDATA_SEQUENCE_DEFAULT-1;

            if( ardrone_get_mask_from_state(navdata->ardrone_state, ARDRONE_NAVDATA_BOOTSTRAP) == FALSE )
              ardrone_tool_send_com_watchdog(); // acknowledge
          }

          if( navdata->sequence > sequence )
          {
            i = 0;

            ardrone_navdata_unpack_all(&navdata_unpacked, navdata, &navdata_cks);
            cks = ardrone_navdata_compute_cks( &navdata_buffer[0], size - sizeof(navdata_cks_t) );

            if( cks == navdata_cks )
            {
              while( ardrone_navdata_handler_table[i].init != NULL )
              {
                if( ardrone_navdata_handler_table[i].process != NULL )
                  ardrone_navdata_handler_table[i].process( &navdata_unpacked );

                i++;
              }
            }
            else
            {
              PRINT("[Navdata] Checksum failed : %d (distant) / %d (local)\n", navdata_cks, cks);
            }
          }
          else
          {
            PRINT("[Navdata] Sequence pb : %d (distant) / %d (local)\n", navdata->sequence, sequence);
          }

          // remaining = sizeof(navdata);

          sequence = navdata->sequence;
        }
      }
    }

    // Release resources alllocated by handlers
    i = 0;
    while( ardrone_navdata_handler_table[i].init != NULL )
    {
      ardrone_navdata_handler_table[i].release();

      i ++;
    }
  }

  vp_com_close(COM_NAVDATA(), &navdata_socket);

  DEBUG_PRINT_SDK("Thread navdata_update ended\n");

  return (THREAD_RET)res;
}

uint32_t ardrone_navdata_client_get_num_retries(void)
{
  return num_retries;
}

C_RESULT ardrone_navdata_client_shutdown(void)
{
   bContinue = FALSE;

   return C_OK;
}


