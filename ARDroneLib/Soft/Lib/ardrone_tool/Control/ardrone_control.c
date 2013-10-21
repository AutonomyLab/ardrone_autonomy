#include <config.h>

#include <VP_Os/vp_os_print.h>
#include <VP_Com/vp_com.h>

#include <ardrone_api.h>
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/Control/ardrone_control_configuration.h>
#include <ardrone_tool/Control/ardrone_control_ack.h>
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/Com/config_wifi.h>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#endif

static vp_com_socket_t  control_socket;
static Read             control_read    = NULL;
static Write            control_write   = NULL;

static vp_os_mutex_t  control_mutex;
static vp_os_cond_t   control_cond;

static bool_t   control_waited;
static bool_t bContinue = TRUE;
static uint32_t ardrone_state;

static int32_t start_index_in_queue;
static int32_t end_index_in_queue;
static ardrone_control_event_ptr_t  ardrone_control_event_queue[ARDRONE_CONTROL_MAX_NUM_EVENTS_IN_QUEUE];

static vp_os_mutex_t  event_queue_mutex;

C_RESULT ardrone_control_init(void)
{
  COM_CONFIG_SOCKET_CONTROL(&control_socket, VP_COM_CLIENT, CONTROL_PORT, wifi_ardrone_ip);
	
  control_waited = FALSE;
  ardrone_state   = 0;

  vp_os_mutex_init(&control_mutex);
  vp_os_cond_init(&control_cond, &control_mutex);

  vp_os_mutex_init(&event_queue_mutex);

  start_index_in_queue = 0;
  end_index_in_queue   = (start_index_in_queue - 1) & (ARDRONE_CONTROL_MAX_NUM_EVENTS_IN_QUEUE - 1);

  return C_OK;
}

C_RESULT ardrone_control_shutdown(void)
{
	ardrone_control_resume_on_navdata_received(0);
	/*BUG FIX : Dont destroy the mutexes here,
	they are still being used by the ardrone_control thread,
	while this function is called by another thread.*/

	bContinue = FALSE;

	return C_OK;
}

C_RESULT ardrone_control_connect_to_drone()
{
	int res_open_socket;

#ifdef _WIN32
	int timeout_windows=1000;/*milliseconds*/
#else
	struct timeval tv;
#endif

	res_open_socket = vp_com_open(COM_CONTROL(), &control_socket, &control_read, &control_write);
	if( VP_SUCCEEDED(res_open_socket) )
	{
		tv.tv_sec   = 1;
		tv.tv_usec  = 0;

		setsockopt((int32_t)control_socket.priv,
					SOL_SOCKET,
					SO_RCVTIMEO,
					#ifdef _WIN32
						(const char*)&timeout_windows, sizeof(timeout_windows)
					#else
						(const char*)&tv, sizeof(tv)
					#endif
					);

		control_socket.is_disable = FALSE;
		return C_OK;
	}
	else
	{
		DEBUG_PRINT_SDK("VP_Com : Failed to open socket for control\n");
		perror("FTOSFC");
		return C_FAIL;
	}
}


/**
 * \brief Signals the client control thread that new navdata were received.
 * Called by one of the navdata callbacks.
 */
C_RESULT ardrone_control_resume_on_navdata_received(uint32_t new_ardrone_state)
{
	vp_os_mutex_lock(&control_mutex);
    if( control_waited )
	{
		ardrone_state = new_ardrone_state;
		vp_os_cond_signal(&control_cond);
	}
	vp_os_mutex_unlock(&control_mutex);
	return C_OK;
}

C_RESULT ardrone_control_read(uint8_t* buffer, int32_t* size)
{
  C_RESULT res = C_FAIL;

  if( control_read != NULL )
  {
	  res = control_read(&control_socket, buffer, size);
	  if(*size == 0)
	  {
		  res = C_FAIL;
	  }
  }

  return res;
}

C_RESULT ardrone_control_write(const uint8_t* buffer, int32_t* size)
{
  C_RESULT res = C_FAIL;

  if( control_write != NULL )
  {
    res = control_write(&control_socket, buffer, size);
  }

  return res;
}

C_RESULT ardrone_control_send_event( ardrone_control_event_t* event )
{
  C_RESULT res;
  int32_t next_index_in_queue;

  res = C_FAIL;

  vp_os_mutex_lock(&event_queue_mutex);
    next_index_in_queue = (start_index_in_queue + 1) & (ARDRONE_CONTROL_MAX_NUM_EVENTS_IN_QUEUE - 1);
    if( next_index_in_queue != end_index_in_queue )
    {
      ardrone_control_event_queue[start_index_in_queue] = event;
      start_index_in_queue = next_index_in_queue;

      res = C_OK;
  }

  vp_os_mutex_unlock(&event_queue_mutex);

  return res;
}

DEFINE_THREAD_ROUTINE( ardrone_control, nomParams )
{
	C_RESULT res_wait_navdata = C_OK;
	C_RESULT res = C_OK;
	uint32_t retry, current_ardrone_state;
	int32_t next_index_in_queue;
	ardrone_control_event_ptr_t  current_event;
	
	retry = 0;
	current_event = NULL;
	
	DEBUG_PRINT_SDK("Thread control in progress...\n");
	control_socket.is_disable = TRUE;
	
	ardrone_control_connect_to_drone();

    while( bContinue
          && !ardrone_tool_exit() )
	{
		vp_os_mutex_lock(&control_mutex);
		control_waited = TRUE;

		/* Wait for new navdata to be received. */
		res_wait_navdata = vp_os_cond_timed_wait(&control_cond, 1000);
		vp_os_mutex_unlock(&control_mutex);

		/*
		 * In case of timeout on the navdata, we assume that there was a problem
		 * with the Wifi connection.
		 * It is then safer to close and reopen the control socket (TCP 5559) since
		 * some OS might stop giving data but not signal any disconnection.
		 */
		if(VP_FAILED(res_wait_navdata))
		{
            if (FALSE == control_socket.is_disable)
            {
			DEBUG_PRINT_SDK("Timeout while waiting for new navdata.\n");
                vp_com_close(COM_CONTROL(), &control_socket);
				control_socket.is_disable = TRUE;
		}
        }
        else
        {
			
		if(control_socket.is_disable)
		{
			ardrone_control_connect_to_drone();
		}
		
		if(VP_SUCCEEDED(res_wait_navdata) && (!control_socket.is_disable))
		{
			vp_os_mutex_lock(&control_mutex);
			current_ardrone_state = ardrone_state;
			control_waited = FALSE;
			vp_os_mutex_unlock(&control_mutex);
			
			if( ardrone_tool_exit() ) // Test if we received a signal because we are quitting the application
				THREAD_RETURN( res );
			
 			if( current_event == NULL )
			{
				vp_os_mutex_lock(&event_queue_mutex);
				next_index_in_queue = (end_index_in_queue + 1) & (ARDRONE_CONTROL_MAX_NUM_EVENTS_IN_QUEUE - 1);
				
				if( next_index_in_queue != start_index_in_queue )
				{ // There's an event to process
					current_event = ardrone_control_event_queue[next_index_in_queue];
					if( current_event != NULL )
					{
						if( current_event->ardrone_control_event_start != NULL )
						{
							current_event->ardrone_control_event_start( current_event );
						}
					}
					end_index_in_queue = next_index_in_queue;
					
					retry = 0;
				}
				
				vp_os_mutex_unlock(&event_queue_mutex);
			}
			
			if( current_event != NULL )
			{
				switch( current_event->event )
				{
					case CFG_GET_CONTROL_MODE:
					case CUSTOM_CFG_GET_CONTROL_MODE: /* multiconfiguration support */
						res = ardrone_control_configuration_run( current_ardrone_state, (ardrone_control_configuration_event_t*) current_event );
						break;
						
					case ACK_CONTROL_MODE:
						res = ardrone_control_ack_run( current_ardrone_state, (ardrone_control_ack_event_t *) current_event);
						break;
						
					default:
						break;
				}
				
				if( VP_FAILED(res) )
				{
					retry ++;
					if( retry > current_event->num_retries)
						current_event->status = ARDRONE_CONTROL_EVENT_FINISH_FAILURE;
				}
				else
				{
					retry = 0;
				}
				
				if( current_event->status & ARDRONE_CONTROL_EVENT_FINISH )
				{
 					if( current_event->ardrone_control_event_end != NULL )
						current_event->ardrone_control_event_end( current_event );
					
 					/* Make the thread read a new event on the next loop iteration */
					current_event = NULL;
				}
				else
				{
					/* Not changing 'current_event' makes the loop process the same
					 * event when the next navdata packet arrives. */
                    }
				}
			}
		}
  }// while

  /*Bug fix - mutexes were previously detroyed by another thread,
  which made ardrone_control crash.*/
	  vp_os_mutex_destroy(&event_queue_mutex);
	  vp_os_cond_destroy(&control_cond);
	  vp_os_mutex_destroy(&control_mutex);

  vp_com_close(COM_CONTROL(), &control_socket);

  THREAD_RETURN( res );
}
