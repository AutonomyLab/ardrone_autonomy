#include <VP_Os/vp_os_malloc.h>

#include <config.h>
#include <ardrone_api.h>
#include <ardrone_tool/Control/ardrone_control_soft_update.h>

int8_t*  send_buffer = NULL;
int32_t  send_buffer_size = 0;

C_RESULT ardrone_control_soft_update_run( uint32_t ardrone_state, ardrone_control_soft_update_event_t* event )
{
  C_RESULT res = C_OK;

  switch( event->ack_received )
  {
  case 0: // First step - Reset command mask if needed
    if( ardrone_state & ARDRONE_COMMAND_MASK )
    {
      ardrone_at_update_control_mode(ACK_CONTROL_MODE, 0);
    }
    else
    {
      event->ack_received ++;
    }
    break;
  case 1:
    if( !(ardrone_state & ARDRONE_COMMAND_MASK) )
    {
        ardrone_at_update_control_mode(event->event, event->filesize);
    }
    else
    {
      event->ack_received ++;
    }
    break;

  case 2:
	if( (ardrone_state & ARDRONE_COMMAND_MASK) == 0)
	{
		if((ardrone_state & ARDRONE_FW_FILE_MASK) && (ardrone_state & ARDRONE_FW_VER_MASK))
		{
			event->status = ARDRONE_CONTROL_EVENT_FINISH_SUCCESS;
			event->error_state = FIRMWARE_NO_ERR;
		}
		else
	    {
	    	event->status = ARDRONE_CONTROL_EVENT_FINISH_FAILURE;
	    	if(ardrone_state & ARDRONE_FW_VER_MASK)
	    		event->error_state = FIRMWARE_FILE_ERR;
	    	else
	    		event->error_state = FIRMWARE_VERSION_ERR;
	    }
	}
    break;
  }

  return res;
}
