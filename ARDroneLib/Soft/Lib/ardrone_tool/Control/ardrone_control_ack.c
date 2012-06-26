#include <config.h>
#include <ardrone_api.h>
#include <ardrone_tool/Control/ardrone_control_ack.h>

C_RESULT ardrone_control_ack_run( uint32_t ardrone_state, ardrone_control_ack_event_t* event )
{
  C_RESULT res;

  switch( event->ack_state )
  {
    case ACK_COMMAND_MASK_TRUE:
      res = ( ardrone_state & ARDRONE_COMMAND_MASK ) ? C_OK : C_FAIL;
      if( VP_SUCCEEDED(res) )
      {
    	  ardrone_at_update_control_mode( ACK_CONTROL_MODE);
          event->ack_state = ACK_COMMAND_MASK_FALSE;
      }
      break;

    case ACK_COMMAND_MASK_FALSE:
      ardrone_at_update_control_mode( ACK_CONTROL_MODE);

      res = ( ( ardrone_state & ARDRONE_COMMAND_MASK ) == 0 ) ? C_OK : C_FAIL;
      if( VP_SUCCEEDED(res) )
        event->status = ARDRONE_CONTROL_EVENT_FINISH_SUCCESS;
      break;

    default:
      res = C_FAIL;
      event->ack_state = ACK_COMMAND_MASK_TRUE; // Go back to a know state (we may timeout)
      break;
  }

  return res;
}
