#ifndef _ARDRONE_CONTROL_SOFT_UPDATE_H_
#define _ARDRONE_CONTROL_SOFT_UPDATE_H_

#include <ardrone_tool/Control/ardrone_control.h>

#include <stdio.h>

typedef enum _ardrone_control_soft_update_error_state_t
{
	FIRMWARE_NO_ERR,
	FIRMWARE_FILE_ERR,
	FIRMWARE_VERSION_ERR
} ardrone_control_soft_update_error_state_t;

typedef struct _ardrone_control_soft_update_event_t {

  uint32_t  event;            // event type
  uint32_t  num_retries;      // number of times we'll try to execute this event
  uint32_t  status;           // event status

  ardrone_control_event_cb  ardrone_control_event_start;
  ardrone_control_event_cb  ardrone_control_event_end;

  uint32_t  ack_received;     // Progress of this event
  ardrone_control_soft_update_error_state_t error_state;
  uint32_t  filesize;
  uint32_t  sendsize;         // Data size in bytes we'll send per write
  FILE*     fp;               // Descriptor of opened file

} ardrone_control_soft_update_event_t;

C_RESULT ardrone_control_soft_update_run( uint32_t ardrone_state, ardrone_control_soft_update_event_t* event );

#endif // _ARDRONE_CONTROL_SOFT_UPDATE_H_
