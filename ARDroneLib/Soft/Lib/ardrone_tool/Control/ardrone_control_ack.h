#ifndef _ARDRONE_CONTROL_ACK_H_
#define _ARDRONE_CONTROL_ACK_H_

#include <ardrone_tool/Control/ardrone_control.h>

typedef enum _ardrone_ack_state_t {
  ACK_COMMAND_MASK_TRUE,  // First we are waiting command mask goes true
  ACK_COMMAND_MASK_FALSE, // Then we are waiting after its reset
} ardrone_ack_state_t;

typedef struct _ardrone_control_ack_event_t {

  uint32_t  event;            // event type
  uint32_t  num_retries;      // number of times we'll try to execute this event
  uint32_t  status;           // event status

  ardrone_control_event_cb  ardrone_control_event_start;
  ardrone_control_event_cb  ardrone_control_event_end;

  ardrone_ack_state_t   ack_state;

} ardrone_control_ack_event_t;

C_RESULT ardrone_control_ack_run( uint32_t ardrone_state, ardrone_control_ack_event_t* event );

#endif // _ARDRONE_CONTROL_CONFIGURATION_H_
