#ifndef _ARDRONE_TOOL_CONTROL_H_
#define _ARDRONE_TOOL_CONTROL_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_thread_helper.h>

#define ARDRONE_CONTROL_MAX_NUM_EVENTS_IN_QUEUE   32

typedef enum _ardrone_control_event_status_t {
  ARDRONE_CONTROL_EVENT_IDLE            = 0x0000,
  ARDRONE_CONTROL_EVENT_WAITING         = 0x1000,
  ARDRONE_CONTROL_EVENT_IN_PROGRESS     = 0x2000,
  ARDRONE_CONTROL_EVENT_FINISH          = 0x4000,
  ARDRONE_CONTROL_EVENT_FINISH_SUCCESS  = 0x4001,
  ARDRONE_CONTROL_EVENT_FINISH_FAILURE  = 0x4002
} ardrone_control_event_status_t;

struct _ardrone_control_event_t;

typedef void (*ardrone_control_event_cb)( struct _ardrone_control_event_t* event );

typedef struct _ardrone_control_event_t {

  uint32_t event;       // event type
  uint32_t num_retries; // number of times we'll try to execute this event
  uint32_t status;      // event status

  ardrone_control_event_cb  ardrone_control_event_start;
  ardrone_control_event_cb  ardrone_control_event_end;

  uint8_t data[];       // User data associated with this event

} ardrone_control_event_t, *ardrone_control_event_ptr_t;

C_RESULT ardrone_control_init(void);
C_RESULT ardrone_control_shutdown(void);
C_RESULT ardrone_control_resume_on_navdata_received(uint32_t ardrone_state);

C_RESULT ardrone_control_read(int8_t* buffer, int32_t* size);
C_RESULT ardrone_control_write(const int8_t* buffer, int32_t* size);

C_RESULT ardrone_control_send_event( ardrone_control_event_t* event );

PROTO_THREAD_ROUTINE( ardrone_control, nomParams );

#endif // _ARDRONE_TOOL_CONTROL_H_
