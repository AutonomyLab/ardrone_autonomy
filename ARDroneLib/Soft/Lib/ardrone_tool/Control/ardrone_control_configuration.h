#ifndef _ARDRONE_CONTROL_CONFIGURATION_H_
#define _ARDRONE_CONTROL_CONFIGURATION_H_

#include <stdio.h>
#include <iniparser3.0b/src/iniparser.h>

#include <ardrone_tool/Control/ardrone_control.h>

// Size of temporary buffer used to parse incoming configuration data
// This value should be big enough to hold a line from the config file
#define ARDRONE_CONTROL_CONFIGURATION_INI_BUFFER_SIZE         1024

typedef enum _ardrone_config_state_t {
  CONFIG_REQUEST_INI,
  CONFIG_RECEIVE_INI,
  /* Stephane - multiconfiguration support */
  CUSTOM_CONFIG_REQUEST,
  CUSTOM_CONFIG_RECEIVE,
  CONFIG_RECEIVED,
  CUSTOM_CONFIG_RECEIVED
} ardrone_config_state_t;

typedef struct _ardrone_control_configuration_event_t {

  uint32_t  event;            // event type
  uint32_t  num_retries;      // number of times we'll try to execute this event
  uint32_t  status;           // event status

  ardrone_control_event_cb  ardrone_control_event_start;
  ardrone_control_event_cb  ardrone_control_event_end;

  ardrone_config_state_t  config_state;
  dictionary* ini_dict;

} ardrone_control_configuration_event_t;

C_RESULT ardrone_control_configuration_run( uint32_t ardrone_state, ardrone_control_configuration_event_t* event );

#endif // _ARDRONE_CONTROL_CONFIGURATION_H_
