#ifndef _ARDRONE_INPUT_H_
#define _ARDRONE_INPUT_H_

#include <VP_Os/vp_os_types.h>
#include <ardrone_tool/ardrone_tool.h>

typedef struct _input_device_t {
  char name[MAX_NAME_LENGTH];

  C_RESULT (*init)(void);
  C_RESULT (*update)(void);
  C_RESULT (*shutdown)(void);
} input_device_t;

typedef struct _input_state_pcmd_t_
{
	int32_t flag;
	float32_t phi;
	float32_t theta;
	float32_t gaz;
	float32_t yaw;
	float32_t psi;
	float32_t psi_accuracy;
} input_state_pcmd_t;

typedef struct _input_state_t
{
  uint32_t user_input;
  input_state_pcmd_t pcmd;
} input_state_t;

// Input change handling
C_RESULT ardrone_tool_set_ui_pad_ag(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_ab(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_ad(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_ah(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_l1(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_r1(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_l2(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_r2(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_select(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_start(int32_t value);
C_RESULT ardrone_tool_set_ui_pad_xy(int32_t x, int32_t y);

C_RESULT ardrone_tool_set_ui_pad_phi_trim( int32_t phi_trim );
C_RESULT ardrone_tool_set_ui_pad_theta_trim( int32_t theta_trim );
C_RESULT ardrone_tool_set_ui_pad_yaw_trim( int32_t yaw_trim );

C_RESULT ardrone_tool_set_progressive_cmd(int32_t flag, float32_t phi, float32_t theta, float32_t gaz, float32_t yaw, float32_t psi, float32_t psi_accuracy);

input_state_t* ardrone_tool_input_get_state( void ); 

// Input API
C_RESULT ardrone_tool_input_add( input_device_t* device );
C_RESULT ardrone_tool_input_remove( input_device_t* device );

C_RESULT ardrone_tool_input_init(void);
C_RESULT ardrone_tool_input_reset(void);
C_RESULT ardrone_tool_input_update(void);
C_RESULT ardrone_tool_input_shutdown(void);
C_RESULT ardrone_tool_input_start_reset(void);

#endif // _ARDRONE_INPUT_H_
