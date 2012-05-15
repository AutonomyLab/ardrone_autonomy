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

typedef struct _input_state_t
{
  int32_t ag;
  int32_t ab;
  int32_t ad;
  int32_t ah;
  int32_t l1;
  int32_t r1;
  int32_t l2;
  int32_t r2;
  int32_t select;
  int32_t start;

  int32_t x;
  int32_t y;
} input_state_t;

// Input change handling
input_state_t* ardrone_tool_get_input_state( void ); 
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

// Callbacks for user input event handling
extern C_RESULT custom_reset_user_input(input_state_t* input_state, uint32_t user_input ) WEAK;
extern C_RESULT custom_update_user_input(input_state_t* input_state, uint32_t user_input ) WEAK;

// Input API
C_RESULT ardrone_tool_input_add( input_device_t* device );
C_RESULT ardrone_tool_input_remove( input_device_t* device );

C_RESULT ardrone_tool_input_init(void);
C_RESULT ardrone_tool_input_reset(void);
C_RESULT ardrone_tool_input_update(void);
C_RESULT ardrone_tool_input_shutdown(void);
C_RESULT ardrone_tool_start_reset(void);

#endif // _ARDRONE_INPUT_H_
