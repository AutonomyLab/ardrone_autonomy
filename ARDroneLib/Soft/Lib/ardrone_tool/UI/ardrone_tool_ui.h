#ifndef _ARDRONE_TOOL_UI_H_
#define _ARDRONE_TOOL_UI_H_

#include <VP_Os/vp_os_types.h>
#include <ardrone_tool/UI/ardrone_input.h>

C_RESULT ui_pad_ab(int32_t value);
C_RESULT ui_pad_ad(int32_t value);
C_RESULT ui_pad_ag(int32_t value);
C_RESULT ui_pad_ah(int32_t value);
C_RESULT ui_pad_l1(int32_t value);
C_RESULT ui_pad_r1(int32_t value);
C_RESULT ui_pad_l2(int32_t value);
C_RESULT ui_pad_r2(int32_t value);
C_RESULT ui_pad_xy_change(int32_t x, int32_t y);
C_RESULT ui_pad_phi_trim( int32_t phi_trim );
C_RESULT ui_pad_theta_trim( int32_t theta_trim );
C_RESULT ui_pad_yaw_trim( int32_t yaw_trim );

C_RESULT ui_pad_reset_user_input(input_state_t* input_state);
C_RESULT ui_pad_update_user_input(input_state_t* input_state);
C_RESULT ui_pad_reset_user_input_start(input_state_t* input_state);

C_RESULT ui_pad_start_stop(int32_t value);
C_RESULT ui_pad_select(int32_t value);

uint32_t ui_get_user_input(void);

#endif // _ARDRONE_TOOL_UI_H_
