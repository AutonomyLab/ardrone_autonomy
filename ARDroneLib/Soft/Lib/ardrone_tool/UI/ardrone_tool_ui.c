//
// Inputs specification
// rien d'appuyer : 11540000
// start: 11540200
// select: 11540000
// 1 : 11540001
// 2 : 11540002
// 3 : 11540004
// 4 : 11540008
// 5 : 11540010
// 6 : 11540020
// 7 : 11540040
// 8 : 11540080
// x+: 12540000
// x-: 10540000
// y+: 01540000
// y-: 21540000
//
// trims
// r2 + x+ = 11640080
// r2 + x- = 11440080
// r2 + y+ = 11500080
// r2 + y- = 11580080
// r2 + 5  = 11140090
// r2 + 6  = 119400A0
//

#include <config.h>
#include <ardrone_api.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/UI/ardrone_tool_ui.h>

static uint32_t user_input = 0;

C_RESULT ui_pad_ab(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_AB);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_AB);

	return C_OK;
}

C_RESULT ui_pad_ag(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_AG);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_AG);

	return C_OK;
}

C_RESULT ui_pad_ad(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_AD);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_AD);

	return C_OK;
}

C_RESULT ui_pad_ah(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_AH);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_AH);

	return C_OK;
}

C_RESULT ui_pad_l1(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_L1);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_L1);

	return C_OK;
}

C_RESULT ui_pad_r1(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_R1);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_R1);

	return C_OK;
}

C_RESULT ui_pad_l2(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_L2);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_L2);

	return C_OK;
}

C_RESULT ui_pad_r2(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_R2);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_R2);

	return C_OK;
}

C_RESULT ui_pad_start_stop(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_START);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_START);

	return C_OK;
}

C_RESULT ui_pad_select(int32_t value)
{
	if( value )
		user_input |= (1 << ARDRONE_UI_BIT_SELECT);
	else
		user_input &= ~(1 << ARDRONE_UI_BIT_SELECT);
	
	return C_OK;
}

C_RESULT ui_pad_xy_change(int32_t x, int32_t y)
{
  user_input &= ~(3 << ARDRONE_UI_BIT_X);
  user_input &= ~(3 << ARDRONE_UI_BIT_Y);

  user_input |= (x + 1) << ARDRONE_UI_BIT_X;
  user_input |= (y + 1) << ARDRONE_UI_BIT_Y;

  return C_OK;
}

C_RESULT ui_pad_reset_user_input(input_state_t* input_state)
{
  user_input = 0;
  
  input_state->ag       = 0;
  input_state->ab       = 0;
  input_state->ad       = 0;
  input_state->ah       = 0;
  input_state->l1       = 0;
  input_state->r1       = 0;
  input_state->l2       = 0;
  //input_state->r2       = 0; (To avoid angle trim done when CTRL_TRANS_LANDING =>CTRL_LANDING)
  input_state->select   = 0;
  input_state->start    = 0;

  input_state->x        = 0;
  input_state->y        = 0;
  
  ui_pad_ag(1);
  ui_pad_ag(0);
  ui_pad_ab(1);
  ui_pad_ab(0);
  ui_pad_ad(1);
  ui_pad_ad(0);
  ui_pad_ah(1);
  ui_pad_ah(0);
  ui_pad_l1(1);
  ui_pad_l1(0);
  ui_pad_r1(1);
  ui_pad_r1(0);
  ui_pad_l2(1);
  ui_pad_l2(0);
  //ui_pad_r2_pressed(input_state);
  //ui_pad_r2_released(input_state);
  ui_pad_xy_change(0, 0);
  ui_pad_yaw_trim(0);
  ui_pad_phi_trim(0);
  ui_pad_theta_trim(0);

  return custom_reset_user_input(input_state, user_input);
}

C_RESULT ui_pad_reset_user_input_start(input_state_t* input_state)
{
  ui_pad_start_stop(0);

  return custom_reset_user_input(input_state, user_input);
}

C_RESULT ui_pad_update_user_input(input_state_t* input_state)
{
  ardrone_at_set_ui_value( user_input );

  return custom_update_user_input(input_state, user_input );
}

C_RESULT ui_pad_phi_trim( int32_t phi_trim )
{
	user_input &= ~(3 << ARDRONE_UI_BIT_TRIM_PHI);
	user_input |= (phi_trim + 1) << ARDRONE_UI_BIT_TRIM_PHI;

	return C_OK;
}

C_RESULT ui_pad_theta_trim( int32_t theta_trim )
{
	user_input &= ~(3 << ARDRONE_UI_BIT_TRIM_THETA);
	user_input |= (theta_trim + 1) << ARDRONE_UI_BIT_TRIM_THETA;

	return C_OK;
}

C_RESULT ui_pad_yaw_trim( int32_t yaw_trim )
{
	user_input &= ~(3 << ARDRONE_UI_BIT_TRIM_YAW);
	user_input |= (yaw_trim + 1) << ARDRONE_UI_BIT_TRIM_YAW;

	return C_OK;
}

uint32_t ui_get_user_input(void)
{
  return user_input;
}
