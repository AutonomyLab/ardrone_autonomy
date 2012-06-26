#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_assert.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/ardrone_version.h>

static input_device_t* devices_list[MAX_NUM_DEVICES];

static input_state_t input_state = { 0 };

C_RESULT ardrone_tool_input_add( input_device_t* device )
{
  C_RESULT res;
  int32_t i;

  VP_OS_ASSERT( device != NULL );

  res = C_FAIL;
  i   = 0;

  while( i < MAX_NUM_DEVICES && devices_list[i] != NULL ) i++;

  if( i < MAX_NUM_DEVICES )
  {
    if( VP_SUCCEEDED(device->init()) )
    {
      devices_list[i] = device;
      PRINT("Input device %s added\n", device->name);
      res = C_OK;
    }
    else
    {
      PRINT("Input device %s init failed\n", device->name);
      res = C_FAIL;
    }
  }
  else
  {
    PRINT("Not enough memory to add input device %s\n", device->name);
  }

  return res;
}

static C_RESULT ardrone_tool_input_remove_i( uint32_t i )
{
  C_RESULT res;

  res = C_OK;

  if( i < MAX_NUM_DEVICES )
  {
      if( devices_list[i] != NULL )
    {
        if( VP_SUCCEEDED(devices_list[i]->shutdown()) )
        {
          PRINT("Input device %s removed\n", devices_list[i]->name);
      res = C_OK;
    }
    else
    {
          PRINT("Input device %s removed but an error occured during its shutdown\n", devices_list[i]->name);
      res = C_FAIL;
    }

        devices_list[i] = NULL;
      }
  } else {
    res = C_FAIL;
    PRINT("Input device index out of range\n");
  }

  return res;
}

C_RESULT ardrone_tool_input_remove( input_device_t* device )
{
  C_RESULT res;
  uint32_t i;

  VP_OS_ASSERT( device != NULL );

  res = C_FAIL;
  i   = 0;

  while( i < MAX_NUM_DEVICES && devices_list[i] != device ) i++;

  if( i < MAX_NUM_DEVICES )
  {
    res = ardrone_tool_input_remove_i(i);
  }
  else
  {
    DEBUG_PRINT_SDK("Input %s not found while removing\n", device->name);
  }

  return res;
}

C_RESULT ardrone_tool_set_ui_pad_ab(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_AB);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_AB);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_ag(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_AG);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_AG);
	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_ad(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_AD);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_AD);
	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_ah(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_AH);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_AH);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_l1(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_L1);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_L1);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_r1(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_R1);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_R1);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_l2(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_L2);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_L2);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_r2(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_R2);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_R2);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_select(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_SELECT);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_SELECT);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_start(int32_t value)
{
	if( value )
		input_state.user_input |= (1 << ARDRONE_UI_BIT_START);
	else
		input_state.user_input &= ~(1 << ARDRONE_UI_BIT_START);

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_xy(int32_t x, int32_t y)
{
	input_state.user_input &= ~(3 << ARDRONE_UI_BIT_X);
	input_state.user_input &= ~(3 << ARDRONE_UI_BIT_Y);

	input_state.user_input |= (x + 1) << ARDRONE_UI_BIT_X;
	input_state.user_input |= (y + 1) << ARDRONE_UI_BIT_Y;

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_phi_trim( int32_t phi_trim )
{
	input_state.user_input &= ~(3 << ARDRONE_UI_BIT_TRIM_PHI);
	input_state.user_input |= (phi_trim + 1) << ARDRONE_UI_BIT_TRIM_PHI;

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_theta_trim( int32_t theta_trim )
{
	input_state.user_input &= ~(3 << ARDRONE_UI_BIT_TRIM_THETA);
	input_state.user_input |= (theta_trim + 1) << ARDRONE_UI_BIT_TRIM_THETA;

	return C_OK;
}

C_RESULT ardrone_tool_set_ui_pad_yaw_trim( int32_t yaw_trim )
{
	input_state.user_input &= ~(3 << ARDRONE_UI_BIT_TRIM_YAW);
	input_state.user_input |= (yaw_trim + 1) << ARDRONE_UI_BIT_TRIM_YAW;

	return C_OK;
}

C_RESULT ardrone_tool_set_progressive_cmd(int32_t flag, float32_t phi, float32_t theta, float32_t gaz, float32_t yaw, float32_t psi, float32_t psi_accuracy)
{
	input_state.pcmd.flag 	= flag;
	input_state.pcmd.phi 	= phi;
	input_state.pcmd.theta 	= theta;
	input_state.pcmd.gaz 	= gaz;
	input_state.pcmd.yaw 	= yaw;
	input_state.pcmd.psi	= psi;
	input_state.pcmd.psi_accuracy = psi_accuracy;

	return C_OK;
}

C_RESULT ardrone_tool_input_init(void)
{
  int32_t i;

  i   = 0;

  while( i < MAX_NUM_DEVICES )
  {
    devices_list[i] = NULL;
    i++;
  }

  return ardrone_tool_input_reset();
}

C_RESULT ardrone_tool_input_reset(void)
{
	vp_os_memset(&input_state, 0, sizeof(input_state_t));
	ardrone_tool_set_ui_pad_phi_trim(0);
	ardrone_tool_set_ui_pad_yaw_trim(0);
	ardrone_tool_set_ui_pad_theta_trim(0);
	ardrone_tool_set_ui_pad_xy(0,0);
	return C_OK;
}

C_RESULT ardrone_tool_input_start_reset(void)
{
	ardrone_tool_set_ui_pad_start(0);

	return C_OK;
}

C_RESULT ardrone_tool_input_update(void)
{
  C_RESULT res;
  uint32_t i;

  res = C_OK;
  i   = 0;

  while( VP_SUCCEEDED(res) && i < MAX_NUM_DEVICES )
  {
    if( devices_list[i] != NULL && VP_FAILED(devices_list[i]->update()) )
    {
      PRINT("Input device %s update failed... it'll be removed\n", devices_list[i]->name);
      ardrone_tool_input_remove_i(i);

      res = C_FAIL;
    }
    i++;
  }

  if(IS_ARDRONE1)
  {
	  ardrone_at_set_progress_cmd(input_state.pcmd.flag, input_state.pcmd.phi, input_state.pcmd.theta, input_state.pcmd.gaz, input_state.pcmd.yaw);
  }
  else
  {
	  ardrone_at_set_progress_cmd_with_magneto(input_state.pcmd.flag, input_state.pcmd.phi, input_state.pcmd.theta, input_state.pcmd.gaz, input_state.pcmd.yaw, input_state.pcmd.psi, input_state.pcmd.psi_accuracy);
  }

  ardrone_at_set_ui_pad_value( input_state.user_input );

  return res;
}

C_RESULT ardrone_tool_input_shutdown(void)
{
  uint32_t i;

  i   = 0;

  while( i < MAX_NUM_DEVICES )
  {
    ardrone_tool_input_remove_i(i);
    i++;
  }

  return C_OK;
}

input_state_t* ardrone_tool_input_get_state( void )
{
   return &input_state;
}
