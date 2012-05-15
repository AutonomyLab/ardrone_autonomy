#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_assert.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/UI/ardrone_tool_ui.h>

static input_device_t* devices[MAX_NUM_DEVICES];

static input_state_t input_state = { 0 };

C_RESULT ardrone_tool_input_add( input_device_t* device )
{
  C_RESULT res;
  int32_t i;

  VP_OS_ASSERT( device != NULL );

  res = C_FAIL;
  i   = 0;

  while( i < MAX_NUM_DEVICES && devices[i] != NULL ) i++;

  if( i < MAX_NUM_DEVICES )
  {
    if( VP_SUCCEEDED(device->init()) )
    {
      devices[i] = device;
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

static C_RESULT ardrone_tool_input_remove_i( int32_t i )
{
  C_RESULT res;

  res = C_OK;

  if( devices[i] != NULL )
  {
    if( VP_SUCCEEDED(devices[i]->shutdown()) )
    {
      PRINT("Input device %s removed\n", devices[i]->name);
      res = C_OK;
    }
    else
    {
      PRINT("Input device %s removed but an error occured during its shutdown\n", devices[i]->name);
      res = C_FAIL;
    }

    devices[i] = NULL;
  }

  return res;
}

C_RESULT ardrone_tool_input_remove( input_device_t* device )
{
  C_RESULT res;
  int32_t i;

  VP_OS_ASSERT( device != NULL );

  res = C_FAIL;
  i   = 0;

  while( i < MAX_NUM_DEVICES && devices[i] != device ) i++;

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
	input_state.ab = value;
	return ui_pad_ab(value);
}

C_RESULT ardrone_tool_set_ui_pad_ag(int32_t value)
{
	input_state.ag = value;
	return ui_pad_ag(value);
}

C_RESULT ardrone_tool_set_ui_pad_ad(int32_t value)
{
	input_state.ad = value;
	return ui_pad_ad(value);
}

C_RESULT ardrone_tool_set_ui_pad_ah(int32_t value)
{
	input_state.ah = value;
	return ui_pad_ah(value);
}

C_RESULT ardrone_tool_set_ui_pad_l1(int32_t value)
{
	input_state.l1 = value;
	return ui_pad_l1(value);
}

C_RESULT ardrone_tool_set_ui_pad_r1(int32_t value)
{
	input_state.r1 = value;
	return ui_pad_r1(value);
}

C_RESULT ardrone_tool_set_ui_pad_l2(int32_t value)
{
	input_state.l2 = value;
	return ui_pad_l2(value);
}

C_RESULT ardrone_tool_set_ui_pad_r2(int32_t value)
{
	input_state.r2 = value;
	return ui_pad_r2(value);
}

C_RESULT ardrone_tool_set_ui_pad_select(int32_t value)
{
	input_state.select = value;
	return ui_pad_select(value);
}


C_RESULT ardrone_tool_set_ui_pad_start(int32_t value)
{
	input_state.start = value;
	return ui_pad_start_stop(value);
}

C_RESULT ardrone_tool_set_ui_pad_xy(int32_t x, int32_t y)
{
  input_state.x   = x;
  input_state.y   = y;

  return ui_pad_xy_change(x, y);
}

C_RESULT ardrone_tool_input_init(void)
{
  int32_t i;

  i   = 0;

  while( i < MAX_NUM_DEVICES )
  {
    devices[i] = NULL;
    i++;
  }

  return ardrone_tool_input_reset();
}

C_RESULT ardrone_tool_input_reset(void)
{
  return ui_pad_reset_user_input(&input_state);
}

C_RESULT ardrone_tool_start_reset(void)
{
  input_state.start    = 0;

  return ui_pad_reset_user_input_start(&input_state);
}

C_RESULT ardrone_tool_input_update(void)
{
  C_RESULT res;
  int32_t i;

  res = C_OK;
  i   = 0;

  while( VP_SUCCEEDED(res) && i < MAX_NUM_DEVICES )
  {
    if( devices[i] != NULL && VP_FAILED(devices[i]->update()) )
    {
      PRINT("Input device %s update failed... it'll be removed\n", devices[i]->name);
      ardrone_tool_input_remove_i(i);

      res = C_FAIL;
    }
    i++;
  }

  ui_pad_update_user_input(&input_state);

  return res;
}

C_RESULT ardrone_tool_input_shutdown(void)
{
  int32_t i;

  i   = 0;

  while( i < MAX_NUM_DEVICES )
  {
    ardrone_tool_input_remove_i(i);
    i++;
  }

  return C_OK;
}

input_state_t* ardrone_tool_get_input_state( void )
{
   return &input_state;
}


C_RESULT custom_reset_user_input(input_state_t* input_state, uint32_t user_input ) { return C_OK; }
C_RESULT custom_update_user_input(input_state_t* input_state, uint32_t user_input ) { return C_OK; }
