#include "teleop_twist.h"

inline float max(float a, float b) { return a > b ? a : b; }
inline float min(float a, float b) { return a < b ? a : b; }

bool is_flying = false;
bool needs_reset = false;
geometry_msgs::Twist cmd_vel;

int cam_state=0; // 0 for forward and 1 for vertical, change to enum later

// ros service callback function for toggling Cam
bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (cam_state == 0) // toggle to 1, the vertical camera
    {
      cam_state = 1;

#ifdef _USING_SDK_1_7_
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
#else
      ardrone_at_set_toy_configuration("video:video_channel","1");
#endif

      fprintf(stderr, "\nToggling from frontal camera to vertical camera.\n");
    }
  else if (cam_state == 1) // toggle to the forward camera
    {
      cam_state = 0;

#ifdef _USING_SDK_1_7_
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
#else
      ardrone_at_set_toy_configuration("video:video_channel","0");
#endif

      fprintf(stderr, "\nToggling from vertical camera to frontal camera.\n");      
    }
  return true;
}


/*
// Older rostopic callback function for toggling Cam
void toggleCamCallback(const std_msgs::Empty &msg)
{
  if (cam_state == 0) // toggle to 1, the vertical camera
    {
      cam_state = 1;
      ardrone_at_set_toy_configuration("video:video_channel","1");
      fprintf(stderr, "\nToggling from frontal camera to vertical camera.\n");
    }
  else if (cam_state == 1) // toggle to the forward camera
    {
      cam_state = 0;
      ardrone_at_set_toy_configuration("video:video_channel","0");
      fprintf(stderr, "\nToggling from vertical camera to frontal camera.\n");      
    }
}
*/

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
	const float maxHorizontalSpeed = 1; // use 0.1f for testing and 1 for the real thing
	cmd_vel.linear.x  = max(min(-msg->linear.x, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.y  = max(min(-msg->linear.y, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.z  = max(min(msg->linear.z, 1), -1);
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = max(min(-msg->angular.z, 1), -1);
}

void landCallback(const std_msgs::Empty &msg)
{
	is_flying = false;
}

void resetCallback(const std_msgs::Empty &msg)
{
	needs_reset = true;
}

void takeoffCallback(const std_msgs::Empty &msg)
{
	is_flying = true;
}

C_RESULT open_teleop(void)
{
	return C_OK;
}

C_RESULT update_teleop(void)
{
	// This function *toggles* the emergency state, so we only want to toggle the emergency
	// state when we are in the emergency state (because we want to get out of it).
	ardrone_tool_set_ui_pad_select(needs_reset);
	needs_reset = false;

	// This function sets whether or not the robot should be flying.  If it is flying and you
	// send 0, the robot will slow down the motors and slowly descend to the floor.
	ardrone_tool_set_ui_pad_start(is_flying);

	float left_right = cmd_vel.linear.y;
	float front_back = cmd_vel.linear.x;
	float up_down = cmd_vel.linear.z;
	float turn = cmd_vel.angular.z;

	ardrone_at_set_progress_cmd(1, left_right, front_back, up_down, turn);
	return C_OK;
}

C_RESULT close_teleop(void)
{
	return C_OK;
}

input_device_t teleop = {
	"Teleop",
	open_teleop,
	update_teleop,
	close_teleop
};

