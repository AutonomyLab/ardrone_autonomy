#include "teleop_twist.h"

inline float max(float a, float b) { return a > b ? a : b; }
inline float min(float a, float b) { return a < b ? a : b; }

bool needs_takeoff = false;
bool needs_land = false;
bool needs_reset = false;
geometry_msgs::Twist cmd_vel;

int cam_state = DEFAULT_CAM_STATE; // 0 for forward and 1 for vertical, change to enum later
int set_navdata_demo_value = DEFAULT_NAVDATA_DEMO; 
int32_t detect_enemy_color = ARDRONE_DETECTION_COLOR_ORANGE_YELLOW;
int32_t detect_dtype = CAD_TYPE_MULTIPLE_DETECTION_MODE;
int32_t detect_hori_type = TAG_TYPE_MASK(TAG_TYPE_SHELL_TAG_V2);
int32_t detect_vert_type = TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL);
int32_t detect_indoor_hull = 0;
int32_t detect_disable_placeholder = 0;
int32_t detect_enable_placeholder = 1;

bool toggleNavdataDemoCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    set_navdata_demo_value ^= 1;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &set_navdata_demo_value, NULL);
    fprintf(stderr, "\nToggling navdata_demo, set to %d.\n", set_navdata_demo_value);
    return true;
}

//ros service callback to set the camera channel
//TODO: add input check
bool setCamChannelCallback(ardrone_brown::CamSelect::Request& request, ardrone_brown::CamSelect::Response& response)
{
    cam_state = request.channel;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
    fprintf(stderr, "\nSetting camera channel to : %d.\n", cam_state);
    response.result = true;
    return true;
}
// ros service callback function for toggling Cam
bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    int _modes = (IS_ARDRONE1) ? 4 : 2;
    cam_state = (cam_state + 1) % _modes;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
    fprintf(stderr, "\nSetting camera channel to : %d.\n", cam_state);
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
	const double maxHorizontalSpeed = 1.0; // use 0.1f for testing and 1 for the real thing
	cmd_vel.linear.x  = max(min(-msg->linear.x, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.y  = max(min(-msg->linear.y, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.z  = max(min(msg->linear.z, 1.0), -1.0);
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = max(min(-msg->angular.z, 1.0), -1.0);
}

void landCallback(const std_msgs::Empty &msg)
{    
    needs_land = true;
}

void resetCallback(const std_msgs::Empty &msg)
{	
    needs_reset = true;
}

void takeoffCallback(const std_msgs::Empty &msg)
{
    
    needs_takeoff = true;
}

C_RESULT open_teleop(void)
{
	return C_OK;
}

C_RESULT update_teleop(void)
{
	// This function *toggles* the emergency state, so we only want to toggle the emergency
	// state when we are in the emergency state (because we want to get out of it).
    static int i = 0;
    if (needs_reset)
    {
        ardrone_tool_set_ui_pad_select(1);
        needs_reset = false;
    }
    else if (needs_takeoff)
    {
        ardrone_tool_set_ui_pad_start(1);
        needs_takeoff = false;
    }
    else if (needs_land)
    {
        ardrone_tool_set_ui_pad_start(0);
        needs_land = false;
    }
    else
    {
        // This function sets whether or not the robot should be flying.  If it is flying and you
        // send 0, the robot will slow down the motors and slowly descend to the floor.
        //ardrone_tool_set_ui_pad_start(is_flying);

        float left_right = (float) cmd_vel.linear.y;
        float front_back = (float) cmd_vel.linear.x;
        float up_down = (float) cmd_vel.linear.z;
        float turn = (float) cmd_vel.angular.z;

        //ardrone_at_set_progress_cmd(1, left_right, front_back, up_down, turn);
        //printf(">>> To Send: L<%6.4f,%6.4f,%6.4f> A<%6.4f>\n", front_back, left_right, up_down, turn);
        ardrone_tool_set_progressive_cmd(1, left_right, front_back, up_down, turn, 0.0, 0.0);
        
//        ardrone_tool_set_progressive_cmd(1, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0);
//        if ((i++ % 100) == 0) 
//        {
//            ardrone_tool_set_progressive_cmd(1, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
//        }
//        else
//        {
//            ardrone_tool_set_progressive_cmd(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//        }
    }
    
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

