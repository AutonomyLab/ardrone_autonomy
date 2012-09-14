#include "teleop_twist.h"
#include "ardrone_autonomy/LedAnim.h"

inline float max(float a, float b) { return a > b ? a : b; }
inline float min(float a, float b) { return a < b ? a : b; }

bool needs_takeoff = false;
bool needs_land = false;
bool needs_reset = false;
geometry_msgs::Twist cmd_vel;
float old_left_right = -10.0;
float old_front_back = -10.0;
float old_up_down = -10.0;
float old_turn = -10.0;

int cam_state = DEFAULT_CAM_STATE; // 0 for forward and 1 for vertical, change to enum later
int set_navdata_demo_value = DEFAULT_NAVDATA_DEMO; 
int32_t detect_enemy_color = ARDRONE_DETECTION_COLOR_ORANGE_YELLOW;
int32_t detect_dtype = CAD_TYPE_MULTIPLE_DETECTION_MODE;
int32_t detect_hori_type = TAG_TYPE_MASK(TAG_TYPE_SHELL_TAG_V2);
int32_t detect_vert_type = TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL);
int32_t detect_indoor_hull = 0;
int32_t detect_disable_placeholder = 0;
int32_t detect_enable_placeholder = 1;

const LED_ANIMATION_IDS ledAnimMap[14] = {
	BLINK_GREEN_RED, BLINK_GREEN, BLINK_RED, BLINK_ORANGE,
	SNAKE_GREEN_RED, FIRE, STANDARD, RED, GREEN, RED_SNAKE,BLANK,
	LEFT_GREEN_RIGHT_RED, LEFT_RED_RIGHT_GREEN, BLINK_STANDARD};

bool toggleNavdataDemoCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    set_navdata_demo_value ^= 1;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &set_navdata_demo_value, NULL);
    fprintf(stderr, "\nToggling navdata_demo, set to %d.\n", set_navdata_demo_value);
    return true;
}

//ros service callback to set the camera channel
//TODO: add input check
bool setCamChannelCallback(ardrone_autonomy::CamSelect::Request& request, ardrone_autonomy::CamSelect::Response& response)
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

bool setLedAnimationCallback(ardrone_autonomy::LedAnim::Request& request, ardrone_autonomy::LedAnim::Response& response)
{
    LED_ANIMATION_IDS anim_id = ledAnimMap[request.type % 14]; // Don't trick me
    vp_os_mutex_lock(&twist_lock);
    ardrone_at_set_led_animation(anim_id, (float) fabs(request.freq), (uint32_t) abs(request.duration));
    vp_os_mutex_unlock(&twist_lock);
    response.result = true;
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
    vp_os_mutex_lock(&twist_lock);
	cmd_vel.linear.x  = max(min(-msg->linear.x, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.y  = max(min(-msg->linear.y, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.z  = max(min(msg->linear.z, 1.0), -1.0);
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = max(min(-msg->angular.z, 1.0), -1.0);
    vp_os_mutex_unlock(&twist_lock);
}

void landCallback(const std_msgs::Empty &msg)
{
    vp_os_mutex_lock(&twist_lock);
    needs_land = true;
    vp_os_mutex_unlock(&twist_lock);
}

void resetCallback(const std_msgs::Empty &msg)
{	
    vp_os_mutex_lock(&twist_lock);
    needs_reset = true;
    vp_os_mutex_unlock(&twist_lock);
}

void takeoffCallback(const std_msgs::Empty &msg)
{
    vp_os_mutex_lock(&twist_lock);
    needs_takeoff = true;
    vp_os_mutex_unlock(&twist_lock);
}

C_RESULT open_teleop(void)
{
	return C_OK;
}

C_RESULT update_teleop(void)
{
	// This function *toggles* the emergency state, so we only want to toggle the emergency
	// state when we are in the emergency state (because we want to get out of it).
    vp_os_mutex_lock(&twist_lock);
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

        float left_right = (float) cmd_vel.linear.y;
        float front_back = (float) cmd_vel.linear.x;
        float up_down = (float) cmd_vel.linear.z;
        float turn = (float) cmd_vel.angular.z;
        
        bool is_changed = !(
                (fabs(left_right - old_left_right) < _EPS) && 
                (fabs(front_back - old_front_back) < _EPS) && 
                (fabs(up_down - old_up_down) < _EPS) && 
                (fabs(turn - old_turn) < _EPS)
                );
        
        // These lines are for testing, they should be moved to configurations
        // Bit 0 of control_flag == 0: should we hover?
        // Bit 1 of control_flag == 1: should we use combined yaw mode?
        
        int32_t control_flag = 0x00;
        int32_t combined_yaw = 0x00;
        
        int32_t hover = (int32_t) 
                !(
                (fabs(left_right) < _EPS) && 
                (fabs(front_back) < _EPS) && 
                (fabs(up_down) < _EPS) && 
                (fabs(turn) < _EPS)
                );
        control_flag |= (hover << 0);
        control_flag |= (combined_yaw << 1);
        //ROS_INFO (">>> Control Flag: %d", control_flag);
        
        old_left_right = left_right;
        old_front_back = front_back;
        old_up_down = up_down;
        old_turn = turn;
        //is_changed = true;
        if ((is_changed) || (hover))
        {
            ardrone_tool_set_progressive_cmd(control_flag, left_right, front_back, up_down, turn, 0.0, 0.0);
        }

    }
    vp_os_mutex_unlock(&twist_lock);
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

