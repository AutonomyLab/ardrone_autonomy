#include "ardrone_driver.h"
#include "teleop_twist.h"
#include "video.h"

////////////////////////////////////////////////////////////////////////////////
// class ARDroneDriver
////////////////////////////////////////////////////////////////////////////////

ARDroneDriver::ARDroneDriver()
	: image_transport(node_handle)
{
	cmd_vel_sub = node_handle.subscribe("/cmd_vel", 1, &cmdVelCallback);
	takeoff_sub = node_handle.subscribe("/ardrone/takeoff", 1, &takeoffCallback);
	reset_sub = node_handle.subscribe("/ardrone/reset", 1, &resetCallback);
	land_sub = node_handle.subscribe("/ardrone/land", 1, &landCallback);
	image_pub = image_transport.advertiseCamera("/ardrone/image_raw", 1);
	navdata_pub = node_handle.advertise<ardrone_brown::Navdata>("/ardrone/navdata", 1);
	//toggleCam_sub = node_handle.subscribe("/ardrone/togglecam", 10, &toggleCamCallback);

#ifdef _USING_SDK_1_7_
	//Ensure that the horizontal camera is running
	int cam_state = 0; // horizontal
	int set_navdata_demo_value = 0;
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &set_navdata_demo_value, NULL);
#else
	//Ensure that the horizontal camera is running
	ardrone_at_set_toy_configuration("video:video_channel","0");
#endif

	toggleCam_service = node_handle.advertiseService("/ardrone/togglecam", toggleCamCallback);


}

ARDroneDriver::~ARDroneDriver()
{
}

void ARDroneDriver::run()
{
	ros::Rate loop_rate(40);

	while (node_handle.ok())
	{
		if (current_frame_id != last_frame_id)
		{
			publish_video();
			publish_navdata();
			last_frame_id = current_frame_id;
		}

		ardrone_tool_update();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ARDroneDriver::publish_video()
{
	sensor_msgs::Image image_msg;
	sensor_msgs::CameraInfo cinfo_msg;

	image_msg.width = STREAM_WIDTH;
	image_msg.height = STREAM_HEIGHT;
	image_msg.encoding = "rgb8";
	image_msg.is_bigendian = false;
	image_msg.step = STREAM_WIDTH*3;
	image_msg.data.resize(STREAM_WIDTH*STREAM_HEIGHT*3);
	std::copy(buffer, buffer+(STREAM_WIDTH*STREAM_HEIGHT*3), image_msg.data.begin());

	// We only put the width and height in here.
	cinfo_msg.width = STREAM_WIDTH;
	cinfo_msg.height = STREAM_HEIGHT;

	image_pub.publish(image_msg, cinfo_msg);
}

void ARDroneDriver::publish_navdata()
{
	ardrone_brown::Navdata msg;

	msg.batteryPercent = navdata.vbat_flying_percentage;

	// positive means counterclockwise rotation around axis
	msg.rotX = navdata.phi / 1000.0; // tilt left/right
	msg.rotY = -navdata.theta / 1000.0; // tilt forward/backward
	msg.rotZ = -navdata.psi / 1000.0; // orientation

	msg.altd = navdata.altitude; // cm
	msg.vx = navdata.vx; // mm/sec
	msg.vy = -navdata.vy; // mm/sec
	msg.vz = -navdata.vz; // mm/sec

	msg.tm = arnavtime.time;

	// TODO: Ideally we would be able to figure out whether we are in an emergency state
	// using the navdata.ctrl_state bitfield with the ARDRONE_EMERGENCY_MASK flag, but
	// it seems to always be 0.  The emergency state seems to be correlated with the
	// inverse of the ARDRONE_TIMER_ELAPSED flag, but that just makes so little sense
	// that I don't want to use it because it's probably wrong.  So we'll just use a
	// manual reset for now.

	navdata_pub.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
// custom_main
////////////////////////////////////////////////////////////////////////////////

extern "C" int custom_main(int argc, char** argv)
{
	int res = ardrone_tool_setup_com( NULL );

	if( FAILED(res) )
	{
		printf("Wifi initialization failed. It means either:\n");
		printf("\t* you're not root (it's mandatory because you can set up wifi connection only as root)\n");
		printf("\t* wifi device is not present (on your pc or on your card)\n");
		printf("\t* you set the wrong name for wifi interface (for example rausb0 instead of wlan0) \n");
		printf("\t* ap is not up (reboot card or remove wifi usb dongle)\n");
		printf("\t* wifi device has no antenna\n");
	}
	else
	{
		ardrone_tool_init(argc, argv);
		ros::init(argc, argv, "ardrone_driver");

		ARDroneDriver().run();
	}

	return 0;
}

