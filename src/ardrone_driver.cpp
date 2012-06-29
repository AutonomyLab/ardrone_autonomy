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
    hori_pub = image_transport.advertiseCamera("/ardrone/front/image_raw", 1);
	vert_pub = image_transport.advertiseCamera("/ardrone/bottom/image_raw", 1);
	navdata_pub = node_handle.advertise<ardrone_brown::Navdata>("/ardrone/navdata", 1);
	//toggleCam_sub = node_handle.subscribe("/ardrone/togglecam", 10, &toggleCamCallback);

	//int cam_state = DEFAULT_CAM_STATE; // 0 for forward and 1 for vertical, change to enum later
    //int set_navdata_demo_value = DEFAULT_NAVDATA_DEMO;  
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &set_navdata_demo_value, NULL);

//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detect_type, &detect_dtype, NULL);
//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_h, &detect_hori_type, NULL);
//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_v, &detect_vertfast_type, NULL);
//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_v_hsync, &detect_vert_type, NULL);
//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_colors, &detect_enemy_color, NULL );
//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (groundstripe_colors, &detect_groundstripes_color, NULL);
//	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_without_shell, &detect_indoor_hull, NULL);
	
	
	toggleCam_service = node_handle.advertiseService("/ardrone/togglecam", toggleCamCallback);
	toggleNavdataDemo_service = node_handle.advertiseService("/ardrone/togglenavdatademo", toggleNavdataDemoCallback);
	setCamChannel_service = node_handle.advertiseService("/ardrone/setcamchannel",setCamChannelCallback );
//	setEnemyColor_service = node_handle.advertiseService("/ardrone/setenemycolor", setEnemyColorCallback);
//	setHullType_service = node_handle.advertiseService("/ardrone/sethulltype", setHullTypeCallback);
}

ARDroneDriver::~ARDroneDriver()
{
}

void ARDroneDriver::run()
{
	ros::Rate loop_rate(40);

	int configWait = 250;
	bool configDone = false;
        
        //These are some extra params (experimental)
        int mCodec = P264_CODEC;
        int vbcMode = VBC_MODE_DYNAMIC;
        //int cam_state = DEFAULT_CAM_STATE; // 0 for forward and 1 for vertical, change to enum later
        //int set_navdata_demo_value = DEFAULT_NAVDATA_DEMO;  
        
	while (node_handle.ok())
	{
		// For some unknown reason, sometimes the ardrone critical configurations are not applied
		// when the commands are being sent during SDK initialization. This is a trick to send critical 
		// configurations sometime after SDK boots up.  
		if (configDone == false) 
		{
			configWait--;
			if (configWait == 0) 
			{
				configDone = true;
				fprintf(stderr, "\nSending some critical initial configuration after some delay...\n");
                //Ensure that the horizontal camera is running
                ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_channel, &cam_state, NULL);
                ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &set_navdata_demo_value, NULL);
                //ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_codec, &mCodec, NULL);
                //ARDRONE_TOOL_CONFIGURATION_ADDEVENT (bitrate_ctrl_mode, &vbcMode, NULL);					
				
				ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detect_type, &detect_dtype, NULL);
				ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_v, &detect_vert_type, NULL);
				ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_v_hsync, &detect_disable_placeholder, NULL);
				ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_h, &detect_hori_type, NULL);
				ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_colors, &detect_enemy_color, NULL );
				ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_without_shell, &detect_indoor_hull, NULL);
			}
		}
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
    if (IS_ARDRONE1)
    {
        /*
        * Information on buffer and image sizes.
        * Buffer is always in QVGA size, however for different Camera Modes
        * The picture and PIP sizes are different.
        * 
        * image_raw and buffer are always 320x240. In order to preserve backward compatibilty image_raw remains
        * always as before. Two new set of topics are added for two new cameras : /ardrone/front/xxx and /ardrone/bottom/xxx
        * 
        * In Camera State 0 front image relays the buffer  and image_raw and bottom image are not updated.
        * 
        * In Camera State 1 bottom image is a 174x144 crop of the buffer. The front image is not updated
        * 
        * In Camera State 2 bottom image is a PIP cut of size (87x72) from buffer.
        * The bottom image is a (320-87)x(240) cut of the buffer.
        * 
        * In Camera State 3 front image is a PIP cut of size (58x42) from buffer.
        * The bottom image is a (174-58)x144 crop of the buffer. 
        */
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo cinfo_msg;
        sensor_msgs::Image::_data_type::iterator _it;

        image_msg.width = D1_STREAM_WIDTH;
        image_msg.height = D1_STREAM_HEIGHT;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = D1_STREAM_WIDTH*3;
        image_msg.data.resize(D1_STREAM_WIDTH*D1_STREAM_HEIGHT*3);
        std::copy(buffer, buffer+(D1_STREAM_WIDTH*D1_STREAM_HEIGHT*3), image_msg.data.begin());

        // We only put the width and height in here.
        cinfo_msg.width = D1_STREAM_WIDTH;
        cinfo_msg.height = D1_STREAM_HEIGHT;
        image_pub.publish(image_msg, cinfo_msg);
        if (cam_state == ZAP_CHANNEL_HORI)
        {
            /*
            * Horizontal camera is activated, only /ardrone/front/ is being updated 
            */
            hori_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            /*
            * Vertical camera is activated, only /ardrone/bottom/ is being updated 
            */
            image_msg.width = D1_VERTSTREAM_WIDTH;
            image_msg.height = D1_VERTSTREAM_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_VERTSTREAM_WIDTH*3;
            image_msg.data.clear();
            image_msg.data.resize(D1_VERTSTREAM_WIDTH*D1_VERTSTREAM_HEIGHT*3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_VERTSTREAM_HEIGHT ; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_VERTSTREAM_WIDTH;
            cinfo_msg.height = D1_VERTSTREAM_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_LARGE_HORI_SMALL_VERT)
        {
            /*
            * The Picture in Picture is activated with vertical camera inside the horizontal
            * camera. Both /ardrone/front and /ardrone/bottom is being updated
            */

            // Front (Cropping the first 88 columns)
            image_msg.width = D1_STREAM_WIDTH  - D1_MODE2_PIP_WIDTH;
            image_msg.height = D1_STREAM_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = (D1_STREAM_WIDTH-D1_MODE2_PIP_WIDTH)*3;
            image_msg.data.clear();
            image_msg.data.resize((D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH)*D1_STREAM_HEIGHT*3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_STREAM_HEIGHT; row++)
            {
                int _b = (row * D1_STREAM_WIDTH * 3) + (D1_MODE2_PIP_WIDTH * 3);
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH;
            cinfo_msg.height = D1_STREAM_HEIGHT;
            hori_pub.publish(image_msg, cinfo_msg);

            //Bottom
            image_msg.width = D1_MODE2_PIP_WIDTH;
            image_msg.height = D1_MODE2_PIP_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_MODE2_PIP_WIDTH * 3;
            image_msg.data.clear();
            image_msg.data.resize(D1_MODE2_PIP_WIDTH * D1_MODE2_PIP_HEIGHT * 3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_MODE2_PIP_HEIGHT; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_MODE2_PIP_WIDTH;
            cinfo_msg.height = D1_MODE2_PIP_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_LARGE_VERT_SMALL_HORI)
        {
            /*
            * The Picture in Picture is activated with horizontal camera inside the vertical
            * camera. Both /ardrone/front and /ardrone/bottom is being updated
            */

            // Bottom  (Cropping the first 58 columns)
            image_msg.width = D1_VERTSTREAM_WIDTH   - D1_MODE3_PIP_WIDTH;
            image_msg.height = D1_VERTSTREAM_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = (D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH)*3;
            image_msg.data.clear();
            image_msg.data.resize((D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH)* D1_VERTSTREAM_HEIGHT*3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_VERTSTREAM_HEIGHT; row++)
            {
                int _b = (row * (D1_STREAM_WIDTH * 3)) + (D1_MODE3_PIP_WIDTH * 3);
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH;
            cinfo_msg.height = D1_VERTSTREAM_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg);

            //Front
            image_msg.width = D1_MODE3_PIP_WIDTH;
            image_msg.height = D1_MODE3_PIP_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_MODE3_PIP_WIDTH * 3;
            image_msg.data.clear();
            image_msg.data.resize(D1_MODE3_PIP_WIDTH * D1_MODE3_PIP_HEIGHT * 3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_MODE3_PIP_HEIGHT; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_MODE3_PIP_WIDTH;
            cinfo_msg.height = D1_MODE3_PIP_HEIGHT;
            hori_pub.publish(image_msg, cinfo_msg);
        }
    }
    
    /**
     * For Drone 2 w/ SDK2. Both camera streams are 360p.
     * No 720p support for now.
     * SDK 2.0 Does not support PIP.
     */
    if (IS_ARDRONE2)
    {
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo cinfo_msg;
        sensor_msgs::Image::_data_type::iterator _it;

        image_msg.width = D2_STREAM_WIDTH;
        image_msg.height = D2_STREAM_HEIGHT;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = D2_STREAM_WIDTH*3;
        image_msg.data.resize(D2_STREAM_WIDTH*D2_STREAM_HEIGHT*3);
        std::copy(buffer, buffer+(D2_STREAM_WIDTH*D2_STREAM_HEIGHT*3), image_msg.data.begin());

        // We only put the width and height in here.
        cinfo_msg.width = D2_STREAM_WIDTH;
        cinfo_msg.height = D2_STREAM_HEIGHT;
        image_pub.publish(image_msg, cinfo_msg); // /ardrone
        if (cam_state == ZAP_CHANNEL_HORI)
        {
            /*
            * Horizontal camera is activated, only /ardrone/front/ is being updated 
            */
            hori_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            /*
            * Vertical camera is activated, only /ardrone/bottom/ is being updated 
            */
            vert_pub.publish(image_msg, cinfo_msg);
        }
    }
	
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
	msg.ax = navdata_phys.phys_accs[ACC_X] / 1000.0; // g
	msg.ay = -navdata_phys.phys_accs[ACC_Y] / 1000.0; // g
	msg.az = -navdata_phys.phys_accs[ACC_Z] / 1000.0; // g
	
	// Tag Detection
	msg.tags_count = navdata_detect.nb_detected;
	for (int i = 0; i < navdata_detect.nb_detected; i++)
	{
		/*
		 * The tags_type is in raw format. In order to extract the information 
		 * macros from ardrone_api.h is needed.
		 *
		 * #define DETECTION_MAKE_TYPE(source,tag) ( ((source)<<16) | (tag) )
		 * #define DETECTION_EXTRACT_SOURCE(type)  ( ((type)>>16) & 0x0FF )
		 * #define DETECTION_EXTRACT_TAG(type)     ( (type) & 0x0FF )
		 * 
		 * Please also note that the xc, yc, width and height are in [0,1000] range
		 * and must get converted back based on image resolution.
		 */
		msg.tags_type.push_back(navdata_detect.type[i]);
		msg.tags_xc.push_back(navdata_detect.xc[i]);
		msg.tags_yc.push_back(navdata_detect.yc[i]);
		msg.tags_width.push_back(navdata_detect.width[i]);
		msg.tags_height.push_back(navdata_detect.height[i]);
		msg.tags_orientation.push_back(navdata_detect.orientation_angle[i]);
		msg.tags_distance.push_back(navdata_detect.dist[i]);
	}
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

//extern "C" int custom_main(int argc, char** argv)
int main(int argc, char** argv)
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
		//ardrone_tool_init(argc, argv);
        ros::init(argc, argv, "ardrone_driver");
        ardrone_tool_main(argc, argv, 0);
		ARDroneDriver().run();
        
	}

	return 0;
}

