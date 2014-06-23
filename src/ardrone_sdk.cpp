#include <ardrone_autonomy/ardrone_sdk.h>
#include <ardrone_autonomy/video.h>
#include <ardrone_autonomy/teleop_twist.h>


navdata_unpacked_t *shared_raw_navdata;
ros::Time shared_navdata_receive_time;

vp_os_mutex_t navdata_lock;
vp_os_mutex_t video_lock;
vp_os_mutex_t twist_lock;

long int current_navdata_id = 0;

ARDroneDriver* rosDriver;

int32_t looprate;
bool realtime_navdata;
bool realtime_video;

int32_t should_exit;

extern "C" {
    vp_stages_latency_estimation_config_t vlat;


    DEFINE_THREAD_ROUTINE(update_ros, data)
    {
        PRINT("Thread `update_ros` started \n ");
        ARDroneDriver* driver = (ARDroneDriver *) data;
        driver->run();
        return (THREAD_RET) 0;
    }
    
	 C_RESULT ardrone_tool_init_custom(void) 
     {
     should_exit = 0;
     vp_os_mutex_init(&navdata_lock);
     vp_os_mutex_init(&video_lock);
     vp_os_mutex_init(&twist_lock);

     rosDriver = new ARDroneDriver();
     int _w, _h;
        
        if (IS_ARDRONE2)
        {
            ardrone_application_default_config.video_codec = H264_360P_CODEC;
            _w = D2_STREAM_WIDTH;
            _h = D2_STREAM_HEIGHT;
        }
        else if (IS_ARDRONE1)
        {
            ardrone_application_default_config.video_codec = UVLC_CODEC;
            _w = D1_STREAM_WIDTH;
            _h = D1_STREAM_HEIGHT;
                   
        }
        else
        {
            printf("Something must be really wrong with the SDK!");
        }

        ros::param::param("~looprate",looprate,50);
        ros::param::param("~realtime_navdata",realtime_navdata,false);
        ros::param::param("~realtime_video",realtime_video,false);

        // SET SOME NON-STANDARD DEFAULT VALUES FOR THE DRIVER
        // THESE CAN BE OVERWRITTEN BY ROS PARAMETERS (below)
        ardrone_application_default_config.bitrate_ctrl_mode = VBC_MODE_DISABLED;
        if (IS_ARDRONE2)
        {
            ardrone_application_default_config.max_bitrate = 4000;
        }

        ardrone_application_default_config.navdata_options = NAVDATA_OPTION_FULL_MASK;        
        ardrone_application_default_config.video_channel = ZAP_CHANNEL_HORI;
        ardrone_application_default_config.control_level = (0 << CONTROL_LEVEL_COMBINED_YAW);
        ardrone_application_default_config.flying_mode = FLYING_MODE_FREE_FLIGHT;


        // LOAD THE CUSTOM CONFIGURATION FROM ROS PARAMETERS
        // all possible configuration parameters are stored in config_keys.h (and documented in the manual)
        // taking inspiration from ardrone_tool_configuration.c, we define some macros that replace these parameter definitions
        // with a function which attempts to read corresponding ros parameters, and then if successful, sets the parameter value for the drone
        // Note that we don't actually send these parameters to the drone, otherwise they will be overwritten when the profiles are created
        // in a later stage of the ARDrone initialization.

        #undef ARDRONE_CONFIG_KEY_IMM_a10
        #undef ARDRONE_CONFIG_KEY_REF_a10
        #undef ARDRONE_CONFIG_KEY_STR_a10
        #undef LOAD_PARAM_STR
        #undef LOAD_PARAM_NUM

        #define LOAD_PARAM_NUM(NAME,C_TYPE,DEFAULT)                                                                             \
            {                                                                                                                   \
              double param;                                                                                                     \
              ROS_DEBUG("CHECK: "#NAME" (Default = "#DEFAULT" = %f)",(float)DEFAULT);                                           \
              if(ros::param::get("~"#NAME,param))                                                                               \
              {                                                                                                                 \
                ardrone_application_default_config.NAME = (C_TYPE)param;                                                        \
                ROS_DEBUG("SET: "#NAME" = %f (DEFAULT = %f)", (float)ardrone_application_default_config.NAME, (float)DEFAULT);  \
              }                                                                                                                 \
            }

        #define LOAD_PARAM_STR(NAME,DEFAULT)                                                                                    \
            {                                                                                                                   \
              std::string param;                                                                                                \
              ROS_DEBUG("CHECK: "#NAME" (Default = "#DEFAULT" = %s)",DEFAULT);                                                  \
              if(ros::param::get("~"#NAME,param))                                                                               \
              {                                                                                                                 \
                param = param.substr(0,STRING_T_SIZE-1);                                                                        \
                strcpy(ardrone_application_default_config.NAME , param.c_str());                                                \
                ROS_DEBUG("SET: "#NAME" = %s (DEFAULT = %s)", ardrone_application_default_config.NAME, DEFAULT);                \      
              }                                                                                                                 \
            }

        #define ARDRONE_CONFIG_KEY_REF_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) //do nothing for reference-only parameters
        #define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) LOAD_PARAM_NUM(NAME,C_TYPE, DEFAULT) } // parameters under the custom key are for control of application/user/session, we don't want to change these!
        #define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) LOAD_PARAM_STR(NAME, DEFAULT) }

        #include <config_keys.h> // include the parameter definitions, which will be replaced by the above

        #undef LOAD_PARAM_NUM
        #undef LOAD_PARAM_STR
        #undef ARDRONE_CONFIG_KEY_IMM_a10
        #undef ARDRONE_CONFIG_KEY_REF_a10
        #undef ARDRONE_CONFIG_KEY_STR_a10

        // Now we delete any old configuration that we may have stored, this is so the profiles will be reinitialized to drone default before being updated with the potentially new set of custom parameters that we specify above.
        // We have to do this because only non-default parameters are sent, so if we delete a ros_param, the local parameter will be not be changed (above), thus will remain default and thus won't be updated on the drone - a problem if old profiles exist.

        char buffer[MULTICONFIG_ID_SIZE+1];
        
        sprintf(buffer,"-%s",usr_id);
        printf("Deleting Profile %s\n",buffer);
        ARDRONE_TOOL_CONFIGURATION_ADDEVENT (profile_id, buffer, NULL);

        sprintf(buffer,"-%s",app_id);
        printf("Deleting Application %s\n",buffer);
        ARDRONE_TOOL_CONFIGURATION_ADDEVENT (application_id, buffer, NULL);

        // Now continue with the rest of the initialization

        ardrone_tool_input_add(&teleop);
        uint8_t post_stages_index = 0;

        //Alloc structs
        specific_parameters_t * params             = (specific_parameters_t *)vp_os_calloc(1,sizeof(specific_parameters_t));
        specific_stages_t * driver_pre_stages  = (specific_stages_t*)vp_os_calloc(1, sizeof(specific_stages_t));
        specific_stages_t * driver_post_stages = (specific_stages_t*)vp_os_calloc(1, sizeof(specific_stages_t));
        vp_api_picture_t  * in_picture             = (vp_api_picture_t*) vp_os_calloc(1, sizeof(vp_api_picture_t));
        vp_api_picture_t  * out_picture            = (vp_api_picture_t*) vp_os_calloc(1, sizeof(vp_api_picture_t));

        in_picture->width          = _w;
        in_picture->height         = _h;

        out_picture->framerate     = 20;
        out_picture->format        = PIX_FMT_RGB24;
        out_picture->width         = _w;
        out_picture->height        = _h;

        out_picture->y_buf         = (uint8_t*) vp_os_malloc( _w * _h * 3 );
        out_picture->cr_buf        = NULL;
        out_picture->cb_buf        = NULL;

        out_picture->y_line_size   = _w * 3;
        out_picture->cb_line_size  = 0;
        out_picture->cr_line_size  = 0;
        
        //Alloc the lists
        driver_pre_stages->stages_list  = NULL;
        driver_post_stages->stages_list = (vp_api_io_stage_t*)vp_os_calloc(NB_DRIVER_POST_STAGES,sizeof(vp_api_io_stage_t));
        
        //Fill the POST-stages------------------------------------------------------
//        vp_os_memset (&vlat, 0x0, sizeof (vlat));
//        vlat.state = (vp_stages_latency_estimation_state) 0;
//        //vlat.last_decoded_frame_info= (void *)&vec;
//        driver_post_stages->stages_list[post_stages_index].name    = "LatencyEst";
//        driver_post_stages->stages_list[post_stages_index].type    = VP_API_FILTER_DECODER;
//        driver_post_stages->stages_list[post_stages_index].cfg     = (void *)&vlat;
//        driver_post_stages->stages_list[post_stages_index++].funcs = vp_stages_latency_estimation_funcs;
    
        driver_post_stages->stages_list[post_stages_index].name    = "ExtractData";
        driver_post_stages->stages_list[post_stages_index].type    = VP_API_OUTPUT_SDL;
        driver_post_stages->stages_list[post_stages_index].cfg     = NULL;
        driver_post_stages->stages_list[post_stages_index++].funcs   = vp_stages_export_funcs;
        
        driver_pre_stages->length  = 0;
        driver_post_stages->length = post_stages_index;
        
        params->in_pic = in_picture;
        params->out_pic = out_picture;
        params->pre_processing_stages_list  = driver_pre_stages;
        params->post_processing_stages_list = driver_post_stages;
        params->needSetPriority = 1;
        params->priority = 31;
        // Using the provided threaded pipeline implementation from SDK
        START_THREAD(video_stage, params);
        video_stage_init();
        if (ARDRONE_VERSION() >= 2)
        {
            START_THREAD (video_recorder, NULL);
            video_recorder_init ();
            video_recorder_resume_thread();
        }
        // Threads do not start automatically
        video_stage_resume_thread();
        ardrone_tool_set_refresh_time(25);
        //rosDriver->configure_drone();
		START_THREAD(update_ros, rosDriver);
		return C_OK;
	}
    
    C_RESULT ardrone_tool_shutdown_custom() 
    {
        PRINT("Shutting down ... \n ");
        JOIN_THREAD(update_ros);
        delete rosDriver;
        video_stage_resume_thread();
        ardrone_tool_input_remove(&teleop);
        return C_OK;
    }
    
	C_RESULT navdata_custom_init(void *) {
		return C_OK;
	}

    C_RESULT navdata_custom_process(const navdata_unpacked_t * const pnd) {
        vp_os_mutex_lock(&navdata_lock);
        shared_navdata_receive_time = ros::Time::now();
        shared_raw_navdata = (navdata_unpacked_t*)pnd;

        if(realtime_navdata)
        {
            rosDriver->PublishNavdataTypes(*shared_raw_navdata, shared_navdata_receive_time); //if we're publishing navdata at full speed, publish!
            rosDriver->publish_navdata(*shared_raw_navdata, shared_navdata_receive_time);
        }

        current_navdata_id++;
        vp_os_mutex_unlock(&navdata_lock);
		return C_OK;
	}

	C_RESULT navdata_custom_release() {
		return C_OK;
	}
    
    bool_t ardrone_tool_exit() {
        return (should_exit == 1);
    }
    
	BEGIN_THREAD_TABLE
    THREAD_TABLE_ENTRY(video_stage, 31)
    THREAD_TABLE_ENTRY(update_ros, 43)
    THREAD_TABLE_ENTRY(video_recorder, 20)
    THREAD_TABLE_ENTRY(navdata_update, 31)
//	THREAD_TABLE_ENTRY(ATcodec_Commands_Client, 43)
	THREAD_TABLE_ENTRY(ardrone_control, 31)
	END_THREAD_TABLE

	BEGIN_NAVDATA_HANDLER_TABLE
	NAVDATA_HANDLER_TABLE_ENTRY(
			navdata_custom_init,
			navdata_custom_process,
			navdata_custom_release,
			NULL)
	END_NAVDATA_HANDLER_TABLE
}

