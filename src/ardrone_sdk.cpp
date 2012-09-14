#include "ardrone_sdk.h"
#include "video.h"
#include "teleop_twist.h"
#include "ardrone_driver.h"

navdata_demo_t shared_navdata;
navdata_phys_measures_t shared_navdata_phys;
navdata_vision_detect_t shared_navdata_detect;
navdata_pressure_raw_t shared_navdata_pressure;
navdata_magneto_t shared_navdata_magneto;
navdata_wind_speed_t shared_navdata_wind;
navdata_time_t shared_arnavtime;

vp_os_mutex_t navdata_lock;
vp_os_mutex_t video_lock;
vp_os_mutex_t twist_lock;

long int current_navdata_id = 0;

ARDroneDriver* rosDriver;

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


        //TODO: Please FIX this to read default values from ros params and move them to ardrone driver
        //Roadmap: We have the pointer to ARDroneDriver here, so it is doable to return back ros params
        //using this class.
        ardrone_application_default_config.bitrate_ctrl_mode = (int) rosDriver->getRosParam("~bitrate_ctrl_mode", (double) VBC_MODE_DISABLED);
        if (IS_ARDRONE2)
        {
            ardrone_application_default_config.max_bitrate = (int) rosDriver->getRosParam("~max_bitrate", 4000.0);
        }
        // TODO: Fix CAT_COMMONS
        ardrone_application_default_config.outdoor = (bool) rosDriver->getRosParam("~outdoor", 0.0);
        ardrone_application_default_config.flight_without_shell = (bool) rosDriver->getRosParam("~flight_without_shell", 1.0);
        ardrone_application_default_config.altitude_max = (int) rosDriver->getRosParam("~altitude_max", 3000.0);
        ardrone_application_default_config.altitude_min = (int) rosDriver->getRosParam("~altitude_min", 100.0);
        ardrone_application_default_config.enemy_colors = (int) rosDriver->getRosParam("~enemy_colors", (double) ARDRONE_DETECTION_COLOR_ORANGE_YELLOW);
        ardrone_application_default_config.enemy_without_shell = (bool) rosDriver->getRosParam("~enemy_without_shell", (double) 0.0);
        ardrone_application_default_config.video_on_usb = 0;
        ardrone_application_default_config.autonomous_flight = 0;


        ardrone_application_default_config.control_vz_max = (float) rosDriver->getRosParam("~control_vz_max", 850.0);
        ardrone_application_default_config.control_yaw = (float) rosDriver->getRosParam("~control_yaw", (100.0 /180.0) * 3.1415);
        ardrone_application_default_config.euler_angle_max = (float) rosDriver->getRosParam("~euler_angle_max", (12.0 / 180.0) * 3.1415);
        ardrone_application_default_config.bitrate = (int) rosDriver->getRosParam("~bitrate", 4000.0);                
        ardrone_application_default_config.navdata_demo = (int) rosDriver->getRosParam("~navdata_demo", (double) 1);
        ardrone_application_default_config.detect_type = (int) rosDriver->getRosParam("~detect_type", (double) CAD_TYPE_MULTIPLE_DETECTION_MODE);
        ardrone_application_default_config.detections_select_h = rosDriver->getRosParam("~detections_select_h", 
                (double) TAG_TYPE_MASK(TAG_TYPE_SHELL_TAG_V2));
        ardrone_application_default_config.detections_select_v_hsync = rosDriver->getRosParam("~detections_select_v_hsync", 
                (double) TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL));
//        ardrone_application_default_config.detections_select_v = rosDriver->getRosParam("~detections_select_v", 
//                (double) TAG_TYPE_MASK(TAG_TYPE_BLACK_ROUNDEL));
             
        ardrone_application_default_config.navdata_options = NAVDATA_OPTION_FULL_MASK /*&
        ~(NAVDATA_OPTION_MASK(NAVDATA_TRACKERS_SEND_TAG)
        | NAVDATA_OPTION_MASK(NAVDATA_VISION_OF_TAG)
        | NAVDATA_OPTION_MASK(NAVDATA_VISION_PERF_TAG)
        | NAVDATA_OPTION_MASK(NAVDATA_VISION_TAG))*/;
        
        ardrone_application_default_config.video_channel = ZAP_CHANNEL_HORI;
        ardrone_application_default_config.control_level = (0 << CONTROL_LEVEL_COMBINED_YAW);
        ardrone_application_default_config.flying_mode = FLYING_MODE_FREE_FLIGHT;

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
        shared_navdata_detect = pnd->navdata_vision_detect;
        shared_navdata_phys = pnd->navdata_phys_measures;
        shared_navdata = pnd->navdata_demo;
        shared_arnavtime = pnd->navdata_time;
        if (IS_ARDRONE2)
        { // This is neccessary
            shared_navdata_pressure = pnd->navdata_pressure_raw;
            shared_navdata_magneto = pnd->navdata_magneto;
            shared_navdata_wind = pnd->navdata_wind_speed;
        }
        vp_os_mutex_unlock(&navdata_lock);
        current_navdata_id++;
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

