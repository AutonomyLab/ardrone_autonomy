#include "ardrone_sdk.h"
#include "video.h"
#include "teleop_twist.h"

navdata_demo_t navdata;
navdata_phys_measures_t navdata_phys;
navdata_vision_detect_t navdata_detect;

navdata_time_t arnavtime;

extern "C" {
 
	C_RESULT ardrone_tool_init_custom(void) {
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
        
        //TODO: Please FIX this to read default values from ros params
        ardrone_application_default_config.bitrate_ctrl_mode = VBC_MODE_DYNAMIC;
        ardrone_application_default_config.autonomous_flight = 0;
        ardrone_application_default_config.control_level = (0 << CONTROL_LEVEL_COMBINED_YAW);
        ardrone_application_default_config.outdoor = false;
        ardrone_application_default_config.flight_without_shell = true;
        ardrone_application_default_config.flying_mode = FLYING_MODE_FREE_FLIGHT;
        ardrone_application_default_config.video_on_usb = 0;
        ardrone_application_default_config.altitude_max = 3000;
        ardrone_application_default_config.altitude_min = 100;
        ardrone_application_default_config.control_vz_max = 850;
        ardrone_application_default_config.control_yaw = (100.0 /180.0) * 3.1415;
        ardrone_application_default_config.euler_angle_max = (12.0 / 180.0) * 3.1415;
        
        
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
        params->needSetPriority = 0;
        params->priority = 0;
        // Using the provided threaded pipeline implementation from SDK
        START_THREAD(video_stage, params);
        video_stage_init();
        // Threads do not start automatically
        video_stage_resume_thread();
		//START_THREAD(video_update_thread, 0);
        //START_THREAD(mani, 0);
		return C_OK;
	}

	C_RESULT navdata_custom_init(void *) {
		return C_OK;
	}

	C_RESULT navdata_custom_process(const navdata_unpacked_t * const pnd) {
		navdata_detect = pnd->navdata_vision_detect;
		navdata_phys = pnd->navdata_phys_measures;
		navdata = pnd->navdata_demo;
		arnavtime = pnd->navdata_time;
		return C_OK;
	}

	C_RESULT navdata_custom_release() {
		return C_OK;
	}

	BEGIN_THREAD_TABLE
    THREAD_TABLE_ENTRY(video_stage, 20)
	THREAD_TABLE_ENTRY(navdata_update, 20)
	THREAD_TABLE_ENTRY(ATcodec_Commands_Client, 20)
	THREAD_TABLE_ENTRY(ardrone_control, 20)
	END_THREAD_TABLE

	BEGIN_NAVDATA_HANDLER_TABLE
	NAVDATA_HANDLER_TABLE_ENTRY(
			navdata_custom_init,
			navdata_custom_process,
			navdata_custom_release,
			NULL)
	END_NAVDATA_HANDLER_TABLE
}

