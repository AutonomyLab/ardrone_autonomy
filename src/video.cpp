#include <ardrone_autonomy/video.h>

#define NB_STAGES 10
#define CAMIF_H_CAMERA_USED CAMIF_CAMERA_OVTRULY

unsigned char buffer[MAX_STREAM_WIDTH * MAX_STREAM_HEIGHT * 3];
long int current_frame_id = 0;

extern "C" C_RESULT export_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	return (SUCCESS);
}

extern "C" C_RESULT export_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
//    PRINT("In Transform before copy\n");
//        printf("The size of buffer is %d\n", in->size);
    vp_os_mutex_lock(&video_lock);
	memcpy(buffer, in->buffers[0], in->size);
    current_frame_id++;
    if(realtime_video)
    {
    	rosDriver->publish_video();
    }
    vp_os_mutex_unlock(&video_lock);
//    vp_os_mutex_unlock(&video_update_lock);	
 	return (SUCCESS);
}

extern "C" C_RESULT export_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	return (SUCCESS);
}

const vp_api_stage_funcs_t vp_stages_export_funcs =
{
	NULL,
	(vp_api_stage_open_t)export_stage_open,
	(vp_api_stage_transform_t)export_stage_transform,
	(vp_api_stage_close_t)export_stage_close
};
