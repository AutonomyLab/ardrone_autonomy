#include "video.h"

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
        printf("The size of buffer is %d\n", in->size);
//    vp_os_mutex_lock(&video_update_lock);
//    /* Get a reference to the last decoded picture */
//    pixbuf_data      = (uint8_t*)in->buffers[0];
//    /* Copy the entire buffer, TODO: can we movet this to the thread to make the transform faster?*/
	memcpy(buffer, in->buffers[0], in->size);
//    //memcpy(buffer, in->buffers[0], in->size);
//    vp_os_mutex_unlock(&video_update_lock);
//    PRINT("In Transform after copy\n");
	current_frame_id++;
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