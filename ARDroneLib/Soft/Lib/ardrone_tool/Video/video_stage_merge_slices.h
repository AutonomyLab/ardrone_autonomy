#ifndef _VIDEO_STAGE_MERGE_SLICES_H_
#define _VIDEO_STAGE_MERGE_SLICES_H_

#include <VP_Api/vp_api.h>


typedef struct {
  uint32_t accumulated_size;
  uint32_t buffer_size;
  uint8_t  *buffer;
  uint8_t  **bufferPointer;
  int nb_slices;
 }video_stage_merge_slices_buffer_t;
 
typedef struct _video_stage_merge_slices_config_t
{
 int mergingBuffer;
 int readyBuffer;
 video_stage_merge_slices_buffer_t bufs[2];
}
video_stage_merge_slices_config_t;

C_RESULT video_stage_merge_slices_handle (video_stage_merge_slices_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param);
C_RESULT video_stage_merge_slices_open(video_stage_merge_slices_config_t *cfg);
C_RESULT video_stage_merge_slices_transform(video_stage_merge_slices_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_stage_merge_slices_close(video_stage_merge_slices_config_t *cfg);

extern const vp_api_stage_funcs_t video_stage_merge_slices_funcs;

#endif
