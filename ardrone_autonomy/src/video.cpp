/**
Software License Agreement (BSD)

\file      video.cpp
\authors   Mani Monajjemi <mmonajje@sfu.ca>
\copyright Copyright (c) 2012, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ardrone_autonomy/video.h>

#define NB_STAGES 10
#define CAMIF_H_CAMERA_USED CAMIF_CAMERA_OVTRULY

unsigned char buffer[MAX_STREAM_WIDTH * MAX_STREAM_HEIGHT * 3];
int32_t current_frame_id = 0;
ros::Time shared_video_receive_time;

extern "C" C_RESULT export_stage_open(void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  return (SUCCESS);
}

extern "C" C_RESULT export_stage_transform(void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
//    PRINT("In Transform before copy\n");
//        printf("The size of buffer is %d\n", in->size);
  vp_os_mutex_lock(&video_lock);
  shared_video_receive_time = ros::Time::now();
  memcpy(buffer, in->buffers[0], in->size);
  current_frame_id++;
  if (realtime_video)
  {
    ros_driver->PublishVideo();
  }
  vp_os_mutex_unlock(&video_lock);
//    vp_os_mutex_unlock(&video_update_lock);
  return (SUCCESS);
}

extern "C" C_RESULT export_stage_close(void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
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
