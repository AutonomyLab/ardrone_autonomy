#ifndef _VIDEO_STAGE_ENCODED_RECORDER_H_
#define _VIDEO_STAGE_ENCODED_RECORDER_H_

#include <stdio.h>
#include <VP_Api/vp_api.h>
#include <utils/ardrone_video_encapsuler.h>

#define VIDEO_ENCODED_FILENAME_LENGTH 1024

#ifndef _VIDEO_ENCODED_RECORD_STATE_ENUM_
#define _VIDEO_ENCODED_RECORD_STATE_ENUM_
typedef enum
{
  VIDEO_ENCODED_STOPPED = 0,
  VIDEO_ENCODED_RECORDING,
  VIDEO_ENCODED_START_RECORD,
  VIDEO_ENCODED_WAITING_STREAM_END,
} video_encoded_record_state;
#endif

typedef void (*video_stage_encoded_recorder_callback)(const char *mediaPath, bool_t addToQueue);

typedef struct _video_stage_encoded_recorder_config_t
{
  char video_filename[VIDEO_ENCODED_FILENAME_LENGTH];
  ardrone_video_t *video;
  video_encoded_record_state startRec;
  uint32_t fps;
  vp_os_mutex_t videoMutex; 
  int stage;
  uint16_t lastStreamId;
  uint16_t currentStreamId;
  video_stage_encoded_recorder_callback finish_callback;
#ifdef USE_ELINUX
  /* Asynchronous recording thread */
  bool_t use_asynchronous_mode;
  THREAD_HANDLE thread_handle;
  int thread_priority;
  void * thread;
  int32_t quota;   /*! Quantity of video data we can store */
  int com_pipe[2];
#endif
} video_stage_encoded_recorder_config_t;

/*
 * Message passed in the communication pipe in asynchronous mode.
 */
#ifdef USE_ELINUX
	typedef struct{
		void * slice;
		int sliceSize;
	}video_stage_encoded_recorder_msg_t;
#endif


C_RESULT video_stage_encoded_recorder_handle (video_stage_encoded_recorder_config_t *cfg, PIPELINE_MSG msg_id, void *callback, void *param);
C_RESULT video_stage_encoded_recorder_open(video_stage_encoded_recorder_config_t *cfg);
C_RESULT video_stage_encoded_recorder_transform(video_stage_encoded_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_stage_encoded_recorder_close(video_stage_encoded_recorder_config_t *cfg);

video_encoded_record_state
video_stage_encoded_complete_recorder_state (void);
void video_stage_encoded_recorder_enable (bool_t enable, uint32_t timestamp);
void video_stage_encoded_recorder_force_stop (void);
void video_stage_encoded_recorder_com_timeout (void);
bool_t video_stage_encoded_recorder_state (void);

C_RESULT video_stage_encoded_recorder_finish (video_stage_encoded_recorder_config_t *cfg);
int frameIsLastFrame (uint8_t *data);

extern const vp_api_stage_funcs_t video_encoded_recorder_funcs;

extern video_stage_encoded_recorder_config_t video_stage_encoded_recorder_config;


#ifdef USE_ELINUX
#define VIDEO_STAGE_ENCODED_RECORDER_STACK_SIZE (8192)
PROTO_THREAD_ROUTINE_STACK(video_stage_encoded_recorder,param,VIDEO_STAGE_ENCODED_RECORDER_STACK_SIZE);
#endif



#endif // _VIDEO_STAGE_ENCODED_RECORDER_H_
