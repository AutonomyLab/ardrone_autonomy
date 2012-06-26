#ifndef _ACADEMY_STAGE_RECORDER_H_
#define _ACADEMY_STAGE_RECORDER_H_

#include <stdio.h>
#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>
#include <ardrone_tool/Academy/academy.h>

#ifndef _ACADEMY_RECORD_STATE_ENUM_
#define _ACADEMY_RECORD_STATE_ENUM_
typedef enum
{
	ACADEMY_RECORD_STOP,    // Video recording has been stopped. Stage will end and restart.
	ACADEMY_RECORD_START,   // Video recording has started.
} 
ardrone_academy_record_state;
#endif

typedef struct _ardrone_academy_stage_recorder_config_t_ ardrone_academy_stage_recorder_config_t;

typedef void (*ardrone_academy_recorder_callback)(ardrone_academy_stage_recorder_config_t *);

struct _ardrone_academy_stage_recorder_config_t_
{
	// public
	vp_api_picture_t *picture;
	DEST_HANDLE 	  dest;
    ardrone_academy_recorder_callback callback;
    
 	// private
    FILE* fp;
    ardrone_academy_record_state startRec;
	char video_filename[ACADEMY_MAX_FILENAME];
};

bool_t ardrone_academy_stage_recorder_state(void);
void ardrone_academy_stage_recorder_enable(bool_t enable, uint32_t timestamps);

C_RESULT ardrone_academy_stage_recorder_open(ardrone_academy_stage_recorder_config_t *cfg);
C_RESULT ardrone_academy_stage_recorder_transform(ardrone_academy_stage_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT ardrone_academy_stage_recorder_close(ardrone_academy_stage_recorder_config_t *cfg);

extern const vp_api_stage_funcs_t ardrone_academy_stage_recorder_funcs;
extern ardrone_academy_stage_recorder_config_t ardrone_academy_stage_recorder_config;

#endif // _ACADEMY_STAGE_RECORDER_H_
