#ifdef FFMPEG_SUPPORT

#ifndef _VIDEO_STAGE_FFMPEG_RECORDER_H_
#define _VIDEO_STAGE_FFMPEG_RECORDER_H_

/* From FFMPEG example */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavfilter/avfilter.h>
#include <libavutil/avutil.h>
#include <libavcodec/avcodec.h>

#include <stdio.h>
#include <VP_Api/vp_api.h>

#include <ardrone_tool/Video/video_stage_recorder.h>


#ifndef _VIDEO_RECORD_STATE_ENUM_
#define _VIDEO_RECORD_STATE_ENUM_
#endif

typedef struct _video_stage_ffmpeg_recorder_config_t
{
	// Public
	char video_filename[1024];
	uint32_t *numframes;
	int stage;

	// Private
	video_record_state startRec;
	int flag_video_file_open;
} video_stage_ffmpeg_recorder_config_t;

C_RESULT video_stage_ffmpeg_recorder_handle (video_stage_ffmpeg_recorder_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param);
C_RESULT video_stage_ffmpeg_recorder_open(video_stage_ffmpeg_recorder_config_t *cfg);
C_RESULT video_stage_ffmpeg_recorder_transform(video_stage_ffmpeg_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_stage_ffmpeg_recorder_close(video_stage_ffmpeg_recorder_config_t *cfg);

void create_video_file(const char*filename,int width,int height,int frame_rate, enum PixelFormat pix_fmt);
void close_video_file();
AVStream *add_video_stream(AVFormatContext *oc, enum CodecID codec_id,int width, int height, int frame_rate, enum PixelFormat pix_fmt);
AVFrame *alloc_picture(enum PixelFormat pix_fmt, int width, int height);
void open_video(AVFormatContext *oc, AVStream *st);
void write_video_frame(AVFormatContext *oc, AVStream *st);
void close_video(AVFormatContext *oc, AVStream *st);

extern const vp_api_stage_funcs_t video_ffmpeg_recorder_funcs;

#endif // _VIDEO_STAGE_RECORDER_H_

#endif // FFMPEG_SUPPORT
