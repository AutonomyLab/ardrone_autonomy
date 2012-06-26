#include <time.h>
#ifndef _WIN32
	#include <sys/time.h>
#else
 
 #include <sys/timeb.h>
 #include <Winsock2.h>  // for timeval structure

 int gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#endif

#include <VP_Os/vp_os_malloc.h>
#include <VP_Api/vp_api_picture.h>

#include <config.h>
#include <ardrone_tool/Video/video_stage_recorder.h>

//#define USE_FIXED_60FPS

#ifdef USE_VIDEO_YUV
#define VIDEO_FILE_EXTENSION "yuv"
#else
#define VIDEO_FILE_EXTENSION "y"
#endif

#ifndef VIDEO_FILE_DEFAULT_PATH
#ifdef USE_ELINUX
#define VIDEO_FILE_DEFAULT_PATH "/data/video/usb/"
#else
#define VIDEO_FILE_DEFAULT_PATH root_dir
extern char root_dir[];
#endif
#endif

#if defined (NAVDATA_VISION_INCLUDED) && defined (USE_ELINUX)
static int32_t picture_captured = 0;
extern void navdata_set_raw_picture(int32_t new_raw_picture);
#endif

const vp_api_stage_funcs_t video_recorder_funcs = {
  (vp_api_stage_handle_msg_t) video_stage_recorder_handle,
  (vp_api_stage_open_t) video_stage_recorder_open,
  (vp_api_stage_transform_t) video_stage_recorder_transform,
  (vp_api_stage_close_t) video_stage_recorder_close
};

C_RESULT
video_stage_recorder_handle (video_stage_recorder_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param)
{
   void (*recorder_callback)(video_stage_recorder_config_t*) = callback;

	switch (msg_id)
	{
		case PIPELINE_MSG_START: //video
			{
				if(cfg->startRec==VIDEO_RECORD_STOP)
				{
					cfg->startRec=VIDEO_RECORD_HOLD;
				}
				else
				{
					cfg->startRec=VIDEO_RECORD_STOP;
				}
			}
			break;
  
		case PIPELINE_MSG_COMMAND: //picture
			{
				if(cfg->startRec==VIDEO_RECORD_STOP)
				{
					cfg->startRec=VIDEO_PICTURE_HOLD;
				}
				else
				{
					cfg->startRec=VIDEO_RECORD_STOP;
				}
			}	
		default:
			{
				break;
			}
	}
	if (recorder_callback!=NULL) { recorder_callback(cfg); }

	return (VP_SUCCESS);
}

C_RESULT video_stage_recorder_open(video_stage_recorder_config_t *cfg)
{
	cfg->startRec=VIDEO_RECORD_STOP;
  return C_OK;
}

C_RESULT video_stage_recorder_transform(video_stage_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
#ifndef _WIN32

	time_t temptime;
	struct timeval tv;
#ifdef USE_FIXED_60FPS
	static struct timeval old_tv;
	static uint8_t* old_pic=NULL;
	unsigned long delta_us=0;
	int ratio=0;
	int i=0;
#endif
	vp_os_mutex_lock( &out->lock );

	if( out->status == VP_API_STATUS_INIT )
	{
		out->numBuffers   = 1;
		out->indexBuffer  = 0;
		out->lineSize     = NULL;
		//out->buffers      = (int8_t **) vp_os_malloc( sizeof(int8_t *) );
	}

	out->size     = in->size;
	out->status   = in->status;
	out->buffers  = in->buffers;

	if( in->status == VP_API_STATUS_ENDED ) {
		out->status = in->status;
	}
	else if(in->status == VP_API_STATUS_STILL_RUNNING) {
		out->status = VP_API_STATUS_PROCESSING;
	}
	else {
		out->status = in->status;
	}

	gettimeofday(&tv,NULL);
	vp_api_picture_t* picture = (vp_api_picture_t *) in->buffers;

	if(cfg->startRec==VIDEO_RECORD_HOLD)
	{
		struct tm *atm;

		temptime = (time_t)tv.tv_sec;
		atm = localtime(&temptime);  //atm = localtime(&tv.tv_sec);
		printf("recording video\n");
		if(strlen(cfg->video_filename) == 0)
			sprintf(cfg->video_filename, "%s/video_%04d%02d%02d_%02d%02d%02d_w%i_h%i.%s",
				VIDEO_FILE_DEFAULT_PATH,
				atm->tm_year+1900, atm->tm_mon+1, atm->tm_mday,
				atm->tm_hour, atm->tm_min, atm->tm_sec,
				picture->width,picture->height,
				VIDEO_FILE_EXTENSION);

		cfg->fp = fopen(cfg->video_filename, "wb");
		if (cfg->fp == NULL)
			printf ("error open file %s\n", cfg->video_filename);
		cfg->startRec=VIDEO_RECORD_START;
	}

	if(cfg->startRec==VIDEO_PICTURE_HOLD)
	{
		struct tm *atm;


		temptime = (time_t)tv.tv_sec;
		atm = localtime(&temptime);  //atm = localtime(&tv.tv_sec);

		printf("recording picture\n"); 
		sprintf(cfg->video_filename, "%s/picture_%04d%02d%02d_%02d%02d%02d_w%i_h%i.%s",
				VIDEO_FILE_DEFAULT_PATH,
				atm->tm_year+1900, atm->tm_mon+1, atm->tm_mday,
				atm->tm_hour, atm->tm_min, atm->tm_sec,
				picture->width,picture->height,
				VIDEO_FILE_EXTENSION);

		cfg->fp = fopen(cfg->video_filename, "wb");
		if (cfg->fp == NULL)
			printf ("error open file %s\n", cfg->video_filename);
		cfg->startRec=VIDEO_PICTURE_START;
	}
	if( cfg->fp != NULL && out->size > 0 && out->status == VP_API_STATUS_PROCESSING && (cfg->startRec==VIDEO_RECORD_START||cfg->startRec==VIDEO_PICTURE_START ))
	{


#if defined (NAVDATA_VISION_INCLUDED) && defined (USE_ELINUX)
		navdata_set_raw_picture(picture_captured++);
#endif

#ifdef USE_FIXED_60FPS
		delta_us=(tv.tv_sec*1000000+tv.tv_usec)-(old_tv.tv_sec*1000000+old_tv.tv_usec);
		ratio=delta_us/16666;
		old_tv=tv;

		for(i=0; i<ratio && old_pic; i++)
		{
			fwrite(old_pic, picture->width * picture->height, 1, cfg->fp);
#ifdef USE_VIDEO_YUV
			fwrite(old_pic+(picture->width * picture->height), picture->width * picture->height >> 1, 1, cfg->fp);
#endif
		}

		if(old_pic==NULL)
		{
			old_pic=vp_os_malloc(picture->width * picture->height*3/2);
		}

		memcpy(old_pic, picture->y_buf, picture->width * picture->height);
		memcpy(old_pic + picture->width * picture->height, picture->cb_buf, picture->width * picture->height >> 2);
		memcpy(old_pic + picture->width * picture->height*5/4, picture->cr_buf, picture->width * picture->height >> 2);
#endif //USE_FIXED_60FPS

		fwrite(picture->y_buf, picture->width * picture->height, 1, cfg->fp);
#ifdef USE_VIDEO_YUV
		fwrite(picture->cb_buf, picture->width * picture->height >> 2, 1, cfg->fp);
		fwrite(picture->cr_buf, picture->width * picture->height >> 2, 1, cfg->fp);
#endif
		if(cfg->startRec==VIDEO_PICTURE_START )//if picture, we stop after one picture (	and if video we continue )
		{
			cfg->startRec=VIDEO_RECORD_STOP;
		}
	}
	else
	{
		if(cfg->startRec==VIDEO_RECORD_STOP && cfg->fp !=NULL)
		{
#ifdef USE_FIXED_60FPS
			if(old_pic)
			{
				vp_os_free(old_pic);
				old_pic=NULL;
			}
#endif
			fclose(cfg->fp);
			cfg->fp=NULL;
		}
	}

	vp_os_mutex_unlock( &out->lock );
#endif
	return C_OK;
}

C_RESULT video_stage_recorder_close(video_stage_recorder_config_t *cfg)
{
  if( cfg->fp != NULL )
    fclose( cfg->fp );

  return C_OK;
}
