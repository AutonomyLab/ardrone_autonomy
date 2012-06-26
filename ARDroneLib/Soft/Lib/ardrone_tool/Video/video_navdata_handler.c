#include <ardrone_tool/Video/video_navdata_handler.h>
#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/ardrone_tool_configuration.h>


#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#endif

#define TIMEOUT_SEC (1)
#define TIMEOUT_MS (1000*TIMEOUT_SEC)

uint32_t hdvideo_remaining_frames;
uint32_t hdvideo_remaining_kilobytes;
uint32_t hdvideo_maximum_kilobytes;
float hdvideo_fifo_fill_percentage; // range 0.0 - 100.0
float hdvideo_retrieving_progress;

static uint32_t hdvideo_frames_to_retreive;

C_RESULT video_navdata_handler_init( void* data )
{
  hdvideo_remaining_frames = 0;
  hdvideo_remaining_kilobytes = 0;
  hdvideo_maximum_kilobytes = 0;
  hdvideo_fifo_fill_percentage = 0.0;
  hdvideo_retrieving_progress = -1.0;
  hdvideo_frames_to_retreive = 0;
  return C_OK;
}


#ifdef _WIN32
#define isTimeout(NOW,PREV)	(((NOW-PREV) >= TIMEOUT_MS) ? 1 : 0)
#define getTime(TIME) (TIME) = GetTickCount()
#else
#define getTime(TIME) gettimeofday (&TIME, NULL)
static inline int isTimeout (struct timeval now, struct timeval prev)
{
	struct timeval diff;
	if (now.tv_usec >= prev.tv_usec)
	{
		diff.tv_usec = now.tv_usec - prev.tv_usec;
		diff.tv_sec  = now.tv_sec  - prev.tv_sec;
	}
	else
	{
		diff.tv_usec = 1000000 - prev.tv_usec + now.tv_usec;
		diff.tv_sec  = now.tv_sec  - prev.tv_sec - 1;
	}
	return ((diff.tv_sec >= TIMEOUT_SEC) ? 1 : 0);
}
#endif


C_RESULT video_navdata_handler_process( const navdata_unpacked_t* const navdata )
{
 /* Compute time between two prevous navdata to detect a timeout
  * (i.e. more than 1 sec between two navdatas)
  */
  static int firstTime = 1;
#if _WIN32
  static DWORD prev, now;
#else
  static struct timeval prev, now;
#endif
  getTime (now);
  if (1 == firstTime)
  {
    firstTime = 0;
  }
  else
  {
  	int flag = isTimeout (now, prev);
    if (flag)
    {
  	  video_com_stage_notify_timeout ();
    }
  }

  /* Video storage navdatas */
  hdvideo_remaining_frames = navdata->navdata_hdvideo_stream.storage_fifo_nb_packets;
  hdvideo_remaining_kilobytes = navdata->navdata_hdvideo_stream.storage_fifo_size;
  hdvideo_maximum_kilobytes = ardrone_control_config.video_storage_space;
  if (NAVDATA_HDVIDEO_STORAGE_FIFO_IS_FULL & navdata->navdata_hdvideo_stream.hdvideo_state)
    {
      // Socket is flagged as full 
      hdvideo_fifo_fill_percentage = 100.0;
    }
  else if ((0 == hdvideo_maximum_kilobytes) ||
           (hdvideo_maximum_kilobytes < hdvideo_remaining_kilobytes))
    {
      // Unexpected result (we don't have the total size
      // or the current size is greater than the total size)
      hdvideo_fifo_fill_percentage = -1.0;
    }
  else
    {
      // Normal case
      hdvideo_fifo_fill_percentage = (hdvideo_remaining_kilobytes * 100.0) / (hdvideo_maximum_kilobytes * 1.0);
    }

  // Process check
  if (0 != hdvideo_frames_to_retreive &&
      hdvideo_frames_to_retreive >= hdvideo_remaining_frames)
    {
      uint32_t _retreived_frames = hdvideo_frames_to_retreive - hdvideo_remaining_frames;
      hdvideo_retrieving_progress = (float)(_retreived_frames) / (float)(hdvideo_frames_to_retreive);
    }
  else
    {
      hdvideo_retrieving_progress = -1.0;
    }

  getTime (prev);
  return C_OK;
}

C_RESULT video_navdata_handler_release( void )
{
  return C_OK;
}

void startRetreiving (void)
{
  hdvideo_frames_to_retreive = hdvideo_remaining_frames;
}

void endRetreiving (void)
{
  hdvideo_frames_to_retreive = 0;
}
