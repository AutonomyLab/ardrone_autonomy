#include <time.h>
#ifndef _WIN32
#include <sys/time.h>
#else
#include <sys/timeb.h>
#include <Winsock2.h>  // for timeval structure
#endif

#include <VP_Api/vp_api.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Api/vp_api_picture.h>
#include <ardrone_tool/Academy/academy.h>

#include <config.h>
#include <ardrone_tool/Video/video_stage_encoded_recorder.h>
#include <utils/ardrone_date.h>

#include <ardrone_tool/Video/video_navdata_handler.h>

#ifdef USE_ELINUX
#define VIDEO_ENCODED_FILE_DEFAULT_PATH "/data/video/usb/"
#endif

#define VIDEO_ENCODED_RECORDER_VERBOSE (1)

#if defined(DEBUG) || VIDEO_ENCODED_RECORDER_VERBOSE
#define __VER_DEBUG (1)
#else
#define __VER_DEBUG (0)
#endif

#if __VER_DEBUG
#define VER_PRINT(...)                                                  \
    do                                                                  \
    {                                                                   \
        printf ("Video encoded recorder [%s @ %d] : ", __FUNCTION__, __LINE__); \
        printf (__VA_ARGS__);                                           \
        printf ("\n");                                                  \
    } while (0)
#else
#define VER_PRINT(...)
#endif

/* Uncomment to force all apps to generate .mov format instead of .mp4 for non-apple devices */
//#define FORCE_MOV_FORMAT

#if defined(TARGET_OS_IPHONE) || defined (TARGET_OS_IPHONE_SIMULATOR) || defined (FORCE_MOV_FORMAT)
#define MOVIE_FILE_EXTENSION "mov"
#define VIDEO_FORMAT (ARDRONE_VIDEO_MOV)
#else
#define MOVIE_FILE_EXTENSION "mp4"
#define VIDEO_FORMAT (ARDRONE_VIDEO_MP4)
#endif

#ifndef VIDEO_ENCODED_FILE_DEFAULT_PATH
#define VIDEO_ENCODED_FILE_DEFAULT_PATH flight_dir
extern char flight_dir[];
#endif

const vp_api_stage_funcs_t video_encoded_recorder_funcs = {
    (vp_api_stage_handle_msg_t) video_stage_encoded_recorder_handle,
    (vp_api_stage_open_t) video_stage_encoded_recorder_open,
    (vp_api_stage_transform_t) video_stage_encoded_recorder_transform,
    (vp_api_stage_close_t) video_stage_encoded_recorder_close
};

video_stage_encoded_recorder_config_t video_stage_encoded_recorder_config;


#ifdef USE_ELINUX
DEFINE_THREAD_ROUTINE_STACK(video_stage_encoded_recorder,param,VIDEO_STAGE_ENCODED_RECORDER_STACK_SIZE)
{
    int res;
    video_stage_encoded_recorder_config_t *cfg = (video_stage_encoded_recorder_config_t *)param;
    video_stage_encoded_recorder_msg_t msg;
    char errBuf [50] = {0};
    ardrone_video_error_t vError = ARDRONE_VIDEO_NO_ERROR;
    parrot_video_encapsulation_t *PaVE;
    //int counter = 0;

    /* TODO : check if mutual exclusion with the main thread is needed */

    while(1)
    {
        /* Read the address of a new slice to write in the video file */
        res = read(cfg->com_pipe[0],&msg,sizeof(msg));

        if (res!=sizeof(msg)) { sleep(1); continue; }

        //printf("----> Reading %p %d bytes\n",msg.slice,msg.sliceSize);
        PaVE = (parrot_video_encapsulation_t *) msg.slice;

        if (msg.slice!=NULL)
        {
            /* Create a new MP4 file if necessary */
            if (NULL==cfg->video)
            {
                cfg->video = ardrone_video_start (cfg->video_filename, cfg->fps, VIDEO_FORMAT, &vError);
                if (ARDRONE_VIDEO_SUCCEEDED (vError) && NULL != cfg->video)
                {
                    cfg->currentStreamId = PaVE->stream_id;
                }
                else
                {
                    VER_PRINT ("An error occured when creating video");
                    cfg->startRec=VIDEO_ENCODED_STOPPED;
                }
            }

            if (NULL!=cfg->video)
            {
                ardrone_video_error_t vError = ardrone_video_addSlice (cfg->video, msg.slice);

                if (ARDRONE_VIDEO_FAILED (vError))
                {
                    ardrone_video_error_string (vError, errBuf, 50);
                    VER_PRINT ("Error while recording frame : %s", errBuf);
                    ardrone_video_cleanup (&(cfg->video));
                    cfg->startRec = VIDEO_ENCODED_STOPPED;
                }
                if (1 == frameIsLastFrame(msg.slice))
                {
                    VER_PRINT ("Received last frame for current stream, finishing video\n");
                    video_stage_encoded_recorder_finish (cfg);
                }

                //counter++; if (counter%16) { sync(); }
            }

            /* Free the data bloc which was just written */
            vp_os_free(msg.slice);
        }
        else  /* the data pointer is NULL to signal the end of the recording */
        {
            video_stage_encoded_recorder_finish (cfg);
        }

    }/*while 1*/
}
#endif


C_RESULT video_stage_encoded_recorder_finish (video_stage_encoded_recorder_config_t *cfg)
{
    C_RESULT retVal = C_OK;

#ifndef USE_ELINUX
    endRetreiving ();
#endif

    cfg->lastStreamId = cfg->currentStreamId;
    ardrone_video_error_t vError = ardrone_video_finish (&(cfg->video));
    if (ARDRONE_VIDEO_SUCCEEDED (vError))
    {
        VER_PRINT ("Video %s successfully written", cfg->video_filename);
        if(cfg->finish_callback != NULL)
            cfg->finish_callback((const char *)cfg->video_filename);
    }
    else
    {
        VER_PRINT ("Error while completing video");
        retVal = C_FAIL;
    }
    // Erase filename
    vp_os_memset (cfg->video_filename, 0x0, VIDEO_ENCODED_FILENAME_LENGTH);
    cfg->startRec = VIDEO_ENCODED_STOPPED;

#ifdef USE_ELINUX
  ardrone_video_system_cleanup();
#endif

    return retVal;
}

C_RESULT video_stage_encoded_recorder_handle (video_stage_encoded_recorder_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param)
{
#ifdef USE_ELINUX
    /* Handling the asynchronous mode */
    video_stage_encoded_recorder_msg_t asyn_msg;
#endif


#ifndef USE_ELINUX
    vp_os_mutex_lock (&cfg->videoMutex);
#endif

    void (*recorder_callback)(video_stage_encoded_recorder_config_t*) = callback;

    switch (msg_id)
    {
    case PIPELINE_MSG_START: // video start
        if(cfg->startRec==VIDEO_ENCODED_STOPPED)
        {
            time_t t;
            char date[ARDRONE_DATE_MAXSIZE];
            ardrone_time2date(*((uint32_t*)param), ARDRONE_FILE_DATE_FORMAT, date);
            char media_dirname[VIDEO_ENCODED_FILENAME_LENGTH];
            struct stat statbuf;

            snprintf(media_dirname, VIDEO_ENCODED_FILENAME_LENGTH, "%s/media_%s", VIDEO_ENCODED_FILE_DEFAULT_PATH, date);
            if((stat(media_dirname, &statbuf) != 0) && (mkdir(media_dirname, 0777) >= 0))
                VER_PRINT("Create local media directory %s if not exist\n", media_dirname);

            t = time(NULL);
            ardrone_time2date(t, ARDRONE_FILE_DATE_FORMAT, date);

          snprintf(cfg->video_filename, 
                     VIDEO_ENCODED_FILENAME_LENGTH,
                     "%s/video_%s.%s",
                     media_dirname,
                     date,
                     MOVIE_FILE_EXTENSION);

            cfg->startRec = VIDEO_ENCODED_START_RECORD;
        }
        break;

    case PIPELINE_MSG_STOP: // video stop
        if (cfg->startRec==VIDEO_ENCODED_START_RECORD)
        {
            cfg->startRec=VIDEO_ENCODED_STOPPED;
        }
        else if (cfg->startRec==VIDEO_ENCODED_RECORDING)
        {
            cfg->startRec = VIDEO_ENCODED_WAITING_STREAM_END;
#ifndef USE_ELINUX
            startRetreiving ();
#endif
        }
        break;

    case PIPELINE_MSG_SUSPEND: // Force stop
        if (cfg->startRec==VIDEO_ENCODED_START_RECORD)
        {
            cfg->startRec=VIDEO_ENCODED_STOPPED;
        }
        else if (cfg->startRec==VIDEO_ENCODED_RECORDING ||
                 cfg->startRec==VIDEO_ENCODED_WAITING_STREAM_END)
        {
#ifdef USE_ELINUX
            if (cfg->use_asynchronous_mode)
            {
                /* Send an empty slice to the asynchronous thread */
                asyn_msg.sliceSize = 0;
                asyn_msg.slice     = NULL;
                write(cfg->com_pipe[1],&asyn_msg,sizeof(asyn_msg));
            }
            else
            {
#endif
                video_stage_encoded_recorder_finish (cfg);

#ifdef USE_ELINUX
            }
#endif

        }
        break;

    default:
        break;
    }
    if (recorder_callback!=NULL) {
        recorder_callback(cfg);
    }

#ifndef USE_ELINUX
    vp_os_mutex_unlock (&cfg->videoMutex);
#endif

    return (VP_SUCCESS);
}

bool_t video_stage_encoded_recorder_state(void)
{
    return (video_stage_encoded_recorder_config.startRec != VIDEO_ENCODED_STOPPED);
}

video_encoded_record_state video_stage_encoded_complete_recorder_state (void)
{
    return video_stage_encoded_recorder_config.startRec;
}

void video_stage_encoded_recorder_enable(bool_t enable, uint32_t timestamp)
{
    printf("Recording %s...\n", enable ? "started" : "stopped");
    video_stage_encoded_recorder_handle (&video_stage_encoded_recorder_config, (enable ? PIPELINE_MSG_START : PIPELINE_MSG_STOP), NULL, &timestamp);
}

void video_stage_encoded_recorder_com_timeout (void)
{
    if (VIDEO_ENCODED_WAITING_STREAM_END == video_stage_encoded_recorder_config.startRec)
    {
        video_stage_encoded_recorder_force_stop ();
    }
}

void video_stage_encoded_recorder_force_stop (void)
{
    printf ("Force recording stop\n");
    video_stage_encoded_recorder_handle (&video_stage_encoded_recorder_config, PIPELINE_MSG_SUSPEND, NULL, NULL);
}

C_RESULT video_stage_encoded_recorder_open(video_stage_encoded_recorder_config_t *cfg)
{

    if (0 == cfg->fps)
    {
        cfg->fps = 30;
    }
    cfg->startRec=VIDEO_ENCODED_STOPPED;
    cfg->lastStreamId = UINT16_MAX;
    cfg->currentStreamId = UINT16_MAX;

    cfg->video = NULL;

#ifndef USE_ELINUX
    vp_os_mutex_init (&cfg->videoMutex);
    ardrone_video_remove_all (VIDEO_ENCODED_FILE_DEFAULT_PATH);
#endif

#ifdef USE_ELINUX
    if (cfg->use_asynchronous_mode){
        /* Create a pipe to send video slices to the recording thread */
        pipe((int*)(&cfg->com_pipe));

        /* Create the above-mentioned thread */
        vp_os_thread_create(thread_video_stage_encoded_recorder,
                            (void*)cfg,
                            &cfg->thread_handle,
                            cfg->thread_priority,
	  						  "VideoFlashRec",
                            (void*)stack_video_stage_encoded_recorder,
                            sizeof(stack_video_stage_encoded_recorder),
                            &cfg->thread );
    }
#endif

    return C_OK;
}

int frameIsLastFrame (uint8_t *data)
{
    int retVal = 0;
    if (PAVE_CHECK (data))
    {
        parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)data;
        if (PAVE_CTRL_LAST_FRAME_IN_STREAM & PaVE->control)
        {
            retVal = 1;
        }
    }
    return retVal;
}

#ifdef USE_ELINUX
int ardrone_video_usb_key_get_free_space(void);
#endif

C_RESULT video_stage_encoded_recorder_transform(video_stage_encoded_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
#ifdef USE_ELINUX
    /* Handling the asynchronous mode */
    video_stage_encoded_recorder_msg_t asyn_msg;
#endif

    vp_os_mutex_lock (&out->lock);

#ifndef USE_ELINUX
    vp_os_mutex_lock (&cfg->videoMutex);
#endif

    // Copy in to out
    if (NULL != in && NULL != out)
    {
        out->numBuffers   = in->numBuffers;
        out->indexBuffer  = in->indexBuffer;
        out->lineSize     = in->lineSize;
        out->size         = in->size;
        out->status       = in->status;
        out->buffers      = in->buffers;
        out->status       = VP_API_STATUS_PROCESSING;
    }

#ifdef USE_ELINUX
    /* The drone pipeline may call the recorder with an empty input */

    if (NULL==in || NULL==in->buffers || in->size<1) { vp_os_mutex_unlock (&out->lock); return C_OK; }
#endif

    uint8_t *slice = in->buffers[in->indexBuffer];
    parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)slice;

    if (cfg->lastStreamId == PaVE->stream_id)
    {
        VER_PRINT ("Got a video slice from an old stream, skipping\n");
#ifndef USE_ELINUX
        vp_os_mutex_unlock (&cfg->videoMutex);
#endif
        vp_os_mutex_unlock (&out->lock);
        return C_OK;
    }

    if(cfg->startRec == VIDEO_ENCODED_START_RECORD)
    {
        ardrone_video_error_t vError = ARDRONE_VIDEO_NO_ERROR;

#ifdef USE_ELINUX
        /* Determine the amount of video data we can store */
      cfg->quota = ardrone_video_usb_key_get_free_space() /* kbytes */ - ( 100 * 1024 ); /* Keep 100 megabytes free */
        if(cfg->quota<0) { cfg->quota = 0; }
        printf("Flash video recording : available space : %dkb (time est. %d\'%d\") - (keeping a 100Mb margin on %s)\n",
               cfg->quota,(cfg->quota>>10)/60,(cfg->quota>>10)%60,
               VIDEO_ENCODED_FILE_DEFAULT_PATH);

        if (cfg->use_asynchronous_mode)
        {
            cfg->startRec=VIDEO_ENCODED_RECORDING;
        }
        else{
#endif

            cfg->video = ardrone_video_start (cfg->video_filename, cfg->fps, VIDEO_FORMAT, &vError);
            if (ARDRONE_VIDEO_SUCCEEDED (vError) && NULL != cfg->video)
            {
                cfg->startRec=VIDEO_ENCODED_RECORDING;
                cfg->currentStreamId = PaVE->stream_id;
            }
            else
            {
                VER_PRINT ("An error occured when creating video");
                cfg->startRec=VIDEO_ENCODED_STOPPED;
            }

#ifdef USE_ELINUX
        }
#endif

    }

    if((cfg->startRec == VIDEO_ENCODED_RECORDING) ||
       (cfg->startRec == VIDEO_ENCODED_WAITING_STREAM_END))
    {

#if USE_LINUX
        /* Quick and dirty debugging inside AR.Drone Navigation */
        if (PAVE_CHECK(slice))
        {
            parrot_video_encapsulation_t * pave = (parrot_video_encapsulation_t*) slice;
            printf("Recording frame %dx%d num %d   \n\033[1A",pave->display_width,pave->display_height,pave->frame_number);
            if (pave->control & PAVE_CTRL_LAST_FRAME_IN_STREAM) {
                printf("\n\nEnd of stream PaVE received !\n\n");
            }
        }
#endif

#ifdef USE_ELINUX
        if (( ( in->size /*bytes*/ ) >>10 ) > cfg->quota )
        {
            printf("Flash video recording : out of disk space.\n");
            cfg->startRec=VIDEO_ENCODED_STOPPED;
        }
        else
        {
            cfg->quota /*kbytes*/ -=  ( ( in->size /*bytes*/ ) >>10 );
    	  navdata_set_hdvideo_usbkey_freespace(cfg->quota);

            if (cfg->use_asynchronous_mode)
            {
                /* Create a RAM buffer to hold a temporary copy of the slice to write */
                asyn_msg.sliceSize = in->size;
                asyn_msg.slice     = vp_os_malloc(asyn_msg.sliceSize);
                if (asyn_msg.slice){
                    vp_os_memcpy(asyn_msg.slice,slice,asyn_msg.sliceSize);
                    /* Send the copy to thread which does the actual writing */
                    //printf("----> Writing %p %d bytes\n",asyn_msg.slice,asyn_msg.sliceSize);
                    write(cfg->com_pipe[1],&asyn_msg,sizeof(asyn_msg));
                }
    		  else
    		  {
    			  printf("Flash video recording : out of RAM - dropping frame.\n");
    		  }
            }
            else
            {
#endif



                ardrone_video_error_t vError = ardrone_video_addSlice (cfg->video, slice);
                if (ARDRONE_VIDEO_FAILED (vError))
                {
                    char errBuf [50] = {0};
                    ardrone_video_error_string (vError, errBuf, 50);
                    VER_PRINT ("Error while recording frame : %s", errBuf);
                    ardrone_video_cleanup (&(cfg->video));
                    cfg->startRec = VIDEO_ENCODED_STOPPED;
                }

                if (1 == frameIsLastFrame (slice))
                {
                    VER_PRINT ("Received last frame for current stream, finishing video\n");
                    video_stage_encoded_recorder_finish (cfg);
                }

#ifdef USE_ELINUX
            }
        }
#endif
    }
#ifndef USE_ELINUX
    vp_os_mutex_unlock (&cfg->videoMutex);
#endif
    vp_os_mutex_unlock (&out->lock);
    return C_OK;
}

C_RESULT video_stage_encoded_recorder_close(video_stage_encoded_recorder_config_t *cfg)
{
    if (NULL != cfg->video)
    {
        ardrone_video_finish (&(cfg->video));
    }
    return C_OK;
}
