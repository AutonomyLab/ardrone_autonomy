/*
 *  video_stage_ffmpeg_decoder.c
 *  ARDroneLib
 *
 *  Created by f.dhaeyer on 09/12/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */

#ifdef FFMPEG_SUPPORT

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <ardrone_tool/Video/video_stage_ffmpeg_decoder.h>
#include <Maths/time.h>
#include <math.h>
#include <sys/time.h>
#include <video_encapsulation.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* CONFIGURATION */

/**
 * Enable (1) to display FPS
 */
#define DISPLAY_FPS (0)
/**
 * Display FPS after N frames
 * - default : 20
 */
#define DISPLAY_FPS_FRAMES (60)

/**
 * Enable (1) to display missed/dropped frames
 */
#define DISPLAY_DROPPED_FRAMES (0)

/**
 * Enable (1) to allow decoding of non-PaVE MPEG4/h264 frames
 * Disable (0) for compatibility with VLIB/P264 (won't try to decode if no PaVE is found)
 */
#define FAKE_PaVE (1)
/**
 * Default codec for non-PaVE frames :
 * (0) - h264
 * (1) - MPEG4
 */
#define FAKE_PaVE_CODEC_ID (0)

/**
 * Wait for I-Frame mode :
 * - (0) : display each received frame
 * - (1) : when a frame is lost, wait for next I-Frame
 */
#define WAIT_FOR_I_FRAME (1)

#define FFMPEG_DEBUG_ENABLE_ON_GLOBAL_DEBUG (0) ///< Auto enable debug while on debug mode
#define FFMPEG_DEBUG_ENABLE (0)                 ///< Force enable of debug for this file

/* END OF CONFIGURATION */

#if DISPLAY_FPS
#define NUM_SAMPLES DISPLAY_FPS_FRAMES
#endif

#if FAKE_PaVE_CODEC_ID == 0
#define FAKE_PaVE_CODEC CODEC_MPEG4_AVC
#elif FAKE_PaVE_CODEC_ID == 1
#define FAKE_PaVE_CODEC CODEC_MPEG4_VISUAL
#else
#warning FAKE_PaVE_CODEC_ID : unknown value -> falling back to MPEG4
#define FAKE_PaVE_CODEC CODEC_MPEG4_VISUAL
#endif

#if FFMPEG_DEBUG_ENABLE || (FFMPEG_DEBUG_ENABLE_ON_GLOBAL_DEBUG && defined (DEBUG))
#define __FFMPEG_DEBUG_ENABLED (1)
#define FFMPEG_LOG_LEVEL (AV_LOG_WARNING)
#else
#define __FFMPEG_DEBUG_ENABLED (0)
#define FFMPEG_LOG_LEVEL (AV_LOG_QUIET)
#endif

#if __FFMPEG_DEBUG_ENABLED
#define FFMPEG_DEBUG(...)                                             \
  do                                                                  \
    {                                                                 \
      printf ("FFMPEG-DEBUG (%s @ %d) : ", __FUNCTION__, __LINE__);   \
      printf (__VA_ARGS__);                                           \
      printf ("\n");                                                  \
    } while (0)
#else
#define FFMPEG_DEBUG(...)
#endif

#if DISPLAY_DROPPED_FRAMES
int missed_frames = 0;
int dropped_frames = 0;
int previous_ok_frame = 0;
#endif

const vp_api_stage_funcs_t ffmpeg_decoding_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) ffmpeg_stage_decoding_open,
  (vp_api_stage_transform_t) ffmpeg_stage_decoding_transform,
  (vp_api_stage_close_t) ffmpeg_stage_decoding_close
};

C_RESULT ffmpeg_stage_decoding_open(ffmpeg_stage_decoding_config_t *cfg)
{
  cfg->num_picture_decoded = 0;
  
  /* must be called before using avcodec lib */
  avcodec_init();
  
  /* register all the codecs */
  avcodec_register_all();
  
  av_log_set_level(FFMPEG_LOG_LEVEL);

  cfg->pCodecMP4 = avcodec_find_decoder (CODEC_ID_MPEG4);
  cfg->pCodecH264 = avcodec_find_decoder (CODEC_ID_H264);
  if(NULL == cfg->pCodecMP4 || NULL == cfg->pCodecH264) 
    {
      fprintf(stderr, "Unsupported codec!\n");
      return C_FAIL; // Codec not found
    }

  cfg->pCodecCtxMP4 = avcodec_alloc_context();
  cfg->pCodecCtxH264 = avcodec_alloc_context();
  if (NULL == cfg->pCodecCtxMP4 || NULL == cfg->pCodecCtxH264)
    {
      fprintf(stderr, "Impossible to allocate codec context\n");
      return C_FAIL; // Allocation failure
    }

  cfg->pCodecCtxMP4->pix_fmt = PIX_FMT_YUV420P;
  cfg->pCodecCtxMP4->skip_frame = AVDISCARD_DEFAULT;
  cfg->pCodecCtxMP4->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  cfg->pCodecCtxMP4->error_recognition = FF_ER_CAREFUL;
  cfg->pCodecCtxMP4->skip_loop_filter = AVDISCARD_DEFAULT;
  cfg->pCodecCtxMP4->workaround_bugs = FF_BUG_AUTODETECT;
  cfg->pCodecCtxMP4->codec_type = AVMEDIA_TYPE_VIDEO;
  cfg->pCodecCtxMP4->codec_id = CODEC_ID_MPEG4;
  cfg->pCodecCtxMP4->skip_idct = AVDISCARD_DEFAULT;
  
  cfg->pCodecCtxH264->pix_fmt = PIX_FMT_YUV420P;
  cfg->pCodecCtxH264->skip_frame = AVDISCARD_DEFAULT;
  cfg->pCodecCtxH264->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  cfg->pCodecCtxH264->error_recognition = FF_ER_CAREFUL;
  cfg->pCodecCtxH264->skip_loop_filter = AVDISCARD_DEFAULT;
  cfg->pCodecCtxH264->workaround_bugs = FF_BUG_AUTODETECT;
  cfg->pCodecCtxH264->codec_type = AVMEDIA_TYPE_VIDEO;
  cfg->pCodecCtxH264->codec_id = CODEC_ID_H264;
  cfg->pCodecCtxH264->skip_idct = AVDISCARD_DEFAULT;
  		
  // Open codec
  if(avcodec_open(cfg->pCodecCtxMP4, cfg->pCodecMP4) < 0)
    {
      fprintf (stderr, "Error opening MP4 codec\n");
      return C_FAIL;
    }
  if(avcodec_open(cfg->pCodecCtxH264, cfg->pCodecH264) < 0)
    {
      fprintf (stderr, "Error opening h264 codec\n");
      return C_FAIL;
    }

  cfg->pFrameOutput = avcodec_alloc_frame();
  cfg->pFrame = avcodec_alloc_frame();
  if (NULL == cfg->pFrameOutput || NULL == cfg->pFrame)
    {
      fprintf (stderr, "Unable to alloc frames");
      return C_FAIL;
    }

  cfg->bufferArray = (uint8_t **)vp_os_malloc (sizeof (uint8_t *));
  if (NULL == cfg->bufferArray)
    {
      fprintf (stderr, "Unable to alloc output buffer");
      return C_FAIL;
    }
  cfg->buffer = NULL;
  cfg->img_convert_ctx = NULL;
  
  return C_OK;
}

void empty_av_log_callback (void *ptr, int level, const char *fmt, va_list vl)
{
  // Empty callback so we can hide all ffmpeg av_log outputs
}

#if __FFMPEG_DEBUG_ENABLED
void ffmpeg_decoder_dumpPave (parrot_video_encapsulation_t *PaVE)
{
  printf ("Signature : \"%c%c%c%c\" [0x%02x][0x%02x][0x%02x][0x%02x]\n", PaVE->signature[0], PaVE->signature[1],
          PaVE->signature[2], PaVE->signature[3], PaVE->signature[0], PaVE->signature[1], PaVE->signature[2], PaVE->signature[3]);
  printf ("Codec : %s\n", (PaVE->video_codec == CODEC_MPEG4_VISUAL) ? "MP4" : ((PaVE->video_codec == CODEC_MPEG4_AVC) ? "H264" : "Unknown"));
  printf ("StreamID : %d \n", PaVE->stream_id);
  printf ("Encoded dims : %d x %d\n", PaVE->encoded_stream_width, PaVE->encoded_stream_height);
  printf ("Display dims : %d x %d\n", PaVE->display_width, PaVE->display_height);
  printf ("Header size  : %d (PaVE size : %u)\n", PaVE->header_size, sizeof (parrot_video_encapsulation_t));
  printf ("Payload size : %d\n", PaVE->payload_size);
  printf ("Frame Type / Number : %s : %d : slide %d/%d\n",
     (PaVE->frame_type == FRAME_TYPE_P_FRAME) ? "P-Frame" : ((PaVE->frame_type == FRAME_TYPE_I_FRAME) ? "I-Frame" : "IDR-Frame"), 
     PaVE->frame_number,
     PaVE->slice_index+1,
     PaVE->total_slices);
}
#endif

static inline bool_t check_and_copy_PaVE (parrot_video_encapsulation_t *PaVE, vp_api_io_data_t *data, parrot_video_encapsulation_t *prevPaVE, bool_t *dimChanged)
{
    
  parrot_video_encapsulation_t *localPaVE = (parrot_video_encapsulation_t *)data->buffers[data->indexBuffer];
  if (localPaVE->signature[0] == 'P' &&
      localPaVE->signature[1] == 'a' &&
      localPaVE->signature[2] == 'V' &&
      localPaVE->signature[3] == 'E')
  {
      //FFMPEG_DEBUG("Found a PaVE");
      vp_os_memcpy (prevPaVE, PaVE, sizeof (parrot_video_encapsulation_t)); // Make a backup of previous PaVE so we can check if things have changed
      
      vp_os_memcpy (PaVE, localPaVE, sizeof (parrot_video_encapsulation_t)); // Copy PaVE to our local one
      
#if __FFMPEG_DEBUG_ENABLED
      printf ("------------------------------------\n");
      printf ("PREV : ");
      ffmpeg_decoder_dumpPave (prevPaVE);
      printf ("CURR : ");
      ffmpeg_decoder_dumpPave (PaVE);
      printf ("------------------------------------\n");
      
      
      
#endif
      if (prevPaVE->encoded_stream_width  != PaVE->encoded_stream_width   ||
          prevPaVE->encoded_stream_height != PaVE->encoded_stream_height  ||
          prevPaVE->display_width         != PaVE->display_width          ||
          prevPaVE->display_width         != PaVE->display_width          ||
          prevPaVE->stream_id             != PaVE->stream_id                )
        {
          *dimChanged = TRUE;
        }
      else
        {
          *dimChanged = FALSE;
        }
      data->size = localPaVE->payload_size;
      memmove(data->buffers[data->indexBuffer], &(data->buffers[data->indexBuffer])[localPaVE->header_size], data->size);
#if DISPLAY_DROPPED_FRAMES
      missed_frames += PaVE->frame_number - prevPaVE->frame_number - 1;
#endif
      return TRUE;
    }
  else
    {    
      FFMPEG_DEBUG("No PaVE, signature was [%c][%c][%c][%c]",
                   localPaVE->signature[0],
                   localPaVE->signature[1],
                   localPaVE->signature[2],
                   localPaVE->signature[3]);
#if FAKE_PaVE
      PaVE->encoded_stream_width = 640;
      PaVE->encoded_stream_height = 368;
      PaVE->display_width = 640;
      PaVE->display_height = 360;
      PaVE->video_codec = FAKE_PaVE_CODEC;
      PaVE->frame_type = FRAME_TYPE_I_FRAME;
      vp_os_memcpy (prevPaVE, PaVE, sizeof (parrot_video_encapsulation_t));
      *dimChanged = FALSE;
      return TRUE;
#else
      return FALSE;
#endif
    }
}

C_RESULT ffmpeg_stage_decoding_transform(ffmpeg_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  static const int        sws_flags = SWS_FAST_BILINEAR;
  AVCodecContext  *pCodecCtxMP4 = cfg->pCodecCtxMP4;
  AVCodecContext  *pCodecCtxH264 = cfg->pCodecCtxH264;
  AVFrame         *pFrame = cfg->pFrame;
  AVFrame	  *pFrameOutput = cfg->pFrameOutput;
  static AVPacket packet;
  int	frameFinished = 0;
    
  bool_t frameDimChanged = FALSE;
    static parrot_video_encapsulation_t __attribute__ ((aligned (4))) PaVE;
    static parrot_video_encapsulation_t __attribute__ ((aligned (4))) prevPaVE;
    
#if WAIT_FOR_I_FRAME
  static bool_t waitForIFrame = TRUE;
#endif
    
#ifdef NUM_SAMPLES
  static struct timeval start_time, start_time2;
  static int numsamples = 0;
#endif
    
  if (0 == in->size) // No frame
    {
      FFMPEG_DEBUG ("in->size is zero, don't do anything");
      return C_OK;
    }
  
  vp_os_mutex_lock( &out->lock );
  
  if(out->status == VP_API_STATUS_INIT) // Init only code
    {		
      out->numBuffers   = 1;
      out->buffers      = cfg->bufferArray;
      out->buffers[0]   = NULL;
      out->indexBuffer  = 0;
      out->lineSize     = 0;
        
      av_init_packet(&packet);
 
        
#if __FFMPEG_DEBUG_ENABLED
#else
      av_log_set_callback (&empty_av_log_callback);
#endif
    }
 
  if (! check_and_copy_PaVE(&PaVE, in, &prevPaVE, &frameDimChanged))
    {
      FFMPEG_DEBUG("Received a frame without PaVE informations");
      vp_os_mutex_unlock( &out->lock );
      return C_FAIL;
    }
    
  if ((out->status == VP_API_STATUS_INIT) || frameDimChanged) // Init and "new frame dimensions" code
    {
      pCodecCtxMP4->width = PaVE.encoded_stream_width;
      pCodecCtxMP4->height = PaVE.encoded_stream_height;
      pCodecCtxH264->width = PaVE.encoded_stream_width;
      pCodecCtxH264->height = PaVE.encoded_stream_height;
		
      cfg->src_picture.width = PaVE.display_width;
      cfg->src_picture.height = PaVE.display_height;
      cfg->src_picture.format = pCodecCtxH264->pix_fmt;
      cfg->dst_picture.width = PaVE.display_width;
      cfg->dst_picture.height = PaVE.display_height;
		
      out->size = avpicture_get_size(cfg->dst_picture.format, cfg->dst_picture.width, cfg->dst_picture.height);
      cfg->buffer = (uint8_t *)av_realloc(cfg->buffer, out->size * sizeof(uint8_t));
      out->buffers[0] = cfg->buffer;
		
      avpicture_fill((AVPicture *)pFrameOutput, (uint8_t*)out->buffers[out->indexBuffer], cfg->dst_picture.format,
                     cfg->dst_picture.width, cfg->dst_picture.height);
		
        
      cfg->img_convert_ctx = sws_getCachedContext(cfg->img_convert_ctx, PaVE.display_width, PaVE.display_height,
                                             pCodecCtxH264->pix_fmt, PaVE.display_width, PaVE.display_height,
                                             cfg->dst_picture.format, sws_flags, NULL, NULL, NULL);

      if (out->status == VP_API_STATUS_INIT)
        {
#ifdef NUM_SAMPLES
          gettimeofday(&start_time, NULL);
#endif		
          out->status = VP_API_STATUS_PROCESSING;
          FFMPEG_DEBUG("End of init");
        }
    }

#if	WAIT_FOR_I_FRAME
  if ( (PaVE.frame_number != (prevPaVE.frame_number +1)) 
        && 
        ( PaVE.frame_number != prevPaVE.frame_number || PaVE.slice_index != (prevPaVE.slice_index+1) )   )
    {
      FFMPEG_DEBUG ("Missed a frame :\nPrevious was %d of type %d\nNew is %d of type %d", prevPaVE.frame_number, prevPaVE.frame_type,
                    PaVE.frame_number, PaVE.frame_type);
      waitForIFrame = TRUE;  
    }
    
#if DISPLAY_DROPPED_FRAMES
  if (waitForIFrame && PaVE.frame_type == FRAME_TYPE_P_FRAME)
    {
      FFMPEG_DEBUG ("Dropped a P frame\n");
      dropped_frames++;
    }
#endif
    
    
  if(out->status == VP_API_STATUS_PROCESSING && (!waitForIFrame || (PaVE.frame_type == FRAME_TYPE_IDR_FRAME) || (PaVE.frame_type == FRAME_TYPE_I_FRAME))) // Processing code
    {
      waitForIFrame = FALSE;
#else
      if(out->status == VP_API_STATUS_PROCESSING) // Processing code  
        {
#endif
          /* The 'check_and_copy_PaVE' function already removed the PaVE from the 'in' buffer */
          packet.data = ((unsigned char*)in->buffers[in->indexBuffer]);
          packet.size = in->size;
          FFMPEG_DEBUG("Size : %d", packet.size);
            
        
#ifdef NUM_SAMPLES
          struct timeval end_time;
          static float32_t frame_decoded_time = 0;

          gettimeofday(&start_time2, NULL);
#endif
          // Decode video frame
          if (PaVE.video_codec == CODEC_MPEG4_VISUAL)
            {
              avcodec_decode_video2 (pCodecCtxMP4, pFrame, &frameFinished, &packet);
            }
          else if (PaVE.video_codec == CODEC_MPEG4_AVC)
            {
              avcodec_decode_video2 (pCodecCtxH264, pFrame, &frameFinished, &packet);
            }
        
          // Did we get a video frame?
          if(frameFinished)
            {
              pFrameOutput->data[0] = (uint8_t*)out->buffers[out->indexBuffer];
              sws_scale(cfg->img_convert_ctx, (const uint8_t *const*)pFrame->data, 
                        pFrame->linesize, 0, 
                        PaVE.display_height,
                        pFrameOutput->data, pFrameOutput->linesize);
				
              cfg->num_picture_decoded++;

#ifdef NUM_SAMPLES
              gettimeofday(&end_time, NULL);
              frame_decoded_time += ((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0) - (start_time2.tv_sec * 1000.0 + start_time2.tv_usec / 1000.0));

              if(numsamples++ > NUM_SAMPLES)
                {
                  float32_t value = ((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0) - (start_time.tv_sec * 1000.0 + start_time.tv_usec / 1000.0));
					
                  printf("Frames decoded in average %f fps, received and decoded in average %f fps\n", (1000.0 / (frame_decoded_time / (float32_t)NUM_SAMPLES)), 1000.0 / (value / (float32_t)NUM_SAMPLES));
                  gettimeofday(&start_time, NULL);
                  frame_decoded_time = 0;
                  numsamples = 0;
                }					
#endif
            }
          else
            {
        	  /* Skip frames are usually 7 bytes long
        	   * and make FFMPEG return an error. It is however normal to get
        	   * skip frames from the drone.
        	   */
        	  if (7!=PaVE.payload_size)
              printf ("Decoding failed for a %s\n", (PaVE.frame_type == FRAME_TYPE_P_FRAME) ? "P Frame" : "I Frame");
            }
        
#if DISPLAY_DROPPED_FRAMES
          if ((PaVE.frame_type == FRAME_TYPE_IDR_FRAME) || (PaVE.frame_type == FRAME_TYPE_I_FRAME))
            {
              if (previous_ok_frame != 0)
                {
                  static int globalMiss = 0, globalDrop = 0, globalFrames = 0;
                  globalMiss += missed_frames;
                  globalDrop += dropped_frames;
                  int globalMissDrop = globalMiss + globalDrop;
                  int total_miss = missed_frames + dropped_frames;
                  int total_frames = PaVE.frame_number - previous_ok_frame;
                  globalFrames += total_frames;
                  float missPercent = (100.0 * missed_frames) / (1.0 * total_frames);
                  float dropPercent = (100.0 * dropped_frames) / (1.0 * total_frames);
                  float totalPercent = (100.0 * total_miss) / (1.0 * total_frames);
                  float missMean = (100.0 * globalMiss) / (1.0 * globalFrames);
                  float dropMean = (100.0 * globalDrop) / (1.0 * globalFrames);
                  float totalMean = (100.0 * globalMissDrop) / (1.0 * globalFrames);
                  printf ("LAST %4d F => M %4d (%4.1f%%) / D %4d (%4.1f%%) / T %4d (%4.1f%%) <=> ALL %4d F => M %4d (%4.1f%%) / D %4d (%4.1f%%) / T %4d (%4.1f%%)\n", total_frames, missed_frames, missPercent, dropped_frames, dropPercent, total_miss, totalPercent, globalFrames, globalMiss, missMean, globalDrop, dropMean, globalMissDrop, totalMean);
                }
              missed_frames = 0; dropped_frames = 0;
              previous_ok_frame = PaVE.frame_number;
            }
#endif
        
	}
	
      vp_os_mutex_unlock( &out->lock );
	
      return C_OK;
    }

#define FFMPEG_CHECK_AND_FREE(pointer, freeFunc)        \
  do                                                    \
    {                                                   \
      if (NULL != pointer)                              \
        {                                               \
          freeFunc (pointer);                           \
          pointer = NULL;                               \
        }                                               \
    } while (0)

#define FFMPEG_CHECK_AND_FREE_WITH_CALL(pointer, func, freeFunc)        \
  do                                                                    \
    {                                                                   \
      if (NULL != pointer)                                              \
        {                                                               \
          func (pointer);                                               \
          freeFunc (pointer);                                           \
          pointer = NULL;                                               \
        }                                                               \
    } while (0)
  

  C_RESULT ffmpeg_stage_decoding_close(ffmpeg_stage_decoding_config_t *cfg)
  {
    FFMPEG_CHECK_AND_FREE_WITH_CALL(cfg->pCodecCtxMP4, avcodec_close, av_free);
    FFMPEG_CHECK_AND_FREE_WITH_CALL(cfg->pCodecCtxH264, avcodec_close, av_free);
    FFMPEG_CHECK_AND_FREE(cfg->pFrame, av_free);
    FFMPEG_CHECK_AND_FREE(cfg->pFrameOutput, av_free);
    FFMPEG_CHECK_AND_FREE(cfg->bufferArray, vp_os_free);
    FFMPEG_CHECK_AND_FREE(cfg->buffer, av_free);
    FFMPEG_CHECK_AND_FREE(cfg->img_convert_ctx, sws_freeContext);
    return C_OK;
  }

#endif
