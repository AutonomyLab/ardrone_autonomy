#include <ardrone_tool/Video/video_stage_latency_estimation.h>
#include <ardrone_tool/Video/video_stage_decoder.h>
#include <VP_Os/vp_os_malloc.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdio.h>

/**
 * IPHONE DEBUG AREA
 */
float DEBUG_latency = 0.0;

C_RESULT latency_estimation_stage_handle_message(void *cfg, PIPELINE_MSG msg_id, void *callback, void *param)
{
  printf("Latency estimator message handler.\n");
  return (C_OK);
}


C_RESULT
latency_estimation_stage_open( vp_stages_latency_estimation_config_t *cfg )
{
  cfg->state = 0;
  cfg->w = cfg->h = 0;
  return C_OK;
}

#define N (10)
typedef struct __stat { float min; float avg; } stat_t;
static stat_t average(float f)
{
  static float p[N] = {0};
  static int idx =0;
  int i;
  float avg;
  float fmin;
  stat_t stat;

  p[idx] = f;
  idx = (idx+1)%N;
  avg = 0.0f;
  fmin = p[0];
  for (i=0;i<N;i++) { avg+=p[i]; fmin = (p[i]<fmin) ? p[i]:fmin; }
  avg = avg / (float)N;
  stat.min=fmin;
  stat.avg=avg;
  return stat;
}

C_RESULT
latency_estimation_stage_transform( vp_stages_latency_estimation_config_t *cfg , vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  video_decoder_config_t * vec;
  static int previous_state = 0;
  static long int timestamp1 = 0 , timestamp2 = 0;
  struct timeval time;

  long int delta;
  stat_t stat;


  static char * red_buffer   = NULL;
  static char * green_buffer = NULL;
  static char * blue_buffer  = NULL;

  int i,j;
  int bufsize;
  int m;
  float n;
  float r=0.0f,g=0.0f,b=0.0f;

  vec = (video_decoder_config_t *)cfg->last_decoded_frame_info;

  if(out->status == VP_API_STATUS_INIT) // Init only code
    {
      out->status      = VP_API_STATUS_PROCESSING;
    }
      out->numBuffers  = in->numBuffers;
      out->buffers     = in->buffers;
      out->indexBuffer = in->indexBuffer;
      out->size        = in->size;
      out->lineSize    = in->lineSize;

  if(out->status == VP_API_STATUS_PROCESSING)
    {
      /* Build three RGB buffers whose dimensions are the same than the decoded video stream */
      if ( cfg->w != vec->dst_picture->width || cfg->h != vec->dst_picture->height)
        {
          cfg->w = vec->dst_picture->width;
          cfg->h = vec->dst_picture->height;

          if (red_buffer)  { vp_os_sfree((void**)&red_buffer);   }
          if (green_buffer){ vp_os_sfree((void**)&green_buffer); }
          if (blue_buffer) { vp_os_sfree((void**)&blue_buffer);  }

          switch (vec->dst_picture->format)
            {
            case PIX_FMT_RGB24:
              bufsize = cfg->w * cfg->h * 3;
              break;
            case PIX_FMT_RGB565:
            default:
              bufsize = cfg->w * cfg->h * 2;
              break;
            }

          red_buffer   = vp_os_malloc( bufsize );
          green_buffer = vp_os_malloc( bufsize );
          blue_buffer  = vp_os_malloc( bufsize );

          if (red_buffer)   { vp_os_memset( red_buffer   , 0 , bufsize ); }
          if (green_buffer) { vp_os_memset( green_buffer , 0 , bufsize ); }
          if (blue_buffer)  { vp_os_memset( blue_buffer  , 0 , bufsize ); }

          switch (vec->dst_picture->format)
            {
            case PIX_FMT_RGB24:
              {
                for (i=0;i<bufsize;i+= 3)
                  {
                    red_buffer[i]     = 255;
                    green_buffer[i+1] = 255;
                    blue_buffer[i+2]  = 255;
                  }              
                break;
              }
            case PIX_FMT_RGB565:
            default:
              {
                for (i=0; i < bufsize; i+=2)
                  {
                    red_buffer[i+1] = 0xf8;
                    green_buffer[i+1] = 0x07;
                    green_buffer[i] = 0xe0;
                    blue_buffer[i] = 0x1f;
                  }
                break;
              }
            }
        }



      if (cfg->state == LE_WAITING)
	{
          out->numBuffers = in->numBuffers;
          out->indexBuffer = in->indexBuffer;
	}
      else
	{
          out->numBuffers = 1;
          out->indexBuffer = 0;
	}

      /* Switch state depending on what is seen in the decoded video stream */

#define L (3)
      /* Coordinates of the center pixel */

      r=0.0f;
      g=0.0f;
      b=0.0f;

      uint8_t redVal, greenVal, blueVal;
      for (i=-L;i<=L;i++)	
	{
          for (j=-L;j<=L;j++)	
            {
              
              m = ( cfg->w / 2 +i) + ( cfg->w * (cfg->h/2 +j) );
              
              switch (vec->dst_picture->format)
                {
                case PIX_FMT_RGB24:
                  redVal = in->buffers[in->indexBuffer][3*m];
                  greenVal = in->buffers[in->indexBuffer][3*m+1];
                  blueVal = in->buffers[in->indexBuffer][3*m+2];
                  r += (float)(redVal);
                  g += (float)(greenVal);
                  b += (float)(blueVal);
                  break;
                case PIX_FMT_RGB565:
                  redVal = (in->buffers[in->indexBuffer][2*m+1] >> 3);
                  greenVal = ((in->buffers[in->indexBuffer][2*m+1] & 0x07) << 3) + (in->buffers[in->indexBuffer][2*m] >> 5);
                  blueVal = (in->buffers[in->indexBuffer][2*m] & 0x1f);
                  r += (float)(redVal);
                  g += (float)(greenVal >> 1);
                  b += (float)(blueVal);
                  break;
                default:
                  r = g = b = 0.0;
                  break;
                }
            }
        }
      
      n=(float)((2*L+1)*(2*L+1));
      r/=n;
      g/=n;
      b/=n;

#if 0
      printf ("R %3.1f | G %3.1f | B %3.1f --- State : %d\n", r, g, b, cfg->state);
#endif


#define K (1.0f)

      switch (cfg->state)
        {
        default:
        case LE_DISABLED:
          /* Nothing to do */
          break;
        case LE_WAITING: /* waiting to see GREEN */
          if ( (g>(K*r)) && (g>(K*b)) ) { cfg->state = LE_START; }
          break;
        case LE_START: /* waiting to see red */
          if      ( (g>(K*r)) && (g>(K*b)) ) { cfg->state = LE_START; }
          else if ( (r>(K*b)) && (r>(K*g)) ) { cfg->state = LE_COLOR2; }
          else    { cfg->state = LE_WAITING; printf("================> %f %f %f \n",r,g,b);}
          break;
        case LE_COLOR1: /* GREEN requested */
          if      ( (r>(K*b)) && (r>(K*g)) ) { cfg->state = LE_COLOR2; }
          else if ( (b>(K*g)) && (b>(K*r)) ) { cfg->state = LE_COLOR1; }
          else    { cfg->state = LE_WAITING; printf("================> %f %f %f \n",r,g,b);}
          break;
        case LE_COLOR2: /* BLUE requested */
          /* Display a blue picture */
          if       ( (b>(K*g)) && (b>(K*r)) ) { cfg->state = LE_COLOR1; }
          else if  ( (r>(K*b)) && (r>(K*g)) ) { cfg->state = LE_COLOR2; }
          else     { cfg->state = LE_WAITING; printf("================> %f %f %f \n",r,g,b);}
          
          break;
        }
      
      switch (cfg->state)
	{
        default:
        case LE_DISABLED:
          /* Forward the video stream */
          out->buffers = in->buffers;
          break;
        case LE_WAITING:
          /* Display a gren picture announcing we are waiting for the user to
           * put the drone camera in front of the displayed picture.
           */
          out->buffers = (uint8_t**)&(green_buffer);
          break;
        case LE_START:
        case LE_COLOR1:
          /* Display a red picture */
          out->buffers = (uint8_t**)&(red_buffer);
          break;
        case LE_COLOR2:
          /* Display a blue picture */
          out->buffers = (uint8_t**)&(blue_buffer);
          break;
	}
    }
  
  if (previous_state!=cfg->state && cfg->state>=LE_COLOR1 && previous_state>=LE_COLOR1)
    {
      gettimeofday(&time,NULL);
      timestamp2 = time.tv_sec*1000 + time.tv_usec/1000;
      delta = timestamp2 - timestamp1;
      stat = average(delta);
    
      if (timestamp1!=0 && delta>30 && delta<1000)  /* test on delta to remove false measures */
        {
          printf("State : %d - Middle pixel value : %3.0f %3.0f %3.0f -- Time since last change : %d milliseconds ",
                 cfg->state,
                 r,g,b,
                 (int)delta);
        
          printf("- avg : %4.1f ms - min : %4.f ms\n",stat.avg,stat.min );
          DEBUG_latency = stat.avg;
        }
    
      timestamp1 = timestamp2;
    }
  
  previous_state = cfg->state;
  
  return C_OK;
}

C_RESULT
latency_estimation_stage_close( vp_stages_latency_estimation_config_t *cfg )
{
  return C_OK;
}


const vp_api_stage_funcs_t vp_stages_latency_estimation_funcs =
  {
    (vp_api_stage_handle_msg_t) latency_estimation_stage_handle_message,
    (vp_api_stage_open_t)latency_estimation_stage_open,
    (vp_api_stage_transform_t)latency_estimation_stage_transform,
    (vp_api_stage_close_t)latency_estimation_stage_close
  };
