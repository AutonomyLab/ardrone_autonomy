//
//  video_stage_tcp.c
//  ARDroneEngine
//
//  Created by nicolas on 15/07/11.
//  Copyright 2011 Parrot. All rights reserved.
//

#include "video_stage_tcp.h"
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <video_encapsulation.h>
#include <VP_Os/vp_os_assert.h>

/* CONFIGURATION */

#define VIDEO_TCP_DEBUG_ENABLE_ON_GLOBAL_DEBUG (0) ///< Auto enable debug while on debug mode
#define VIDEO_TCP_DEBUG_ENABLE (0)                 ///< Force enable of debug for this file

/* END OF CONFIGURATION */



#if VIDEO_TCP_DEBUG_ENABLE || (VIDEO_TCP_DEBUG_ENABLE_ON_GLOBAL_DEBUG && defined (DEBUG))
#define __VIDEO_TCP_DEBUG_ENABLED (1)
#else
#define __VIDEO_TCP_DEBUG_ENABLED (0)
#endif

#if __VIDEO_TCP_DEBUG_ENABLED
#define VIDEO_TCP_DEBUG(...)                                            \
  do                                                                    \
    {                                                                   \
      printf ("VIDEO_TCP-DEBUG (%s @ %d) : ", __FUNCTION__, __LINE__);  \
      printf (__VA_ARGS__);                                             \
      printf ("\n");                                                    \
    } while (0)
#else
#define VIDEO_TCP_DEBUG(...)
#endif


void video_stage_tcp_dumpPave (parrot_video_encapsulation_t *PaVE)
{
  printf ("Signature : \"%c%c%c%c\" [0x%02x][0x%02x][0x%02x][0x%02x]\n", PaVE->signature[0], PaVE->signature[1],
          PaVE->signature[2], PaVE->signature[3], PaVE->signature[0], PaVE->signature[1], PaVE->signature[2], PaVE->signature[3]);
  printf ("Frame Type / Number : %s : %d : slice %d/%d\n",
          (PaVE->frame_type == FRAME_TYPE_P_FRAME) ? "P-Frame" : ((PaVE->frame_type == FRAME_TYPE_I_FRAME) ? "I-Frame" : "IDR-Frame"),
          PaVE->frame_number,
          PaVE->slice_index+1,
          PaVE->total_slices);
  printf ("Codec : %s\n", (PaVE->video_codec == CODEC_MPEG4_VISUAL) ? "MP4" : ((PaVE->video_codec == CODEC_MPEG4_AVC) ? "H264" : "Unknown"));
  printf ("StreamID : %d \n", PaVE->stream_id);
  printf ("Encoded dims : %d x %d\n", PaVE->encoded_stream_width, PaVE->encoded_stream_height);
  printf ("Display dims : %d x %d\n", PaVE->display_width, PaVE->display_height);
  printf ("Header size  : %d\n", PaVE->header_size);
  printf ("Payload size : %d\n", PaVE->payload_size);
}



#define BUFFER_MAX_SIZE (cfg->maxPFramesPerIFrame * cfg->frameMeanSize)

const vp_api_stage_funcs_t video_stage_tcp_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) video_stage_tcp_open,
  (vp_api_stage_transform_t) video_stage_tcp_transform,
  (vp_api_stage_close_t) video_stage_tcp_close
};

static inline bool_t frameIsIFrame (uint8_t *framePointer)
{
  parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)framePointer;
  if ( (PaVE->frame_type == FRAME_TYPE_I_FRAME ||
        PaVE->frame_type == FRAME_TYPE_IDR_FRAME) && (PaVE->slice_index == 0) )
    {
      return TRUE;
    }
  return FALSE;
}

static inline bool_t frameHasPaVE (uint8_t *framePointer)
{
  parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)framePointer;
  if (NULL != PaVE &&
      PaVE->signature[0] == 'P' &&
      PaVE->signature[1] == 'a' &&
      PaVE->signature[2] == 'V' &&
      PaVE->signature[3] == 'E')
    {
      return TRUE;
    }
  return FALSE;
}

#if __VIDEO_TCP_DEBUG_ENABLED
static void printBuffer(uint8_t * buf,int size)
{
  const int columns=100;
  int i;
  char c;
  if ( (!buf) || (size<0)) { return; }
  printf("\n"); for (i=0;i<columns;i++) printf("="); printf("\n");
  for (i=0;i<size;i++)
    {
      c=buf[i];
      switch(c)
        {
        case 'P':
        case 'a':  /* ifrm */
        case 'V':
        case 'E':	putchar(c);	break;
#if 0
        case 0  :   putchar('0');	break;  /* nal start */
        case 1  :   putchar('1');	break;  /* nal start */
        case 0x67 :   putchar('S');	break;  /* sps */
        case 0x68 :   putchar('P');	break;  /* pps */
        case 0x65 :   putchar('p');	break;  /* pfrm */
#endif
        default  : putchar('.');
        }

      if ((i%columns)==(columns-1)) { printf("\n"); }
    }
  printf("\n"); for (i=0;i<columns;i++) printf("="); printf("\n");
}
#endif

C_RESULT video_stage_tcp_open(video_stage_tcp_config_t *cfg)
{
  cfg->globalBuffer = vp_os_malloc (BUFFER_MAX_SIZE * sizeof (int8_t));
  if (NULL == cfg->globalBuffer)
    {
      printf ("Unable to allocate TCP Buffer for frame reconstitution\n");
      return C_FAIL;
    }
    
  cfg->frameBuffer = vp_os_malloc (BUFFER_MAX_SIZE * sizeof (int8_t)); //(2 * cfg->frameMeanSize * sizeof (int8_t));
  if (NULL == cfg->frameBuffer)
    {
      printf ("Unable to allocate output frame buffer\n");
      return C_FAIL;
    }
    
  cfg->bufferPointer = vp_os_malloc (sizeof (int8_t *));
  if (NULL == cfg->bufferPointer)
    {
      printf ("Unable to allocate output buffer pointer\n");
      return C_FAIL;
    }
  cfg->currentSize = 0;
  return C_OK;
}

C_RESULT video_stage_tcp_transform(video_stage_tcp_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  uint8_t * buf = NULL;
  bool_t directForward = FALSE;


#if __VIDEO_TCP_DEBUG_ENABLED
  printBuffer(in->buffers[in->indexBuffer],in->size);
#endif

  if(out->status == VP_API_STATUS_INIT) // Init only code
    {
      out->numBuffers   = 1;
      out->buffers      = cfg->bufferPointer;
      out->buffers[0]   = cfg->frameBuffer;
      out->indexBuffer  = 0;
      out->lineSize     = 0;
      out->status = VP_API_STATUS_PROCESSING;
    }

  out->size = 0;

  /* Should not happen, but ... */
  if(in->size < -1 )
    {
      printf ("Bad size ...\n");
      VP_OS_ASSERT(0==1);
      return C_OK;
    }

  if (out->status == VP_API_STATUS_PROCESSING)
    {
      cfg->tcpStageHasMoreData = FALSE;
    }

  if (in->size > 0 && out->status == VP_API_STATUS_PROCESSING)
    {
      /* Directly forward the packet if it exactly contains a whole frame
       * This happens when this stage is used with a UDP stream and one frame per packet.
       */

      if (cfg->currentSize == 0)
        {
          buf = (uint8_t*)(in->buffers[in->indexBuffer]);

          if ( frameHasPaVE(buf)  )
            {
              parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)(in->buffers[in->indexBuffer]);
              if ( in->size == (PaVE->header_size + PaVE->payload_size ))
                {
                  directForward = TRUE;
                  VIDEO_TCP_DEBUG("No luck UDP ! ");
                }
            }
          /* true if a UVLC or P264 header is present */
          else if ( ((*(uint32_t*)buf) & 0xFFFe7c00 )== 0 )
            {
              /* Checks if the first encountered packet looks like a UVLC/P.264 packet */
              directForward = TRUE;
            }
        }

      if (TRUE == directForward)
        {
          out->numBuffers   = in->numBuffers;
          out->buffers      = in->buffers;
          out->indexBuffer  = in->indexBuffer;
          out->lineSize     = in->lineSize;
          out->status       = VP_API_STATUS_PROCESSING;
          out->size         = in->size;
          return C_OK;
        }



      if (in->size + cfg->currentSize >= BUFFER_MAX_SIZE)
        {
          printf ("Got a too big buffer for mine : got %d, had %d, max %d\n", in->size, cfg->currentSize, BUFFER_MAX_SIZE);
          cfg->currentSize = 0;
          return C_OK;
        }

      // Copy input buffer
      vp_os_memcpy(&(cfg->globalBuffer[cfg->currentSize]), in->buffers[in->indexBuffer], in->size);
      cfg->currentSize += in->size;
    }


  if (out->status == VP_API_STATUS_PROCESSING ||
      out->status == VP_API_STATUS_STILL_RUNNING ||
      (cfg->tcpStageHasMoreData==TRUE)
      )
    {
      int lastIFrameStart = -1;  /* Index of the last complete I-frame inside the 'globalBuffer' */

      // Align buffer to first complete PaVE
      int index = 0;
      int maxIndex = cfg->currentSize - sizeof (parrot_video_encapsulation_t);
      // Find first
      for (index = 0; index <= maxIndex; index ++)
        {
          if (cfg->globalBuffer[index]   == 'P' &&
              cfg->globalBuffer[index+1] == 'a' &&
              cfg->globalBuffer[index+2] == 'V' &&
              cfg->globalBuffer[index+3] == 'E')
            {
              break;
            }
        }

      if ( (maxIndex+1) == index || 0 > maxIndex )
        {
          /* Keep data in the buffer in case a piece of PaVE is at the end */
          VIDEO_TCP_DEBUG ("Not enough picture data (PaVE not present or incomplete) ...");
          return C_OK; // No picture, let out->size to zero
        }

      // If needed, dump all datas before this PaVE (so we're sure that we have a PaVE at index 0)
      if (index != 0)
        {
          VIDEO_TCP_DEBUG ("First PaVE was not a index 0 (was at index %d)", index);
          maxIndex -= index;
          cfg->currentSize -= index;
          memmove (cfg->globalBuffer, &(cfg->globalBuffer[index]), cfg->currentSize);
        }

      if (1 == cfg->latencyDrop)
        {
          // Iterate through all frames to find last available I Frame
          int fIndex = 0;
          
#if __VIDEO_TCP_DEBUG_ENABLED
          int fDropped = 0;
          int fTested = 0;
          static int totalDrop = 0;
#endif
          while (fIndex < maxIndex)
            {
              // Check that the frame has a correct pave
              if (FALSE == frameHasPaVE(&cfg->globalBuffer[fIndex]))
                {
                  /* Should not happen, but ... */
#if __VIDEO_TCP_DEBUG_ENABLED
                  printf ("No pave found at index %d (current size = %d) on next frame ...\n",fIndex,cfg->currentSize);
                  printf("Should not happen. Happy debugging !!!\n");
                  video_stage_tcp_dumpPave ((parrot_video_encapsulation_t*)&cfg->globalBuffer[fIndex]);
                  VP_OS_ASSERT(0==1);
#endif
                  break;
                }

              // Check if the frame is complete
              parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)&cfg->globalBuffer[fIndex];
#if __VIDEO_TCP_DEBUG_ENABLED
              printf(" -- PaVE %d at index %d of accumulation buffer ---\n",fTested,fIndex);
              video_stage_tcp_dumpPave (PaVE);
#endif
              int packetSize = PaVE->header_size + PaVE->payload_size;
              if ((cfg->currentSize-fIndex) >= packetSize && frameIsIFrame(&cfg->globalBuffer[fIndex]))
                {
                  VIDEO_TCP_DEBUG ("Jumping to I-frame\n");
                  lastIFrameStart = fIndex;
#if __VIDEO_TCP_DEBUG_ENABLED
                  fDropped = fTested;
#endif
                }
              fIndex += packetSize;
#if __VIDEO_TCP_DEBUG_ENABLED
              fTested++;
#endif
            }
          
          if (0 < lastIFrameStart)
            {
#if __VIDEO_TCP_DEBUG_ENABLED
              totalDrop += fDropped;
              VIDEO_TCP_DEBUG ("I-frame found - dumping %3d frames --> total : %5d\n", fDropped, totalDrop);
#endif
              // Dump all frames before last I frame
              cfg->currentSize -= lastIFrameStart;
              memmove (cfg->globalBuffer, &(cfg->globalBuffer[lastIFrameStart]), cfg->currentSize);
            }
        }

      // Copy frame to decoder if complete
      bool_t frameIsComplete = FALSE;
      parrot_video_encapsulation_t *framePaVE = (parrot_video_encapsulation_t *)cfg->globalBuffer;

      int framePacketSize = framePaVE->header_size + framePaVE->payload_size;

      if (cfg->currentSize >= framePacketSize)
        {
          frameIsComplete = TRUE;
          /* Copy the frame from the accumulation buffer to the output buffer */
          vp_os_memcpy (cfg->frameBuffer, cfg->globalBuffer, framePacketSize);
          cfg->currentSize -= framePacketSize;
          /* Delete the sent frame from the accumulation buffer */
          memmove (cfg->globalBuffer, &(cfg->globalBuffer[framePacketSize]), cfg->currentSize);
          out->size = framePacketSize;
        }


      /* Fill the output structure */
      if (frameIsComplete)
        {
          out->numBuffers   = 1;
          out->buffers      = cfg->bufferPointer;
          out->buffers[0]   = cfg->frameBuffer;
          out->indexBuffer  = 0;
          out->lineSize     = 0;
          out->status       = VP_API_STATUS_PROCESSING;

          /* Check the next available frame .. */
          parrot_video_encapsulation_t *framePaVE = (parrot_video_encapsulation_t *)cfg->globalBuffer;
          if (frameHasPaVE(cfg->globalBuffer) && cfg->currentSize>=sizeof(*framePaVE))
            {
              framePacketSize = framePaVE->header_size + framePaVE->payload_size;
              if (cfg->currentSize >= framePacketSize)
                {
                  /* Do this to inform the socket stage that more frames are available for decoding.
                   * In this case, the socket should be run in a non-blocking mode.
                   * This way, the next decoded frame is either an old frame already present in
                   * the TCP stage buffer, or a newer I-frame which just arrived through the socket.
                   */
                  out->status = VP_API_STATUS_PROCESSING;
                  cfg->tcpStageHasMoreData = TRUE;
                }
            }
        }

      /*
       * Here, the accumulation buffer must have a PaVE, but might not contain a complete frame.
       */
      if(frameIsComplete && FALSE == frameHasPaVE(out->buffers[out->indexBuffer]))
        {
          /* Don't decode frame if it has no PaVE */
          VIDEO_TCP_DEBUG ("Frame don't have PaVE");
          printf("Should not happen. Happy debugging !!!\n");
          video_stage_tcp_dumpPave ((parrot_video_encapsulation_t*)out->buffers[out->indexBuffer]);
          VP_OS_ASSERT(0==1);
          out->size = 0;
        }
      
    }
    
    
  return C_OK;
}

C_RESULT video_stage_tcp_close(video_stage_tcp_config_t *cfg)
{
  vp_os_free (cfg->bufferPointer);
  cfg->bufferPointer = NULL;
  vp_os_free (cfg->globalBuffer);
  cfg->globalBuffer = NULL;
  vp_os_free (cfg->frameBuffer);
  cfg->frameBuffer = NULL;
  return C_OK;
}
