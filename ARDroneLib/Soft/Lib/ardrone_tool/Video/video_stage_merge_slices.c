#include "video_stage_merge_slices.h"
#include <video_encapsulation.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>
#include <stdio.h>

const vp_api_stage_funcs_t video_stage_merge_slices_funcs = {
  (vp_api_stage_handle_msg_t) video_stage_merge_slices_handle,
  (vp_api_stage_open_t) video_stage_merge_slices_open,
  (vp_api_stage_transform_t) video_stage_merge_slices_transform,
  (vp_api_stage_close_t) video_stage_merge_slices_close
};

/**
 * IPHONE DEBUG AREA
 */
float DEBUG_nbSlices = 0.0;
float DEBUG_totalSlices = 0.0;
int DEBUG_missed = 0;
int DEBUG_prevNumber = 0;

/* CONFIGURATION */

#define VIDEO_SLICE_DEBUG_ENABLE_ON_GLOBAL_DEBUG (0) ///< Auto enable debug while on debug mode
#define VIDEO_SLICE_DEBUG_ENABLE (0)                 ///< Force enable of debug for this file

/* END OF CONFIGURATION */


#if VIDEO_SLICE_DEBUG_ENABLE || (VIDEO_SLICE_DEBUG_ENABLE_ON_GLOBAL_DEBUG && defined (DEBUG))
#define __VIDEO_SLICE_DEBUG_ENABLED (1)
#else
#define __VIDEO_SLICE_DEBUG_ENABLED (0)
#endif

#if __VIDEO_SLICE_DEBUG_ENABLED
#define VIDEO_SLICE_DEBUG(...)                                          \
  do                                                                    \
    {                                                                   \
      printf ("VIDEO_SLICE-DEBUG (%s @ %d) : ", __FUNCTION__, __LINE__); \
      printf (__VA_ARGS__);                                             \
      printf ("\n");                                                    \
    } while (0)

#else
#define VIDEO_SLICE_DEBUG(...)
#endif



C_RESULT video_stage_merge_slices_handle (video_stage_merge_slices_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param)
{
  return C_OK;
}

C_RESULT video_stage_merge_slices_open(video_stage_merge_slices_config_t *cfg)
{
  int idx;
  
  for (idx=0;idx<2;idx++)
    {
      cfg->bufs[idx].bufferPointer = vp_os_malloc (sizeof (int8_t *));
    
      if (NULL == cfg->bufs[idx].bufferPointer)
        {
          printf ("Unable to allocate output buffer pointer\n");
          return C_FAIL;
        }
    
      cfg->bufs[idx].accumulated_size =0;
      cfg->bufs[idx].buffer = NULL;
      cfg->bufs[idx].buffer_size = 0;
    }
  
  cfg->mergingBuffer = 0;
  cfg->readyBuffer   = 0;
  
  return C_OK;
}

C_RESULT video_stage_merge_slices_reset(video_stage_merge_slices_config_t *cfg)
{
  int idx;
  
  for (idx=0;idx<2;idx++)
    {
      if (cfg->bufs[idx].buffer) { vp_os_sfree((void**)&(cfg->bufs[idx].buffer)); }
      cfg->bufs[idx].accumulated_size =0;
      cfg->bufs[idx].buffer_size = 0;
      cfg->bufs[idx].nb_slices = 0;
    }
  
  cfg->mergingBuffer = 0;
  cfg->readyBuffer   = 0;
  
  return C_OK;
}


C_RESULT video_stage_merge_slices_transform(video_stage_merge_slices_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  bool_t switchBuffers;
  bool_t dataReady = 0;
  video_stage_merge_slices_buffer_t *buf;
  int dataSize;
  
  out->size = 0;

  if(out->status == VP_API_STATUS_INIT)
    {
      /* By default, feed the next stage with our local output */
      out->numBuffers   = 1;
      out->buffers      = cfg->bufs[0].bufferPointer;
      out->buffers[0]   = cfg->bufs[0].buffer;
      out->indexBuffer  = 0;
      out->lineSize     = 0;
      out->status       = VP_API_STATUS_PROCESSING;
    }

  // check input
  parrot_video_encapsulation_t *PaVE         = (parrot_video_encapsulation_t *) in->buffers[in->indexBuffer];
  parrot_video_encapsulation_t *mergingPaVE  = (parrot_video_encapsulation_t *) cfg->bufs[cfg->mergingBuffer].buffer;
  

  VIDEO_SLICE_DEBUG("PAVE frm %d  sl %d/%d  %s\n ",
                    PaVE->frame_number,
                    PaVE->slice_index+1,
                    PaVE->total_slices,
                    (PaVE->frame_type == FRAME_TYPE_I_FRAME || PaVE->frame_type == FRAME_TYPE_IDR_FRAME) ? "I-frm":"p-frm" );

  if ( (PaVE==NULL) || (!PAVE_CHECK(PaVE)) || (PaVE->total_slices == 1) )
    {
      // No slices, just send the data to the next stage

      /* Feed the next stage with the previous stage's output */
      out->buffers     = in->buffers;
      out->indexBuffer = in->indexBuffer;
      out->lineSize    = in->lineSize;
      out->numBuffers  = in->numBuffers;
      out->status      = in->status;
      out->size        = in->size;
      out->status      = VP_API_STATUS_PROCESSING;
    
      /* Get rid of previously accumulated data */
      video_stage_merge_slices_reset(cfg);

      /* IPHONE DEBUG : set slices miss to 0/1 (to avoid keeping old/fake slice datas) */
      DEBUG_totalSlices = 1;
      DEBUG_nbSlices = 0;
    
      return C_OK;
    }
  
  /*
   * Check if the incoming PaVE belongs to the same frame.
   */
  switchBuffers = (mergingPaVE)  /* At program startup this buffer is empty and switching makes no sense */
    && ( !(pave_is_same_frame(PaVE,mergingPaVE)) );
  
  /* If not the same frame, start building a new one */
  if (switchBuffers)
    {

      if (0 != DEBUG_prevNumber)
        {
          int DEBUG_tempMiss = (PaVE->frame_number - (DEBUG_prevNumber + 1));
          if (0 < DEBUG_tempMiss)
            {
              DEBUG_missed += DEBUG_tempMiss;
            }
        }
      DEBUG_prevNumber = PaVE->frame_number;

      cfg->readyBuffer   = cfg->mergingBuffer;
      cfg->mergingBuffer = (cfg->mergingBuffer+1)%2;

      cfg->bufs[cfg->mergingBuffer].accumulated_size = 0;
      cfg->bufs[cfg->mergingBuffer].nb_slices = 0;
    }

  /* Get a pointer to the buffer to store data to */
  buf = &cfg->bufs[cfg->mergingBuffer];

  // Check destination buffer size
  if (buf->buffer_size < (buf->accumulated_size + in->size))
    {
      buf->buffer_size = buf->accumulated_size + in->size;
      buf->buffer = vp_os_realloc(buf->buffer, buf->buffer_size );
      if(!buf->buffer) { return C_FAIL; }
    }

  /* Copy data */
  if (PaVE->slice_index == 0 || buf->accumulated_size==0)
    {
      /* Copy the entire packet */
      if (buf->buffer) { vp_os_memcpy(buf->buffer, PaVE, in->size); }
      buf->accumulated_size = in->size;
      buf->nb_slices++;
    }
  else
    {
#define mymin(x,y) ( ((x)<(y))?(x):(y) )
      /* Copy only the payload */
      dataSize = mymin(PaVE->payload_size,in->size); /* safety if the PaVE is corrupted */

      if (buf->buffer) { vp_os_memcpy(buf->buffer + buf->accumulated_size, in->buffers[in->indexBuffer] + PaVE->header_size, dataSize); }
      buf->accumulated_size += dataSize;
      buf->nb_slices++;
    }

  /* Select the buffer to send to the next stage */
  
  if (PaVE->slice_index == PaVE->total_slices - 1)
    {
      /* The buffer we just used for merging is ready to be sent */
      cfg->readyBuffer   = cfg->mergingBuffer;
      cfg->mergingBuffer = (cfg->mergingBuffer+1)%2;
      cfg->bufs[cfg->mergingBuffer].accumulated_size = 0;
      cfg->bufs[cfg->mergingBuffer].nb_slices = 0;
      dataReady = 1;
    }
  else if (switchBuffers)
    {
      /* Send the previous buffer if it contains something */
      dataReady = 1;
    }

  // If slice was the last one, continue to next stage
  if (dataReady)
    {
      buf = &cfg->bufs[cfg->readyBuffer];

      if (buf->buffer && buf->accumulated_size)
        {
          int totalSlices;
          parrot_video_encapsulation_t *newPaVE = (parrot_video_encapsulation_t *) buf->buffer;

          /* Save the old value of total slices */
          totalSlices = newPaVE->total_slices;

          newPaVE->payload_size = buf->accumulated_size - PaVE->header_size;
          newPaVE->slice_index  = 0;
          newPaVE->total_slices = 1;

          /* Feed the next stage with our local output */
          out->size         = buf->accumulated_size;
          out->buffers      = buf->bufferPointer;
          out->buffers[0]   = buf->buffer;
          out->indexBuffer  = 0;
          out->numBuffers   = 1;
          out->lineSize     = 0;
          out->status       = VP_API_STATUS_PROCESSING;

          if(!PAVE_CHECK(buf->buffer)) { printf("%s:%d - No PaVE !\n",__FUNCTION__,__LINE__);  assert(0==1); }

          if (buf->nb_slices != totalSlices) { printf("Missing slices (%d)\n",totalSlices-buf->nb_slices); }
          
          DEBUG_nbSlices = 0.9*DEBUG_nbSlices + 0.1*(totalSlices-buf->nb_slices);
          DEBUG_totalSlices = 0.9*DEBUG_totalSlices + 0.1*totalSlices;
        }
    }
  else
    {
      out->size = 0;
    }

  if (out->size)
    {
      PaVE = (parrot_video_encapsulation_t*) out->buffers[0];
      VIDEO_SLICE_DEBUG("Switch %d - Sending - PAVE frm %d  sl %d/%d\n ",
                        switchBuffers,
                        PaVE->frame_number,
                        PaVE->slice_index+1,
                        PaVE->total_slices );}

  return C_OK;
}

C_RESULT video_stage_merge_slices_close(video_stage_merge_slices_config_t *cfg)
{
  int idx;
  video_stage_merge_slices_buffer_t *buf;
    
  for (idx=0;idx<2;idx++)
    {
      buf = &cfg->bufs[idx];
      if (buf->buffer)        { vp_os_sfree((void**)&buf->buffer);        }
      if (buf->bufferPointer) { vp_os_sfree((void**)&buf->bufferPointer); }
    }
  return C_OK;
}

