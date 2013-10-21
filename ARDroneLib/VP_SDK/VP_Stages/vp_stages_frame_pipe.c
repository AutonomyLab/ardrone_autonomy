#include <VP_Os/vp_os_malloc.h>
#include <VP_Stages/vp_stages_frame_pipe.h>
#include <VP_Os/vp_os_print.h>

// Sender function
C_RESULT
vp_stages_frame_pipe_sender_open(vp_stages_frame_pipe_config_t *cfg)
{
  if (cfg->inPicture == NULL)
    return C_FAIL;
  else
    return C_OK;
}

C_RESULT
vp_stages_frame_pipe_sender_transform(vp_stages_frame_pipe_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	int i;

  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
	  /* Allocate an array to store pointers to the output buffers */
	  cfg->output_buffers = vp_os_malloc(cfg->nb_buffers*sizeof(*cfg->output_buffers));
	  /* Point to the first buffer */
	  cfg->index_buffer = 0;

    // initialize receiver vp_api_picture

    /* Copy the picture description */
    vp_os_memcpy(&cfg->outPicture,cfg->inPicture,sizeof(cfg->outPicture));

    /* Allocate memory to contain the copy picture */
	vp_api_picture_alloc(&cfg->outPicture,
        cfg->inPicture->width,
        cfg->inPicture->height,
        cfg->inPicture->format );

    /* fast mode uses a double buffer, allocate it if necessary */
    if (cfg->mode == FAST)
    {
		for (i=0;i<cfg->nb_buffers;i++){
			vp_api_picture_alloc(&cfg->copyPicture,
			          cfg->inPicture->width,
			          cfg->inPicture->height,
			          cfg->inPicture->format );
			cfg->output_buffers[i]=cfg->copyPicture.raw;
		}
    }

    cfg->frame_size = vp_api_picture_get_buffer_size(cfg->inPicture);//y_length + 2*cc_length;
    out->status = VP_API_STATUS_PROCESSING;
  }

  if (in->status == VP_API_STATUS_ERROR)
  {
    // previous stage gave an error
    vp_os_mutex_lock(&(cfg->pipe_mut));
    {
      // signal receiver that sender is closed
      cfg->pipe_state = SENDER_ERROR;
      // resume receiver
      vp_os_cond_signal (&(cfg->buffer_sent));
    }
    vp_os_mutex_unlock(&(cfg->pipe_mut));
    PRINT("%s:%d sender error, close stage\n", __FILE__,__LINE__);
  }
  if( out->status == VP_API_STATUS_PROCESSING )
  {
    vp_os_mutex_lock(&(cfg->pipe_mut));
    {
      if (cfg->pipe_state == WAIT_RECEPTION || cfg->pipe_state == FETCH)
      {
        if (cfg->mode == LOW_LATENCY)
        {
          // in low latency mode, copy directly the frame in the destination buffer
          vp_os_memcpy(cfg->outPicture.raw,cfg->inPicture->raw,vp_api_picture_get_buffer_size(cfg->inPicture));
        }
        else
        {
          // in fast mode, copy frame in the copy buffer
          cfg->index_buffer = (cfg->index_buffer+1)%cfg->nb_buffers;
          vp_api_picture_point_to_buf_address(&cfg->copyPicture,cfg->output_buffers[cfg->index_buffer]);
          vp_os_memcpy(cfg->copyPicture.raw,cfg->inPicture->raw,vp_api_picture_get_buffer_size(cfg->inPicture));
        }

        // signal end of copy
        if (cfg->pipe_state == WAIT_RECEPTION)
        {
          cfg->pipe_state = PAUSE;
          vp_os_cond_signal (&(cfg->buffer_sent));
        }
        else
        {
          cfg->pipe_state = FETCH_PAUSE;
        }
      }
    }
    vp_os_mutex_unlock(&(cfg->pipe_mut));
  }
  /* wire in to out */
  vp_os_memcpy (out, in, sizeof(vp_api_io_data_t));

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}

C_RESULT
vp_stages_frame_pipe_sender_close(vp_stages_frame_pipe_config_t *cfg)
{
	  return C_OK;
}

// Receiver function
C_RESULT
vp_stages_frame_pipe_receiver_open(vp_stages_frame_pipe_config_t *cfg)
{
  	  return C_OK;
}

C_RESULT
vp_stages_frame_pipe_receiver_transform(vp_stages_frame_pipe_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers = 1;
    out->indexBuffer = 0;
    out->status = VP_API_STATUS_PROCESSING;
  }

  if( out->status == VP_API_STATUS_PROCESSING )
  {
    vp_os_mutex_lock(&(cfg->pipe_mut));
    {
      if (cfg->pipe_state != SENDER_ERROR)
    {
      if (cfg->pipe_state == FETCH_PAUSE)
      {
        // a frame has alerady been fetched by a fetch stage
        cfg->pipe_state = PAUSE;
      }
      else
      {
        // no fetch stage was executed, ask for a frame transfer
        cfg->pipe_state = WAIT_RECEPTION;
        vp_os_cond_wait (&(cfg->buffer_sent));
      }

        if (cfg->mode == FAST && cfg->pipe_state != SENDER_ERROR)
      {
        // at this point a new frame is available in the copy buffer
        // swap the two output buffer to make new picture available in outPicture
        vp_api_picture_t temp = cfg->outPicture;
        cfg->outPicture = cfg->copyPicture;
        cfg->copyPicture = temp;

        // trigger a new frame transfert
        cfg->pipe_state = FETCH;
      }

      // at this point, we are sure that cfg->outPicture was initialized by the sender
      out->buffers = &(cfg->outPicture.raw);
      out->size = cfg->frame_size;
      }
    }
    vp_os_mutex_unlock(&(cfg->pipe_mut));

    if (cfg->callback != NULL)
      cfg->callback();
  }

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}

C_RESULT
vp_stages_frame_pipe_receiver_close(vp_stages_frame_pipe_config_t *cfg)
{
  vp_os_free (cfg->outPicture.raw);
  cfg->outPicture.raw = NULL;
  vp_os_cond_destroy (&(cfg->buffer_sent));
  vp_os_mutex_destroy (&(cfg->pipe_mut));
  return C_OK;
}

// Fetch functions
// This stage can be optionally used with a frame_pipe_receiver.
// Fetch stage triggers a frame transfer without blocking the pipeline.
// When executing the associated frame_pipe_receiver stage (the one sharing the same cfg) 2 behaviors can occurs :
// - the frame has been transfer thanks to the previous fetch stage, thus the receiver will not block and time will be saved
// - the frame is not already transfered, the stage wait for copy completion
C_RESULT
vp_stages_frame_pipe_fetch_open(vp_stages_frame_pipe_config_t *cfg)
{
  return C_OK;
}

C_RESULT
vp_stages_frame_pipe_fetch_transform(vp_stages_frame_pipe_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->status = VP_API_STATUS_PROCESSING;
  }

  if( out->status == VP_API_STATUS_PROCESSING )
  {
    vp_os_mutex_lock(&(cfg->pipe_mut));
    {
      cfg->pipe_state = FETCH;
    }
    vp_os_mutex_unlock(&(cfg->pipe_mut));
  }

  out->numBuffers = in->numBuffers;
  out->buffers = in->buffers;
  out->indexBuffer = in->indexBuffer;
  out->size = in->size;
  out->lineSize = in->lineSize;
  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}

C_RESULT
vp_stages_frame_pipe_fetch_close(vp_stages_frame_pipe_config_t *cfg)
{
  return C_OK;
}
