#include <VP_Os/vp_os_malloc.h>
#include <VP_Stages/vp_stages_frame_pipe.h>
#ifdef USE_ELINUX
#include "dma_malloc.h"
#define vp_os_malloc(a) dma_malloc(a)
#endif
//#include <VP_Os/elinux/vp_os_ltt.h>

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
	uint32_t y_length;
	uint32_t cc_length;

	  vp_os_mutex_lock(&out->lock);

	  if(out->status == VP_API_STATUS_INIT)
	  {
		 // initialize receiver vp_api_picture
		 cfg->outPicture.format 			= cfg->inPicture->format;
		 cfg->outPicture.width  			= cfg->inPicture->width;
		 cfg->outPicture.height 			= cfg->inPicture->height;
		 cfg->outPicture.framerate 			= cfg->inPicture->framerate;

		 cfg->outPicture.y_pad 				= cfg->inPicture->y_pad;
		 cfg->outPicture.c_pad 				= cfg->inPicture->c_pad;
		 cfg->outPicture.y_line_size 		= cfg->inPicture->y_line_size;
		 cfg->outPicture.cb_line_size 		= cfg->inPicture->cb_line_size;
		 cfg->outPicture.cr_line_size 		= cfg->inPicture->cr_line_size;
		 cfg->outPicture.vision_complete	= cfg->inPicture->vision_complete;
		 cfg->outPicture.complete 			= cfg->inPicture->complete;
		 cfg->outPicture.blockline 			= cfg->inPicture->blockline;


		  y_length = cfg->inPicture->y_line_size * cfg->inPicture->height;
		  cc_length = cfg->inPicture->cb_line_size * cfg->inPicture->height>>1;

		 cfg->frame_size = y_length + 2*cc_length;

		 cfg->outPicture.y_buf = vp_os_malloc (cfg->frame_size * sizeof(int8_t));
		 cfg->outPicture.cb_buf = cfg->outPicture.y_buf + y_length;
		 cfg->outPicture.cr_buf = cfg->outPicture.cb_buf + cc_length;

		 // wire in to out
	     vp_os_memcpy (out, in, sizeof(vp_api_io_data_t));
	     out->status       = VP_API_STATUS_PROCESSING;
	  }

	  if( out->status == VP_API_STATUS_PROCESSING )
	  {
		  vp_os_mutex_lock(&(cfg->pipe_mut));
		  {
			  if (cfg->pipe_state == WAIT_RECEPTION)
			  {
				  // send buffer
				  //LTT_WRITEF ("Pipe start");
				  vp_os_memcpy (cfg->outPicture.y_buf,in->buffers[in->indexBuffer],in->size);
				  //LTT_WRITEF ("Pipe stop");
				  // signal end of copy
				  cfg->pipe_state = PAUSE;
				  vp_os_cond_signal (&(cfg->buffer_sent));
			  }
		  }
		  vp_os_mutex_unlock(&(cfg->pipe_mut));
	  }

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
			  cfg->pipe_state = WAIT_RECEPTION;
			  vp_os_cond_wait (&(cfg->buffer_sent));
			  // at this moment, we are sure that cfg->outPicture was initialized by the sender
			  out->buffers = (int8_t**)&(cfg->outPicture.y_buf);
			  out->size = cfg->frame_size;
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
	vp_os_free (cfg->outPicture.y_buf);
	vp_os_cond_destroy (&(cfg->buffer_sent));
	vp_os_mutex_destroy (&(cfg->pipe_mut));
	return C_OK;
}
