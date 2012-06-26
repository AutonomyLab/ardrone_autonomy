/**
 *  @file     vp_api_supervisor.c
 *  @brief    VP Api. Pipeline supervisor
 */


///////////////////////////////////////////////
// INCLUDES

#include <VP_Api/vp_api_supervisor.h>
#include <VP_Api/vp_api_config.h>
#include <VP_Api/vp_api.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>


///////////////////////////////////////////////
// STATICS

/** Global array containing pipeline handles.
 */
static PIPELINE_ADDRESS pipelines[VP_API_MAX_NUM_PIPELINES] = {(PIPELINE_ADDRESS)NULL};


/**
 *  @fn      vp_api_get_message(vp_api_io_pipeline_t *, DEST_HANDLE *, PIPELINE_MSG *, void *, void *)
 *  @brief   Get a message.
 *
 *  Get a message from the pipeline fifo. Internally called by vp_api_handle_messages()
 *  @param   pipeline   Pipeline definition
 *  @param   dest       Message destination
 *  @param   msg_id     Message identifier
 *  @param   callback   Optional callback function called after processing of the message
 *  @param   param      Optional message parameters
 *  @return  C_RESULT : VP_SUCCESS
 *  @author  Julien Floret <julien.floret.ext\@parrot.com>
 *  @date    28/05/2007
 */
static C_RESULT
vp_api_get_message(vp_api_io_pipeline_t *pipeline, DEST_HANDLE *dest, PIPELINE_MSG *msg_id, void **callback, void **param);


///////////////////////////////////////////////
// CODE

static PIPELINE_HANDLE nb_pipelines = 0;


C_RESULT vp_api_add_pipeline(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle)
{
  C_RESULT res = VP_SUCCESS;
  int i = 0;

  VP_OS_ASSERT(nb_pipelines < VP_API_MAX_NUM_PIPELINES);

  while(pipelines[i] != ((PIPELINE_ADDRESS)NULL))
    {
      i++;
    }

  pipelines[i] = (PIPELINE_ADDRESS) pipeline;
  (*handle) = i;
  nb_pipelines ++;

  return res;
}


C_RESULT vp_api_remove_pipeline(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle)
{
  C_RESULT res = VP_SUCCESS;

  pipelines[*handle] = (PIPELINE_ADDRESS) NULL;
  nb_pipelines --;

  return res;
}


vp_api_io_pipeline_t * vp_api_get_pipeline(PIPELINE_HANDLE handle)
{
  vp_api_io_pipeline_t *pipeline = (vp_api_io_pipeline_t *) pipelines[handle];
  return (pipeline);
}


C_RESULT vp_api_post_message(DEST_HANDLE dest, PIPELINE_MSG msg_id, void *callback, void *param)
{
  C_RESULT res = VP_SUCCESS;

  vp_api_io_pipeline_t *pipeline = (vp_api_io_pipeline_t *) pipelines[dest.pipeline];

  /* Do not send the message if the pipeline does not exist yet
	This happens when calling the callback function of the 'video_channel' configuration
    value at drone startup. */
  if (pipeline==NULL) { return VP_FAILURE; }

  VP_OS_ASSERT(pipeline->fifo.nb_waiting >= 0);

  vp_os_mutex_lock(&pipeline->fifo.mutex);

  if((pipeline->fifo.ppost + sizeof(DEST_HANDLE) + sizeof(PIPELINE_MSG) + sizeof(void *) + sizeof(void *)) >= (pipeline->fifo.pbase + VP_API_PIPELINE_FIFO_SIZE))
    pipeline->fifo.ppost = pipeline->fifo.pbase;

  vp_os_memcpy(pipeline->fifo.ppost, &dest, sizeof(DEST_HANDLE));
  pipeline->fifo.ppost += sizeof(DEST_HANDLE);

  vp_os_memcpy(pipeline->fifo.ppost, &msg_id, sizeof(PIPELINE_MSG));
  pipeline->fifo.ppost += sizeof(PIPELINE_MSG);

  if(callback != NULL)
    vp_os_memcpy(pipeline->fifo.ppost, &callback, sizeof(void *));
  else
    vp_os_memset(pipeline->fifo.ppost, 0, sizeof(void *));
  pipeline->fifo.ppost += sizeof(void *);

  if(param != NULL)
    vp_os_memcpy(pipeline->fifo.ppost, &param, sizeof(void *));

  else
    vp_os_memset(pipeline->fifo.ppost, 0, sizeof(void *));
  pipeline->fifo.ppost += sizeof(void *);

  pipeline->fifo.nb_waiting ++;

  vp_os_mutex_unlock(&pipeline->fifo.mutex);

  return res;
}


static C_RESULT vp_api_get_message(vp_api_io_pipeline_t *pipeline, DEST_HANDLE *dest, PIPELINE_MSG *msg_id, void **callback, void **param)
{
  C_RESULT res = VP_SUCCESS;

  VP_OS_ASSERT(pipeline->fifo.nb_waiting > 0);

  vp_os_mutex_lock(&pipeline->fifo.mutex);

  if((pipeline->fifo.pget + sizeof(DEST_HANDLE) + sizeof(PIPELINE_MSG) + sizeof(void *) + sizeof(void *)) >= (pipeline->fifo.pbase + VP_API_PIPELINE_FIFO_SIZE))
    pipeline->fifo.pget = pipeline->fifo.pbase;

  if(dest != NULL)
    vp_os_memcpy(dest, pipeline->fifo.pget, sizeof(DEST_HANDLE));
  pipeline->fifo.pget += sizeof(DEST_HANDLE);

  if(msg_id != NULL)
    vp_os_memcpy(msg_id, pipeline->fifo.pget, sizeof(PIPELINE_MSG));
  pipeline->fifo.pget += sizeof(PIPELINE_MSG);

  if(callback != NULL)
    vp_os_memcpy(callback, pipeline->fifo.pget, sizeof(void *));
  pipeline->fifo.pget += sizeof(void *);

  if(param != NULL)
    vp_os_memcpy(param, pipeline->fifo.pget, sizeof(void *));
  pipeline->fifo.pget += sizeof(void *);

  pipeline->fifo.nb_waiting --;

  vp_os_mutex_unlock(&pipeline->fifo.mutex);

  return res;
}


C_RESULT vp_api_handle_messages(vp_api_io_pipeline_t *pipeline)
{
  C_RESULT res = VP_SUCCESS;

  DEST_HANDLE dest;
  PIPELINE_MSG msg_id;
  void *callback = NULL;
  void *param = NULL;
  uint32_t i;

  while(pipeline->fifo.nb_waiting > 0)
  {
    if(VP_FAILED(vp_api_get_message(pipeline, &dest, &msg_id, &callback, &param)))
      res = VP_FAILURE;
    else
    {
      if(dest.stage == VP_API_DEST_PIPELINE_LEVEL)
      {
    	 if (pipeline->handle_msg){
           pipeline->handle_msg(pipeline, msg_id, callback, param);
    	 }
      }
      else if(dest.stage == VP_API_DEST_STAGE_BROADCAST)
      {
        for(i=0; i < pipeline->nb_stages; i++)
        {
          pipeline->stages[i].funcs.handle_msg(pipeline->stages[i].cfg, msg_id, callback, param);
        }
      }
      else if((dest.stage >=0) && (dest.stage < (int16_t)pipeline->nb_stages))
      {
    	if (pipeline->stages[dest.stage].funcs.handle_msg != NULL)
          pipeline->stages[dest.stage].funcs.handle_msg(pipeline->stages[dest.stage].cfg, msg_id, callback, param);
      }
    }
  }
  return res;
}
