/**
 *  @file     vp_api.c
 *  @brief    VP Api. Pipeline definition
 *  @author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  @author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  @author   Thomas Landais <thomas.landais@parrot.fr>
 *  @author   Julien Floret <julien.floret.ext@parrot.com>
 *  @version  2.0
 *  @date     first release 16/03/2007
 *  @date     modification  24/05/2007
 */

///////////////////////////////////////////////
// INCLUDES

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_supervisor.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>
#ifdef USE_ELINUX
#include <VP_Os/elinux/vp_os_ltt.h>
#endif

const vp_api_resolution_wh_t VP_API_RESOLUTION_WH [VP_API_RES_UB]={
		/* VP_API_RES_LB    */ {0,0},
		/* VP_API_RES_SQCIF */ {128,96},
		/* VP_API_RES_QCIF */ {176,144},
		/* VP_API_RES_QVGA  */ {320,240},
		/* VP_API_RES_CIF   */ {352,288},
		/* VP_API_RES_VGA   */ {640,480},
		/* VP_API_RES_QQCIF */ {88,72},
		/* VP_API_RES_TWEAKY_QQVGA */ {160,120},
		/* VP_API_RES_hdtv360P */  {640,360},
 		/* VP_API_RES_hdtv720P */  {1280,720},
 		/* VP_API_RES_hdtv1080P */ {1920,1080}
};
///////////////////////////////////////////////
// STATICS

static C_RESULT
vp_api_iteration(vp_api_io_pipeline_t *pipeline, vp_api_io_stage_t* previousStage, vp_api_io_stage_t* stage);


///////////////////////////////////////////////
// CODE

C_RESULT
vp_api_open(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle)
{
  C_RESULT res;

  vp_api_io_stage_t *stage;
  uint32_t i;

  res = C_OK;

  VP_OS_ASSERT(pipeline);
  VP_OS_ASSERT(pipeline->nb_stages > 0 && pipeline->nb_stages <= VP_API_MAX_NUM_STAGES);
  VP_OS_ASSERT(pipeline->stages);

  pipeline->nb_still_running = 0;

  for(i = 0 ; i < pipeline->nb_stages && VP_SUCCEEDED(res); i++)
  {
    stage = &pipeline->stages[i];

    VP_OS_ASSERT(stage->funcs.open);
    VP_OS_ASSERT(stage->funcs.transform);
    VP_OS_ASSERT(stage->funcs.close);

	vp_os_mutex_init(&stage->data.lock);
	vp_os_mutex_lock(&stage->data.lock);

	res = stage->funcs.open(stage->cfg);

	if( VP_SUCCEEDED(res) )

    {
      // Set all data fields to 0
        stage->data.numBuffers=0;
        stage->data.buffers=NULL;
        stage->data.indexBuffer=0;

        stage->data.size=0;
        stage->data.lineSize=0;

        stage->data.status = VP_API_STATUS_INIT;
        /* stage->data.lock = Be_careful_not_to_erase_it;  */

        //vp_os_memset(&stage->data, 0, sizeof(vp_api_io_data_t));

    }
		else
		{
			// To avoid problems on vp_api_close
			pipeline->nb_stages=i+1;
		}
  }

  pipeline->fifo.pbase   = (char *) vp_os_malloc(VP_API_PIPELINE_FIFO_SIZE);
  pipeline->fifo.pget    = pipeline->fifo.pbase;
  pipeline->fifo.ppost   = pipeline->fifo.pbase;
  pipeline->fifo.nb_waiting = 0;
  vp_os_memset(pipeline->fifo.pbase, 0, VP_API_PIPELINE_FIFO_SIZE);
  vp_os_mutex_init(&pipeline->fifo.mutex);

  if( VP_SUCCEEDED(res) )
  {
    res = vp_api_add_pipeline(pipeline, handle);
  }

  return res;
}


C_RESULT
vp_api_run(vp_api_io_pipeline_t *pipeline, vp_api_io_data_t *out_data)
{
  vp_api_io_stage_t* previousStage = NULL;
  vp_api_io_stage_t* currentStage = NULL;
  C_RESULT res = VP_SUCCESS;
  uint32_t i=0;

  if(pipeline->fifo.nb_waiting > 0)
    if(VP_FAILED(vp_api_handle_messages(pipeline)))
      res = VP_FAILURE;

  if(pipeline->nb_still_running != 0)
  {
    for(i = pipeline->nb_stages-1 ; pipeline->stages[i].data.status != VP_API_STATUS_STILL_RUNNING ; i--)
    {
      VP_OS_ASSERT(i >= 0);
    }
    pipeline->nb_still_running--;
    if(i > 0)
      currentStage = &pipeline->stages[i-1];
  }

  for(; i < pipeline->nb_stages ; i++)
  {
    previousStage = currentStage;
    currentStage = &pipeline->stages[i];

    RTMON_UVAL(SDK_STAGE_INDEX_UVAL, i);
    if(VP_SUCCEEDED(res) && VP_FAILED(vp_api_iteration(pipeline, previousStage, currentStage)))
    {
      res = VP_FAILURE;
    }

    //do not execute next stages if no data is given
    if(pipeline->stages[i].data.size == 0)
    {
      break;
    }
  }

  if(currentStage!=NULL)
	  *out_data = currentStage->data;
  else
	  res = VP_FAILURE;

  return res;
}

C_RESULT
vp_api_flush(vp_api_io_pipeline_t *pipeline)
{
  vp_api_io_data_t data;
  C_RESULT res = VP_SUCCESS;

  while(pipeline->nb_still_running != 0)
    if(VP_FAILED(vp_api_run(pipeline, &data)))
      res = VP_FAILURE;

  return res;
}

C_RESULT
vp_api_close(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle)
{
  vp_api_io_stage_t *stage;
  C_RESULT res = VP_SUCCESS;
  uint32_t i;

  VP_OS_ASSERT(pipeline->nb_stages > 0 && pipeline->nb_stages <= VP_API_MAX_NUM_STAGES);

  res = vp_api_remove_pipeline(pipeline, handle);

#ifdef DEBUG_MODE
  if( pipeline->fifo.pbase != NULL )
#endif // DEBUG_MODE
    vp_os_free(pipeline->fifo.pbase);

  for(i = 0 ; i < pipeline->nb_stages ; i++)
  {
    stage = &pipeline->stages[i];
    if(VP_FAILED(stage->funcs.close(stage->cfg)))
      res = VP_FAILURE;

    vp_os_mutex_unlock(&stage->data.lock);
    vp_os_mutex_destroy(&stage->data.lock);
  }

  return res;
}

static C_RESULT
vp_api_iteration(vp_api_io_pipeline_t *pipeline, vp_api_io_stage_t* previousStage, vp_api_io_stage_t* stage)
{
  C_RESULT res = VP_SUCCESS;
  vp_api_io_data_t *previousData = NULL;

  if(previousStage)
  {
    previousData = &previousStage->data;
  }

  vp_os_mutex_unlock(&stage->data.lock);
  RTMON_USTART(SDK_STAGE_TRANSFORM_UEVENT);
#ifdef USE_ELINUX
  if(stage->name != NULL)
    LTT_WRITEF("stage %s ->",stage->name);

  if(stage->disabled)
  {
   res = vp_api_stage_empty_transform(stage->cfg, previousData, &stage->data);
  }
  else
#endif
  res = stage->funcs.transform(stage->cfg, previousData, &stage->data);
#ifdef USE_ELINUX
  if(stage->name != NULL)
    LTT_WRITEF("stage %s <-",stage->name);
#endif
  RTMON_USTOP(SDK_STAGE_TRANSFORM_UEVENT);

  if(stage->data.status == VP_API_STATUS_STILL_RUNNING)
  {
    pipeline->nb_still_running++;
  }
  vp_os_mutex_lock(&stage->data.lock);

  return res;
}
