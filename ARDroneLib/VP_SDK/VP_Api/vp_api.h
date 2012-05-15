/**
 *  @file     vp_api.h
 *  @brief    VP Api. Pipeline definition
 *  @author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  @author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  @author   Thomas Landais <thomas.landais@parrot.fr>
 *  @author   Julien Floret <julien.floret.ext@parrot.com>
 *  @version  2.0
 *  @date     first release 16/03/2007
 *  @date     modification  24/05/2007
 */

#ifndef _VP_API_INCLUDE_H_
#define _VP_API_INCLUDE_H_

#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_config.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_supervisor.h>


/**
 * @enum  _VP_API_IO_TYPES_
 * @brief Available io stages type
 */
typedef enum _VP_API_IO_TYPES_
{
  // INPUTS
  VP_API_INPUT_BUFFER,
  VP_API_INPUT_CAMIF,
  VP_API_INPUT_FILE,
  VP_API_INPUT_SOCKET,

  // FILTERS
  VP_API_FILTER_ENCODER,
  VP_API_FILTER_DECODER,
  VP_API_FFMPEG_ENCODER,
  VP_API_FFMPEG_DECODER,
  VP_API_FILTER_VISION,
  VP_API_VISION_PREPARE,
  VP_API_FILTER_YUV2RGB,
  VP_API_JPEG_ENCODER,
  VP_API_JPEG_DECODER,
  VP_API_ARWIZ,

  // OUTPUTS
  VP_API_OUTPUT_CONSOLE,
  VP_API_OUTPUT_SDL,
  VP_API_OUTPUT_LCD,
  VP_API_OUTPUT_FILE,
  VP_API_OUTPUT_SOCKET,

  // MIXERS
  VP_API_PIPE
}
VP_API_IO_TYPE;

/**
 * @enum  _VP_API_IO_STATUS_
 * @brief Stage's state definition
 */
typedef enum _VP_API_IO_STATUS_
{
  VP_API_STATUS_INIT        = 0, // needs being equal to zero to ease initialization
  VP_API_STATUS_PROCESSING,
  VP_API_STATUS_STILL_RUNNING,
  VP_API_STATUS_ENDED,
  VP_API_STATUS_ERROR
}
VP_API_IO_STATUS;


/**
 * @struct _vp_api_io_data_
 * @brief  This structure is used by stages to share data in a generic way
 */
typedef struct _vp_api_io_data_
{
  uint32_t         numBuffers;
  int8_t         **buffers;
  uint32_t         indexBuffer;

  int32_t          size;
  int32_t         *lineSize;

  VP_API_IO_STATUS status;

  vp_os_mutex_t    lock;
}
vp_api_io_data_t;


/**
 * @struct _vp_api_io_stage_
 * @brief  Public definition of a stage
 *
 * This structure is used to configure stages
 * The cfg field can be used to share data between stages in a specific way
 */
typedef struct _vp_api_io_stage_
{
  VP_API_IO_TYPE        type;
  void                 *cfg;
  vp_api_stage_funcs_t  funcs;
  vp_api_io_data_t      data;
}
vp_api_io_stage_t;


/**
 * @struct _vp_api_io_pipeline_
 * @brief  Public definition of the pipeline
 *
 * This structure is used to configure the pipeline
 */
typedef struct _vp_api_io_pipeline_
{
  uint32_t                nb_stages;
  vp_api_io_stage_t      *stages;
  vp_api_handle_msg_t     handle_msg;
  uint32_t                nb_still_running;
  vp_api_fifo_t           fifo;

}
vp_api_io_pipeline_t;

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @fn      C_RESULT vp_api_open(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle)
 * @brief  Creates internally all
 * @param   pipeline  Pipeline definition
 * @param   handle    Pipeline handle
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_api_open(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle);


/**
 * @fn      C_RESULT vp_api_run(vp_api_io_pipeline_t *pipeline, vp_api_io_data_t *out_data)
 * @brief   Runs pipeline
 * @param   pipeline  Pipeline definition
 * @param   out_data  Output data of the last pipeline stage
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_api_run(vp_api_io_pipeline_t *pipeline, vp_api_io_data_t *out_data);


/**
 * @fn      C_RESULT vp_api_flush(vp_api_io_pipeline_t *pipeline)
 * @brief   Flushes pipeline
 * @param   pipeline  Pipeline definition
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_api_flush(vp_api_io_pipeline_t *pipeline);

/**
 * @fn      C_RESULT vp_api_close(vp_api_io_pipeline_t *pipeline)
 * @brief   Clean up
 * @param   pipeline  Pipeline definition
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_api_close(vp_api_io_pipeline_t *pipeline, PIPELINE_HANDLE *handle);

#ifdef __cplusplus
}
#endif

#endif // ! _VP_API_INCLUDE_H_
