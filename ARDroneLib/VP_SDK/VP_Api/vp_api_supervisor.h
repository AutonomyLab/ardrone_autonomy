/**
 *  @file     vp_api_supervisor.h
 *  @brief    VP Api. Pipeline supervisor
 */


#ifndef _VP_API_SUPERVISOR_H_
#define _VP_API_SUPERVISOR_H_

/**
 * @addtogroup VP_SDK
 * @{ */

/**
 * @addtogroup VP_Api
 * @{ */

/**
 * @defgroup vp_api_supervisor Api supervisor
 *
 * \section Brief
 * \code
 Supervise pipelines :
 Handle multiple pipelines per application.
 Handle pipeline messages and dispatch them to stages.
 * \endcode
 *
 * \section History
 *
 * \par date: 2007-06-14  author: <julien.floret.ext\@parrot.com>
 *  - Handle different message dispatchs : Pipeline level, stage broadcast
 *
 * \par date: 2007-05-28  author: <julien.floret.ext\@parrot.com>
 *  - Add supervisor
 *
 * @{ */

///////////////////////////////////////////////
// INCLUDES

#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>

///////////////////////////////////////////////
// GLOBALS

struct _vp_api_io_pipeline_;


///////////////////////////////////////////////
// DEFINES

/**
 *  @def VP_API_DEST_PIPELINE_LEVEL
 *  Pipeline level : no dispatch to stages
 */
/**
 *  @def VP_API_DEST_STAGE_BROADCAST
 *  Broadcast message to all stages
 */

#define VP_API_DEST_PIPELINE_LEVEL            0x7fff
#define VP_API_DEST_STAGE_BROADCAST           0x7ffe


typedef int32_t PIPELINE_ADDRESS;   ///< Pipeline address
typedef int16_t PIPELINE_HANDLE;    ///< Pipeline handle


/** Pipeline message identifiers
 */
typedef enum _PIPELINE_MSG
{
  PIPELINE_MSG_START,
  PIPELINE_MSG_STOP,
  PIPELINE_MSG_SUSPEND,
  PIPELINE_MSG_RESUME,
  PIPELINE_MSG_RESET,
  PIPELINE_MSG_END,
  PIPELINE_MSG_SYNCHRONIZE,
  PIPELINE_MSG_COMMAND
}
PIPELINE_MSG;


/**
 *  @brief   Pipeline messages fifo
 */
typedef struct _vp_api_fifo_
{
  char   *pbase;             ///< Base address of the fifo
  char   *pget;              ///< Pointer to the next message to get
  char   *ppost;             ///< Where to post a new message in the fifo
  int32_t    nb_waiting;        ///< Number of messages waiting to be handled
  vp_os_mutex_t mutex;
}
vp_api_fifo_t;


/**
 *  @brief   Handle to a message destination
 */
typedef union _DEST_HANDLE
{
  int32_t   handle;             ///< Pipeline handle (16 bits) + stage number (16bits)
  struct
  {
    PIPELINE_HANDLE pipeline;   ///< Pipeline handle
    int16_t         stage;      ///< Stage number
  };
}
DEST_HANDLE;


typedef C_RESULT (*vp_api_handle_msg_t)(  struct _vp_api_io_pipeline_ *pipeline,
                                          PIPELINE_MSG                 msg_id,
                                          void                        *callback,
                                          void                        *param
                                       );


///////////////////////////////////////////////
// PROTOTYPES

/**
 *  @fn      vp_api_add_pipeline(struct _vp_api_io_pipeline_ *, PIPELINE_HANDLE *)
 *  @brief   Add a pipeline and get its handle
 *  @param   pipeline Pipeline to add
 *  @param   handle   Handle to the added pipeline
 *  @return  C_RESULT : VP_SUCCESS
 *  @author  Julien Floret <julien.floret.ext\@parrot.com>
 *  @date    28/05/2007
 */
C_RESULT vp_api_add_pipeline(struct _vp_api_io_pipeline_ *pipeline, PIPELINE_HANDLE *handle);


/**
 *  @fn      vp_api_remove_pipeline(struct _vp_api_io_pipeline_ *, PIPELINE_HANDLE *)
 *  @brief   Remove a pipeline and release handle
 *  @param   pipeline Pipeline to remove
 *  @param   handle   Handle of the pipeline
 *  @return  C_RESULT : VP_SUCCESS
 *  @author  Aurelien Morelle <aurelien.morelle@parrot.com>
 *  @date    18/09/2008
 */
C_RESULT vp_api_remove_pipeline(struct _vp_api_io_pipeline_ *pipeline, PIPELINE_HANDLE *handle);


/**
 *  @fn      vp_api_get_pipeline(PIPELINE_HANDLE)
 *  @brief   Get a pipeline address from its handle
 *  @param   handle Pipeline handle
 *  @return  struct _vp_api_io_pipeline_ * : Pipeline to get
 *  @author  Julien Floret <julien.floret.ext\@parrot.com>
 *  @date    28/05/2007
 */
struct _vp_api_io_pipeline_ * vp_api_get_pipeline(PIPELINE_HANDLE handle);


/**
 *  @fn      vp_api_post_message(DEST_HANDLE, PIPELINE_MSG, void *, void *)
 *  @brief   Post a message to a pipeline
 *  @param   dest     Message destination
 *  @param   msg_id   Message identifier
 *  @param   callback Optional callback function called after processing of the message
 *  @param   param    Optional message parameters
 *  @return  C_RESULT : VP_SUCCESS or VP_FAILURE
 *  @author  Julien Floret <julien.floret.ext\@parrot.com>
 *  @date    28/05/2007
 */
C_RESULT vp_api_post_message(DEST_HANDLE dest, PIPELINE_MSG msg_id, void *callback, void *param);


/**
 *  @fn      vp_api_handle_messages(struct _vp_api_io_pipeline_ *)
 *  @brief   Handle pipeline messages
 *
 *  This function handles pipeline messages by calling user defined callback functions
 *  @param   pipeline   Current pipeline
 *  @return  C_RESULT : VP_SUCCESS
 *  @author  Julien Floret <julien.floret.ext\@parrot.com>
 *  @date    28/05/2007
 */
C_RESULT vp_api_handle_messages(struct _vp_api_io_pipeline_ *pipeline);

// vp_api_supervisor
/** @} */
// VP_Api
/** @} */
// VP_SDK
/** @} */

#endif // ! _VP_API_SUPERVISOR_H_
