/**
 * \file vp_stages_i_camif.h
 * \brief VP_Stages. camif stage declaration
 */

#ifndef _VP_STAGES_I_CAMIF_H_
#define _VP_STAGES_I_CAMIF_H_

/**
 * @defgroup VP_SDK
 * @{ */

/**
 * @defgroup VP_Stages
 * @{ */

/**
 * @defgroup vp_stages_i_camif input camif stage
 * @{ */

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_thread_helper.h>
#define ICAMIF_STACK_SIZE 8192

#ifdef _INCLUDED_FOR_DOXYGEN_
#else // ! _INCLUDED_FOR_DOXYGEN_

#include <VP_Api/vp_api_picture.h>

//! a block is 2^4 = 16 lines
# define CAMIF_BLOCKLINES_LOG2       4
# define CAMIF_BLOCKLINES            (1 << CAMIF_BLOCKLINES_LOG2)

#define CAMIF_MAX_BUFFERS			 8
#define CAMIF_MIN_BUFFERS			 4

typedef enum _CAMIF_RESOLUTION
{
	  CAMIF_RES_LB=0,

	  CAMIF_RES_SQCIF,
	  CAMIF_RES_QCIF,
	  CAMIF_RES_QVGA,
	  CAMIF_RES_CIF,
	  CAMIF_RES_VGA,
	  CAMIF_RES_QQCIF,
	  CAMIF_RES_TWEAKY_QQVGA,
	  CAMIF_RES_hdtv360P,
	  CAMIF_RES_hdtv720P,

	  CAMIF_RES_UB
} CAMIF_RESOLUTION;
#endif // < _INCLUDED_FOR_DOXYGEN_


///////////////////////////////////////////////
// TYPEDEFS

typedef enum _CAPTURE_STATE
{
  RUNNING,
  STOPPED
} CAPTURE_STATE;

// vp_stages_i_camif
/** @} */
// VP_Stages
/** @} */
// VP_SDK
/** @} */

#endif // > _VP_STAGES_I_CAMIF_H_
