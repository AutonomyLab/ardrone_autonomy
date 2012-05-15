/**
 *  @file   vp_stages_yuv2rgb.h
 *  @brief  VP Stages. YUV to RGB converter stage declaration
 */

///////////////////////////////////////////////
// INCLUDES

#ifndef _VP_STAGES_YUV2RGB_INCLUDE_
#define _VP_STAGES_YUV2RGB_INCLUDE_

/** 
 * @addtogroup VP_Stages
 * @{ */

/** 
 * @defgroup vp_stages_yuv2rgb YUV-4:2:0p to RGB converter stage
 *
 * \section Brief
 * \code
 Gets in input a frame which type is vp_api_picture_t and format is planar YUV-4:2:0.
 Converts it into RGB format : RGB-565 (16 bits), RGB24 or Alpha-RGB (32 bits)
 Stores the result frame into the output buffer.
 * \endcode
 *
 * \section History
 *
 * @{ */

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>

///////////////////////////////////////////////
// TYPEDEFS

/** Select RGB format
 */
typedef enum _VP_STAGES_RGB_FORMAT
{
  VP_STAGES_RGB_FORMAT_MARKER_BEGIN = -1,
  VP_STAGES_RGB_FORMAT_RGB565       = 0,
  VP_STAGES_RGB_FORMAT_RGB24,
  VP_STAGES_RGB_FORMAT_ARGB32,
  VP_STAGES_RGB_FORMAT_MARKER_END
} VP_STAGES_RGB_FORMAT;

typedef enum _VP_STAGES_YUV2RGB_CONVERSION_MODE
{
  VP_STAGES_YUV2RGB_MODE_NORMAL = 0,
  VP_STAGES_YUV2RGB_MODE_UPSIDE_DOWN,
} VP_STAGES_YUV2RGB_CONVERSION_MODE;


/**
 *  @brief   Configuration structure for the YUV-4:2:0p to RGB-Alpha converter stage
 */
typedef struct _vp_stages_yuv2rgb_config_
{
  VP_STAGES_RGB_FORMAT rgb_format; //!< Select RGB format

#ifdef USE_YUV2RGB_STRETCH
  uint32_t*  hline0;
  uint32_t*  hline1;
  uint32_t*  vline;
#endif  // < USE_YUV2RGB_STRETCH

  VP_STAGES_YUV2RGB_CONVERSION_MODE mode;

} vp_stages_yuv2rgb_config_t;


/**
 *  @var     vp_stages_YUV_to_RGB_t
 *  @brief   Define pointer type to a YUV to RGB conversion function
 */
typedef void (*vp_stages_YUV_to_RGB_t)(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);

///////////////////////////////////////////////
// FUNCTIONS

void asm_yuvrgb565(uint8_t* dst, int32_t dst_rbytes, uint32_t* src, int y);

/**
 *  @fn      vp_stages_yuv2rgb_stage_open(vp_stages_yuv2rgb_config_t *)
 *  @brief   Open the YUV to RGB converter stage.
 *  @param   cfg  Stage configuration
 *  @return  C_RESULT : VP_SUCCESS
 */
C_RESULT vp_stages_yuv2rgb_stage_open(vp_stages_yuv2rgb_config_t *cfg);


/**
 *  @fn      vp_stages_yuv2rgb_stage_transform(vp_stages_yuv2rgb_config_t *, vp_api_io_data_t *, vp_api_io_data_t *)
 *  @brief   Apply a transform over the YUV to RGB converter stage.
 *
 *  Gets in input a vp_api_picture_t frame which contains the Y, U, and V arrays.\n
 *  Calls internal generic function vp_stages_YUV_to_RGB.
 *  @param   cfg  Stage configuration
 *  @param   in   Input buffer : contains a vp_api_picture_t *
 *  @param   out  Output buffer
 *  @return  C_RESULT : VP_SUCCESS
 *  @author  Julien Floret <julien.floret.ext\@parrot.com>
 *  @date    Last modification  30/04/2007
 */
C_RESULT vp_stages_yuv2rgb_stage_transform(vp_stages_yuv2rgb_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);


/**
 *  @fn      vp_stages_yuv2rgb_stage_close(vp_stages_yuv2rgb_config_t *)
 *  @brief   Close the YUV to RGB converter stage.
 *  @param   cfg  Stage configuration
 *  @return  C_RESULT : VP_SUCCESS
 */
C_RESULT vp_stages_yuv2rgb_stage_close(vp_stages_yuv2rgb_config_t *cfg);

// vp_stages_yuv2rgb
/** @} */
// VP_Stages
/** @} */

#endif // ! _VP_STAGES_YUV2RGB_INCLUDE_
