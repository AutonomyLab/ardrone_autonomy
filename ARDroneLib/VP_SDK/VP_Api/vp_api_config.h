/*! \file     vp_api_config.h
    \brief    VP Api. Pipeline configuration
    \author   Julien Floret <julien.floret.ext@parrot.com>
    \version  1.0
    \date     22/05/2007
*/


#ifndef _VP_API_CONFIG_H_
#define _VP_API_CONFIG_H_

#define STAGE_JULIEN

///////////////////////////////////////////////
// VP_API_SUPERVISOR

/**
 *  @def    VP_API_MAX_NUM_PIPELINES
 *  @brief  Encoder buffer size
 */
#define VP_API_MAX_NUM_PIPELINES             16

/**
 *  @def    VP_API_MAX_NUM_STAGES
 *  @brief  Encoder buffer size
 */
#define VP_API_MAX_NUM_STAGES                128

/**
 *  @def    VP_API_PIPELINE_FIFO_SIZE
 *  @brief  Encoder buffer size
 */
#define VP_API_PIPELINE_FIFO_SIZE          (4096)


///////////////////////////////////////////////
// VP_STAGES_IO_FILTER

/**
 *  @def    VP_STAGES_IO_FILTER_OUTBUF_SIZE
 *  @brief  Encoder buffer size
 */
#define VP_STAGES_IO_FILTER_OUTBUF_SIZE 100000
#define VP_STAGES_IO_FFMPEG_OUTBUF_SIZE 100000

/**
 *  @def    VP_STAGES_IO_FILTER_DECODER_TIME_STATS
 *  @brief  Display number of frames/sec
 */
#define VP_STAGES_IO_FILTER_DECODER_TIME_STATS

#ifdef VP_STAGES_IO_FILTER_DECODER_TIME_STATS
# define VP_STAGES_IO_FILTER_NB_FRAMES_DISPLAY  500
#endif  // < VP_STAGES_IO_FILTER_DECODER_TIME_STATS

///////////////////////////////////////////////
// VP_STAGES_O_LCD

/**
 *  @def    VP_STAGES_O_LCD_NUMBER_OF_FRAMEBUFFER_MAX
 *  @brief  Max number of LCD buffers.
 */
#define VP_STAGES_O_LCD_NUMBER_OF_FRAMEBUFFER_MAX 3

//#define VP_STAGES_O_LCD_CADENCED_DISPLAY

/**
 *  @def    VP_STAGES_O_LCD_TICKS_FRAME
 *  @brief  Time between two displays.
 *
 *  In case of a cadenced display, this value indicates how much time a frame is displayed.
 */
#define VP_STAGES_O_LCD_TICKS_FRAME      4

///////////////////////////////////////////////
// VP_STAGES_YUV2RGB

//#define QCIF_TO_QVGA

#ifdef QCIF_TO_QVGA
# define USE_YUV2RGB_STRETCH
#endif  // < QCIF_TO_QVGA

//#define VP_STAGES_YUV2RGB_NOT_ALL_FRAMES

#ifdef VP_STAGES_YUV2RGB_NOT_ALL_FRAMES
# define VP_STAGES_YUV2RGB_NB_FRAMES             3
#endif  // < VP_STAGES_YUV2RGB_NOT_ALL_FRAMES


#endif // ! _VP_API_CONFIG_H_
