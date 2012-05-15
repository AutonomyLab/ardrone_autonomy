#ifndef _VISION_COMMON_H_
#define _VISION_COMMON_H_

// NUMBER OF TRACKERS FOR EACH TRACKING
#define NB_CORNER_TRACKERS_WIDTH    5      /* number of trackers in width of current picture */
#define NB_CORNER_TRACKERS_HEIGHT   4      /* number of trackers in height of current picture */

#define DEFAULT_NB_TRACKERS_WIDTH    (NB_CORNER_TRACKERS_WIDTH+1)// + NB_BLOCK_TRACKERS_WIDTH)
#define DEFAULT_NB_TRACKERS_HEIGHT   (NB_CORNER_TRACKERS_HEIGHT+1)// + NB_BLOCK_TRACKERS_HEIGHT)

#define YBUF_OFFSET                 0

// use by ihm/ihm_vision.c
#define DEFAULT_CS                  1
#define DEFAULT_NB_PAIRS            1
#define DEFAULT_LOSS_PER            1
#define DEFAULT_SCALE               1
#define DEFAULT_TRANSLATION_MAX     1
#define DEFAULT_MAX_PAIR_DIST       1
#define DEFAULT_NOISE               1

typedef enum _CAMIF_CAMERA_ENUM_
{
  CAMIF_CAMERA_LB,

  CAMIF_CAMERA_CRESYN,
  CAMIF_CAMERA_VS6524,
  CAMIF_CAMERA_OV7710,
  CAMIF_CAMERA_OV7720,
  CAMIF_CAMERA_OVTRULY,
  CAMIF_CAMERA_OVTRULY_UPSIDE_DOWN_ONE_BLOCKLINE_LESS,

  CAMIF_CAMERA_UB
}
CAMIF_CAMERA;

#endif //_VISION_COMMON_H

