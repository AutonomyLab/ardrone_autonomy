#include <VP_Os/vp_os_types.h>

#include <VLIB/video_controller.h>

#include "video_utils.h"

#if TARGET_CPU_ARM == 1

static uint32_t num_references = 0;

C_RESULT video_utils_init( video_controller_t* controller )
{
  if( num_references == 0 )
  {
  }

  num_references ++;

  return C_OK;
}

C_RESULT video_utils_close( video_controller_t* controller )
{
  if( num_references > 0 )
  {
    num_references --;
  }

  return C_OK;
}

C_RESULT video_utils_set_format( uint32_t width, uint32_t height )
{
  return C_OK;
}

#endif
