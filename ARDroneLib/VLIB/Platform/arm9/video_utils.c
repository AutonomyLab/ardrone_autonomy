#include <VLIB/video_controller.h>
#include <VLIB/video_picture.h>

#include <VLIB/Platform/video_config.h>
#include <VLIB/Platform/video_utils.h>

#include <stdio.h>

static uint32_t num_references = 0;

#if (VIDEO_DCT_USE_INTRAM == 1)
// Put DMA buffer into INTRAM, this should avoid bus contention with CAMIF
static int16_t dct_buffer[2*DCT_BUFFER_SIZE] __attribute__((aligned(CACHE_DLINESIZE), section(".data.fast")));
#endif

C_RESULT video_utils_init( video_controller_t* controller )
{
  if( num_references == 0 )
  {
#if (VIDEO_DCT_USE_INTRAM == 1)
    // Put in non cached INTRAM
    controller->blockline_cache = (int16_t*) (((CYG_ADDRESS) &dct_buffer[0]) + CYGMEM_REGION_intram_SHADOW - CYGMEM_REGION_intram);
#endif
  }

  num_references ++;

  return C_OK;
}

C_RESULT video_utils_close( video_controller_t* controller )
{
  if( num_references > 0 )
  {
    num_references --;

    if( num_references == 0 )
    {
#if (VIDEO_DCT_USE_INTRAM == 1)
      // Put in non cached INTRAM
      controller->blockline_cache = NULL;
#endif
    }
  }

  return C_OK;
}

uint32_t ramcode_format_shifter_op_imm(uint32_t imm)
{
  uint32_t shifter, imm8;

  shifter = 32/2;
  imm8    = imm;

  while( (imm8 & 0xFF) != imm8 )
  {
    imm8 >>= 2;
    shifter --;
  }

  if( (imm8 << (32-shifter*2)) != imm )
    return 0xFFFFFFFF;

  if( shifter == 16 )
    shifter = 0;

  return (shifter << 8 | imm8);
}

C_RESULT video_utils_set_format( uint32_t width, uint32_t height )
{
  return C_OK;
}
