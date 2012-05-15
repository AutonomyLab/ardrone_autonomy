#include <VLIB/video_controller.h>
#include <VLIB/video_picture.h>

#include <VLIB/Platform/video_config.h>
#include <VLIB/Platform/video_utils.h>
#include <VLIB/video_codec.h>

#include <VLIB/Platform/arm9_P6/video_dct_p6.h>
#include <VLIB/Platform/arm9_P6/video_p264_p6.h>

#include <VP_Os/vp_os_print.h>

static uint32_t num_references = 0;

C_RESULT video_utils_init( video_controller_t* controller )
{
  if( num_references == 0 )
  {
    switch(controller->codec_type)
    {
      case UVLC_CODEC :
        video_dct_p6_init();
        break;
      case P264_CODEC :
        video_p264_p6_init();
        break;
      default :
        PRINT ("%s unknown codec %d\n",__FUNCTION__,controller->codec_type);
        return C_FAIL;
    }
  }

  num_references ++;

  return C_OK;
}

C_RESULT video_utils_close( video_controller_t* controller )
{
  if( num_references > 0 )
  {
    switch(controller->codec_type)
    {
      case UVLC_CODEC :
        video_dct_p6_close();
        break;
      case P264_CODEC :
        video_p264_p6_close();
        break;
      default :
        PRINT ("%s unknown codec %d\n",__FUNCTION__,controller->codec_type);
        return C_FAIL;
    }
    num_references --;
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
