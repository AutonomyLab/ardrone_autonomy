#include <VLIB/P264/p264_merge.h>

// merge scaled residual data from iTransform and prediction data already located in the output picture
void p264_merge_4x4 (int16_t* residual,uint8_t* picture, uint32_t x, uint32_t y, uint32_t linesize)
{
  uint32_t i;
  int16_t pixel;
  picture += y*linesize+x;
  for (i=0;i<4;i++)
  {
    pixel = (int16_t)(((int16_t)*picture) + (((*residual++)+32)>>6));
    if (pixel > 0xFF)
      *picture++ = 0xFF;
    else if (pixel < 0)
      *picture++ = 0;
    else
      *picture++ = pixel;
    pixel = (int16_t)(((int16_t)*picture) + (((*residual++)+32)>>6));
    if (pixel > 0xFF)
      *picture++ = 0xFF;
    else if (pixel < 0)
      *picture++ = 0;
    else
      *picture++ = pixel;
    pixel = (int16_t)(((int16_t)*picture) + (((*residual++)+32)>>6));
    if (pixel > 0xFF)
      *picture++ = 0xFF;
    else if (pixel < 0)
      *picture++ = 0;
    else
      *picture++ = pixel;
    pixel = (int16_t)(((int16_t)*picture) + (((*residual++)+32)>>6));
    if (pixel > 0xFF)
      *picture = 0xFF;
    else if (pixel < 0)
      *picture = 0;
    else
      *picture = pixel;
    // jump to next line
    picture += linesize-3;
  }
}
