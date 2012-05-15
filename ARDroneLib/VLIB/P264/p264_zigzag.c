#include <VLIB/P264/p264_zigzag.h>

int32_t video_zztable_t41[16] = {
  0,  1,  4,  8,
  5,  2,  3,  6,
  9, 12, 13, 10,
  7, 11, 14, 15
};

void zagzig_4x4 (int16_t * in, int16_t * out)
{
  int32_t* zztable;
  uint32_t i;

  zztable = &video_zztable_t41[0];
  i = 16;
  while (i--)
  {
    out[*zztable++] = *in++;
  }
}
