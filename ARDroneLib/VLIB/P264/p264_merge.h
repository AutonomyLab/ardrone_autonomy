#include <VP_Os/vp_os_types.h>

// merge unscaled residual data from itransform and prediction data already located in the output picture
void p264_merge_4x4 (int16_t* residual,uint8_t* picture, uint32_t x, uint32_t y, uint32_t linesize);
