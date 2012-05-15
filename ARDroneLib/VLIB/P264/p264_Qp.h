#include <VP_Os/vp_os_types.h>

// rescaling functions
void p264_4x4_residual_scale(int16_t* in, int16_t* out, uint32_t Qp);
void p264_2x2_chromaDC_scale(int16_t* in, int16_t* out, uint32_t Qp);
void p264_4x4_lumaDC_scale(int16_t* in, int16_t* out, uint32_t Qp);
