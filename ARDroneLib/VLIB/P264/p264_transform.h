#include <VP_Os/vp_os_types.h>

// hadamard_2x2 and ihadamard_2x2 are the same
void p264_hadamard_2x2 (int16_t * in, int16_t * out);
void p264_ihadamard_4x4 (int16_t *tblock, int16_t *block);
void p264_inverse_4x4(int16_t *tblock, int16_t *block);
