#include <VP_Os/vp_os_types.h>
#include "p264_common.h"

void p264_inter_mc_luma (inter_partition_mode_t partition, MV_XY_t mv,uint8_t *picture_ref , uint8_t *picture, uint32_t x, uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize);
void p264_inter_mc_chroma (inter_partition_mode_t partition, MV_XY_t mv,uint8_t *picture_ref , uint8_t *picture, uint32_t x, uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize);

void video_p264_decode_inter_chroma_MB (uint8_t * ref_picture, uint8_t* picture_out, uint32_t x,uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize, MV_XY_t* mv , inter_partition_mode_t * part, uint32_t nb_part, int16_t* DC, int16_t* AC, uint32_t qp);

