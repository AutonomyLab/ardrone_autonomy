#include <VP_Os/vp_os_types.h>
#include <VLIB/video_macroblock.h>
#include <VLIB/video_controller.h>


// prepare ip to encode a new frame
C_RESULT video_p264_prepare_slice ( video_controller_t* controller, const vp_api_picture_t* blockline);

// encode num_macro_blocks MB
C_RESULT video_p264_encode_MB(uint32_t num_macro_blocks, video_macroblock_t* next_macroblock ,int32_t qp);

// get encoded num_macro_blocks MB
int32_t video_p264_get_encoded_MB(uint32_t num_macro_blocks, video_macroblock_t* next_macroblock);


// inter decoding functions
void video_p264_decode_inter_luma_MB (uint8_t * ref_picture, uint8_t* picture_out, uint32_t x,uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize, MV_XY_t* mv , inter_partition_mode_t * part, uint32_t nb_part, int16_t* AC, uint32_t qp);

void video_p264_decode_inter_chroma_MB (uint8_t * ref_picture, uint8_t* picture_out, uint32_t x,uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize, MV_XY_t* mv , inter_partition_mode_t * part, uint32_t nb_part, int16_t* DC, int16_t* AC, uint32_t qp);

// intra decoding functions
void video_p264_decode_intra_luma_4x4_MB (int16_t* AC, uint8_t* picture_out, uint32_t picture_width, uint32_t x,uint32_t y, uint32_t linesize, intra_4x4_mode_t * intra_mode,uint32_t qp);

void video_p264_decode_intra_chroma_8x8_MB (int16_t* DC,int16_t* AC,uint8_t* picture_out,uint32_t x,uint32_t y, uint32_t linesize,intra_8x8_chroma_mode_t chroma_mode, uint32_t qp);

void video_p264_decode_intra_luma_16x16_MB (int16_t* DC,int16_t* AC, uint8_t* picture_out,uint32_t x,uint32_t y, uint32_t linesize, intra_16x16_luma_mode_t intra_mode,uint32_t qp);

/////////////// Debug functions /////////////
void print_MB_DCT(MB_p264_t* mb_intra,intra_type_t intra_4x4);
void mat_printf_4x4(int16_t * mat);
void mat_printf_2x2(int16_t * mat);
void mat_printf_16x16_inside_picture(uint8_t * picture,uint32_t x,uint32_t y,uint32_t linesize);
void mat_printf_8x8_inside_picture(uint8_t * picture,uint32_t x,uint32_t y, uint32_t linesize);
