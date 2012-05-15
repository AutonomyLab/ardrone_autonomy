#ifndef _VIDEO_QUANTIZER_H_
#define _VIDEO_QUANTIZER_H_

#include <VLIB/video_controller.h>

// (quant > 0)&&(quant < 31) => use table quantization generated with QUANT_I(i,quant) macro
// quant == 31 				 => old quantization table mode, means QUANT_I(i,2) (backward compatibility)
#define TABLE_QUANTIZATION 31	// quant transmitted for old TABLE_QUANTIZATION mode
#define TBL_QUANT_QUALITY 2		// real quant used for old TABLE_QUANTIZATION mode

// quantization macro
#define QUANT_IJ(i,j,quality) (1 + (1 +(i) + (j))*(quality))
#define QUANT_I(i,quality) (1 + (1 +(i/8) + (i%8))*(quality))

// Utility functions
int16_t* do_quantize_intra_mb(int16_t* ptr, int32_t invQuant, int32_t* last_ptr);
int16_t* do_quantize_inter_mb(int16_t* ptr, int32_t quant, int32_t invQuant, int32_t* last_ptr);
C_RESULT do_unquantize(int16_t* ptr, int32_t picture_type, int32_t quant, int32_t num_coeff);

// Default quantization scheme
C_RESULT video_quantizer_init( video_controller_t* controller );
C_RESULT video_quantizer_update( video_controller_t* controller );
C_RESULT video_quantize( video_controller_t* controller, video_macroblock_t* macroblock, int32_t num_macro_blocks );
C_RESULT video_unquantize( video_controller_t* controller, video_macroblock_t* macroblock, int32_t num_macro_blocks );

#endif // _VIDEO_QUANTIZER_H_
