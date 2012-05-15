#include <VLIB/Platform/video_utils.h>
#include <VLIB/Platform/video_config.h>

#include <VLIB/video_quantizer.h>
#include <VLIB/video_dct.h>
#include <VLIB/video_mem32.h>
#include <VLIB/video_packetizer.h>

#include <VLIB/UVLC/uvlc_codec.h>
#include <VLIB/UVLC/uvlc.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>

#if TARGET_CPU_ARM == 1

#ifdef HAS_UVLC_DECODE_BLOCKLINE
C_RESULT uvlc_read_block_unquantize( video_controller_t* controller, int16_t* data, int32_t quant, int32_t nc )
{
	video_stream_t* stream = &controller->in_stream;
	int32_t* zztable;
	int32_t index, code, run, last;

	zztable = &video_zztable_t81[0];
	code = run = last = 0;
	
	if (quant == TABLE_QUANTIZATION)
	{
	  quant = TBL_QUANT_QUALITY;
	}
	  // table quantization mode
	  video_read_data( stream, (uint32_t*) &code, 10 );
	  code *= QUANT_I(0,quant);
	  data[0] = code;

	  if( nc > 0 )
	  {
		uvlc_decode( stream, &run, &code, &last);
	    while (last == 0)
	    {
	      VP_OS_ASSERT( run < 64 );

	      zztable    += (run+1);
	      index       = *zztable;
	  	code *= QUANT_I(index,quant);
		  data[index] = code;
		  uvlc_decode( stream, &run, &code, &last);
	    }
	  }
	
	return C_OK;
}

uint16_t* uvlc_read_mb_layer_unquantize( video_controller_t* controller, video_macroblock_t* mb, uint16_t* out )
{
	int16_t* data;
	uint32_t code;

	video_zeromem32( (uint32_t*)mb->data, 6 * MCU_BLOCK_SIZE / 2 );

		mb->azq = 0;
		video_read_data( &controller->in_stream, (uint32_t*)&mb->azq, 1 );

		if( mb->azq == 0 )
		{
			video_read_data( &controller->in_stream, &code, 8 );

			mb->num_coeff_y0 = (code >> 0) & 1;
			mb->num_coeff_y1 = (code >> 1) & 1;
			mb->num_coeff_y2 = (code >> 2) & 1;
			mb->num_coeff_y3 = (code >> 3) & 1;
			mb->num_coeff_cb = (code >> 4) & 1;
			mb->num_coeff_cr = (code >> 5) & 1;

			mb->dquant = 0;
			if( (code >> 6) & 1 )
			{
				video_read_data( &controller->in_stream, &code, 2 );

				mb->dquant = (code < 2) ? ~code : code;
			}

			controller->quant += mb->dquant;

			/**************** Block Y0 ****************/
			data = mb->data;
			uvlc_read_block_unquantize( controller, data, controller->quant, mb->num_coeff_y0 );
			idct(data, out);
			out  += MCU_BLOCK_SIZE;

			/**************** Block Y1 ****************/
			data += MCU_BLOCK_SIZE;
			uvlc_read_block_unquantize( controller, data, controller->quant, mb->num_coeff_y1 );
			idct(data, out);
			out  += MCU_BLOCK_SIZE;

			/**************** Block Y2 ****************/
			data += MCU_BLOCK_SIZE;
			uvlc_read_block_unquantize( controller, data, controller->quant, mb->num_coeff_y2 );
			idct(data, out);
			out  += MCU_BLOCK_SIZE;

			/**************** Block Y3 ****************/
			data += MCU_BLOCK_SIZE;
			uvlc_read_block_unquantize( controller, data, controller->quant, mb->num_coeff_y3 );
			idct(data, out);
			out  += MCU_BLOCK_SIZE;

			/**************** Block CB ****************/
			data += MCU_BLOCK_SIZE;
			uvlc_read_block_unquantize( controller, data, controller->quant, mb->num_coeff_cb );
			idct(data, out);
			out  += MCU_BLOCK_SIZE;

			/**************** Block CR ****************/
			data += MCU_BLOCK_SIZE;
			uvlc_read_block_unquantize( controller, data, controller->quant, mb->num_coeff_cr );
			idct(data, out);
			out  += MCU_BLOCK_SIZE;
		}

		mb ++;

	return out;
}

C_RESULT uvlc_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image )
{
	video_codec_t* video_codec;
	vp_api_picture_t blockline = { 0 };
	int16_t *in = NULL;
	uint16_t *out = NULL;
	int32_t num_macro_blocks = 0;
	video_macroblock_t* macroblock = NULL;
	video_picture_context_t blockline_ctx;
	video_gob_t*  gobs;

	controller->mode  = VIDEO_DECODE;
	video_codec       = controller->video_codec;

	blockline                   = *picture;
	blockline.height            = MB_HEIGHT_Y;
	blockline.complete          = 1;
	blockline.vision_complete   = 0;

	picture->complete  = controller->picture_complete;

	blockline_ctx.y_woffset = blockline.y_line_size;
	blockline_ctx.c_woffset = blockline.cb_line_size;
	blockline_ctx.y_hoffset = blockline.y_line_size * MCU_HEIGHT;

	// At least a complete blockline is found
	while( !controller->picture_complete && controller->in_stream.index <= (controller->in_stream.used >> 2) )
	{
		uvlc_unpack_controller( controller );

		if( !controller->picture_complete )
		{
			blockline.blockline  = controller->blockline;

			blockline_ctx.y_src     = picture->y_buf + blockline.blockline * MB_HEIGHT_Y * picture->y_line_size;
			blockline_ctx.cb_src    = picture->cb_buf + blockline.blockline * MB_HEIGHT_C * picture->cb_line_size;
			blockline_ctx.cr_src    = picture->cr_buf + blockline.blockline * MB_HEIGHT_C * picture->cr_line_size;

			picture->blockline  = controller->blockline;
			num_macro_blocks    = controller->mb_blockline;

			macroblock  = &controller->cache_mbs[0];
			gobs        = &controller->gobs[controller->blockline];
			out         = (uint16_t*) gobs->macroblocks->data;

			if( gobs->quant != controller->quant )
			{
				controller->quant = gobs->quant;
				video_quantizer_update( controller );
			}

			while( num_macro_blocks > 0 )
			{
				in = &macroblock->data[0];

				out = uvlc_read_mb_layer_unquantize( controller, macroblock, out );

				num_macro_blocks --;
			}

			video_blockline_from_macro_blocks(&blockline_ctx, gobs->macroblocks->data, controller->mb_blockline, picture->format);

			// Update controller according to video statistics
			video_controller_update( controller, controller->picture_complete );
		}
	}

	if( controller->picture_complete )
	{
		picture->complete   = controller->picture_complete;
		picture->blockline  = 0;

		controller->picture_complete  = 0;
		controller->in_stream.length  = 32;
		//controller->num_frames++;

		*got_image = TRUE;
	}
	else
	{
		controller->in_stream.used  = 0;
		controller->in_stream.index = 0;
	}

	return C_OK;
}
#endif

#endif // TARGET_CPU_ARM == 1
