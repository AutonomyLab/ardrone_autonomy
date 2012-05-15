#include <VLIB/video_quantizer.h>
#include <VLIB/Platform/video_utils.h>

C_RESULT video_quantizer_init( video_controller_t* controller )
{
  // Init quantizer's value
  // This value is between 1 and 31
  // TODO: this is not true in case of P264 but P264 don't use the video_quantizer functions. To be fixed

  int32_t quant = controller->quant;

  if( quant < 1 )
    quant = 1;
  else if( quant >= 32 )
    quant = 31;

  if( controller->picture_type == VIDEO_PICTURE_INTRA )
  {
    controller->Qp    = 0;
    controller->invQp = (1 << 16) / (2*quant);
  }
  else // VIDEO_PICTURE_INTER
  {
    controller->Qp    = quant / 2;
    controller->invQp = (1 << 16) / (2*quant);
  }

  controller->dquant = 0;

  return C_OK;
}

C_RESULT video_quantizer_update( video_controller_t* controller )
{
  // Update quantizer's value
  int32_t quant = controller->quant;

  if( quant < 1 )
    quant = 1;
  else if( quant >= 32 )
    quant = 31;

  if( controller->picture_type == VIDEO_PICTURE_INTRA )
  {
    controller->Qp    = 0;
    controller->invQp = (1 << 16) / (2*quant);
  }
  else // VIDEO_PICTURE_INTER
  {
    controller->Qp    = quant / 2;
    controller->invQp = (1 << 16) / (2*quant);
  }

  controller->dquant = 0;

  return C_OK;
}

C_RESULT video_quantize( video_controller_t* controller, video_macroblock_t* macroblock, int32_t num_macro_blocks )
{
  int32_t sum_y, sum_c, dc0, dc1, dc2, dc3, dcb, dcr;
  int16_t *y0;

  y0  = macroblock->data;

  while( num_macro_blocks > 0 )
  {
    if( controller->do_azq == TRUE )
    {
      int16_t *y1, *y2, *y3, *cb, *cr;

      y1  = y0 + MCU_BLOCK_SIZE;
      y2  = y1 + MCU_BLOCK_SIZE;
      y3  = y2 + MCU_BLOCK_SIZE;
      cb  = y3 + MCU_BLOCK_SIZE;
      cr  = cb + MCU_BLOCK_SIZE;

      // Test for azq (all zero quantized) in luma blocks
      dc0 = *y0;
      dc1 = *y1;
      dc2 = *y2;
      dc3 = *y3;
      dcb = *cb;
      dcr = *cr;

      sum_y = dc0 + dc1 + dc2 + dc3;
      sum_c = dcb + dcr;
    }
    else
    {
      sum_y = 0x7F000000;
      sum_c = 0x7F000000;
    }

    macroblock->azq = (sum_y < controller->aq) & (sum_c < controller->bq);
    macroblock->dquant = controller->dquant;

    // Perform quantification on coefficients if necessary
    if( !macroblock->azq )
    {
      RTMON_USTART(VIDEO_VLIB_DOQUANTIZE);
      if( controller->picture_type == VIDEO_PICTURE_INTRA ) // intra
      {
        y0 = do_quantize_intra_mb(y0, controller->quant, &macroblock->num_coeff_y0);
      }
      else
      {
        y0 = do_quantize_inter_mb(y0, controller->Qp, controller->invQp, &macroblock->num_coeff_y0);
      }
      RTMON_USTOP(VIDEO_VLIB_DOQUANTIZE);
    }

    macroblock++;
    num_macro_blocks--;

    if( macroblock->azq )
    {
      y0  = macroblock->data;
    }
  }

  return C_OK;
}

C_RESULT video_unquantize( video_controller_t* controller, video_macroblock_t* macroblock, int32_t num_macro_blocks )
{
  int16_t *dst;

  controller->quant += macroblock->dquant;

  dst  = macroblock->data;

  while( num_macro_blocks > 0 )
  {
    // TODO Check generated code

    if( !macroblock->azq )
      do_unquantize(dst, controller->picture_type, controller->quant, macroblock->num_coeff_y0);

    dst += MCU_BLOCK_SIZE;

    if( !macroblock->azq )
      do_unquantize(dst, controller->picture_type, controller->quant, macroblock->num_coeff_y1);

    dst += MCU_BLOCK_SIZE;

    if( !macroblock->azq )
      do_unquantize(dst, controller->picture_type, controller->quant, macroblock->num_coeff_y2);

    dst += MCU_BLOCK_SIZE;

    if( !macroblock->azq )
      do_unquantize(dst, controller->picture_type, controller->quant, macroblock->num_coeff_y3);

    dst += MCU_BLOCK_SIZE;

    if( !macroblock->azq )
      do_unquantize(dst, controller->picture_type, controller->quant, macroblock->num_coeff_cb);

    dst += MCU_BLOCK_SIZE;

    if( !macroblock->azq )
      do_unquantize(dst, controller->picture_type, controller->quant, macroblock->num_coeff_cr);

    dst += MCU_BLOCK_SIZE;

    macroblock ++;
    num_macro_blocks--;
  }

  return C_OK;
}

#ifndef HAS_DO_QUANTIZE_INTRA_MB
int16_t* do_quantize_intra_mb(int16_t* ptr, int32_t quant, int32_t* last_ptr)
{
  int32_t i, num_coeff;
  int32_t coeff, last;
  int32_t j;
  if (quant == TABLE_QUANTIZATION)
        quant = TBL_QUANT_QUALITY;

  for( i = 6; i > 0; i-- )
  {
    coeff = *ptr;
    last = 1;
    num_coeff = MCU_BLOCK_SIZE-1;
    coeff = coeff / (QUANT_I(0,quant));
    if( coeff == 0 )
      coeff = 1;
    *ptr = coeff;
    ptr++;
    j = 1;

    while( num_coeff > 0 )
    {
      coeff = *ptr;
      if( coeff != 0 )
      {
        // TODO : division can be slow, a better implementation would consist in building an invQuant(j) table [2^16/QUANT_I(j,quant)] and computing (coeff*invQuant(j))>>16 instead of coeff / (QUANT_I(j,quant))
        coeff = coeff / (QUANT_I(j,quant));

        if( coeff != 0 )
        {
          last++;
        }

        *ptr = coeff;
      }
      ptr++;
      num_coeff--;
      j++;
    }

    *last_ptr++ = last;
  }

  return ptr;
}
#endif // HAS_DO_QUANTIZE_INTRA_MB

int16_t* do_quantize_inter_mb(int16_t* ptr, int32_t quant, int32_t invQuant, int32_t* last_ptr)
{
  int32_t i, num_coeff;
  int32_t coeff, sign, last;

  // LEVEL = ( |COF| - QUANT/2 ) / (2*QUANT) see III.3.2.1

  for( i = 6; i > 0; i-- )
  {
    last = 0;
    num_coeff = MCU_BLOCK_SIZE;

    while( num_coeff > 0 )
    {
      coeff = *ptr;

      if( coeff != 0 )
      {
        sign = coeff < 0;

        if( sign )
          coeff = -coeff;

        coeff -= quant;
        coeff *= invQuant;
        coeff >>= 16;

        if( sign )
          coeff = -coeff;

        if( coeff != 0 )
        {
          last++;
        }

      }

      ptr++;
      num_coeff--;
    }

    *last_ptr++ = last;
  }

  return ptr;
}

#ifndef HAS_DO_UNQUANTIZE
C_RESULT do_unquantize(int16_t* ptr, int32_t picture_type, int32_t quant, int32_t num_coeff)
{
  int32_t coeff;
  uint32_t i=0;

  if (quant == TABLE_QUANTIZATION)
	quant = TBL_QUANT_QUALITY; // TABLE_QUANTIZATION is an old mode and is equivalent to a QUANT_I(i,2) quantization table

  // table quantization mode
  i=0;
  do
  {
	coeff = *ptr;
	if( coeff )
	{
	  coeff *= QUANT_I (i,quant);
	  *ptr = coeff;
	  num_coeff--;
	}
	i++;
	ptr++;
  } while( num_coeff > 0 );

  return C_OK;
}
#endif
