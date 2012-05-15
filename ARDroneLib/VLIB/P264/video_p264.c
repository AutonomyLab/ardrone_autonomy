#include <VLIB/P264/video_p264.h>
#include <VLIB/P264/p264_transform.h>
#include <VLIB/P264/p264_merge.h>
#include <VLIB/Platform/video_utils.h>
#include "p264_inter_mc.h"
#include "p264_intra_pred.h"
#include "p264_zigzag.h"
#include "p264_Qp.h"
#include <VP_Os/vp_os_print.h>

//#define VP264_CHROMA_DEBUG
//#define VP264_LUMA_DEBUG
//#define VP264_LUMA_16x16_DEBUG
//#define VP264_PLUMA_DEBUG
//#define VP264_PCHROMA_DEBUG

#ifndef HAS_P264_FTRANSFORM
//#warning "soft p264 encoding function not implemented"

// prepare ip to encode a new frame
C_RESULT video_p264_prepare_slice ( video_controller_t* controller, const vp_api_picture_t* blockline)
{
  return C_FAIL;
}

// encode num_macro_blocks MB
C_RESULT video_p264_encode_MB(uint32_t num_macro_blocks, video_macroblock_t* next_macroblock ,int32_t qp)
{
  return -1;
}

// get encoded num_macro_blocks MB
int32_t video_p264_get_encoded_MB(uint32_t num_macro_blocks, video_macroblock_t* next_macroblock)
{
  return -1;
}
#endif

// block_4x4 coordinates
typedef struct _block_XY_t {
  uint32_t x;
  uint32_t y;
} block_XY_t;

// block_4x4 decoding order
static block_XY_t block_4x4_decoding_order[BLOCK_SIZE2] = {
{0,0},{4,0},{0,4},{4,4},
{8,0},{12,0},{8,4},{12,4},
{0,8},{4,8},{0,12},{4,12},
{8,8},{12,8},{8,12},{12,12}
};

// intra
void video_p264_decode_intra_luma_4x4_MB (int16_t* AC, uint8_t* picture_out, uint32_t picture_width, uint32_t x,uint32_t y, uint32_t linesize,intra_4x4_mode_t * intra_mode,uint32_t qp)
{
  int16_t tmp_block_4x4[BLOCK_SIZE2];
  int16_t itransform_block_4x4[BLOCK_SIZE2];
  uint32_t diff_x=0,diff_y=0;
  uint32_t i=16;
  block_XY_t * p_block_4x4_decoding_order = block_4x4_decoding_order;

  while(i--)
  {
    // load relative coordinates of next 4x4 block to be decoded
    diff_x = p_block_4x4_decoding_order->x;
    diff_y = (p_block_4x4_decoding_order++)->y;
#ifdef VP264_LUMA_DEBUG
        PRINT ("======= next 4x4 block : (%d,%d) with mode %d\n",diff_x,diff_y,*intra_mode);
#endif

    // do luma intra_4x4(0,0) prediction
    p264_intra_4x4_luma((intra_4x4_mode_t)*intra_mode,
            picture_out, picture_width,
            x+diff_x,y+diff_y, linesize);
#ifdef VP264_LUMA_DEBUG
      PRINT("frame after first block 4x4 prediction\n");
      mat_printf_16x16_inside_picture(picture_out,x,y,linesize);
#endif
    // zag zig residual block_4x4 (0,0)
    zagzig_4x4 (AC, tmp_block_4x4);

    // scale residual block
    p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);

    // itransform residual luma
    p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);

    // merge residual data and intra prediction
    p264_merge_4x4 (itransform_block_4x4,picture_out,x+diff_x,y+diff_y, linesize);
#ifdef VP264_LUMA_DEBUG
      PRINT("frame after first block 4x4 merging\n");
      mat_printf_16x16_inside_picture(picture_out,x,y,linesize);
#endif
    // jump to next residual data
    AC += BLOCK_SIZE2;
    // jump to next intra_4x4 mode
    intra_mode++;
  }
}

void video_p264_decode_intra_luma_16x16_MB (int16_t* DC, int16_t* AC, uint8_t* picture_out,uint32_t x,uint32_t y, uint32_t linesize, intra_16x16_luma_mode_t intra_mode, uint32_t qp)
{
  // DC : the 16 DC coeffs of the current luma MB
  // AC : the 16 AC[16] coeffs of the current chroma MB
  int16_t ihadamard_luma_4x4[BLOCK_SIZE2];
  int16_t itransform_block_4x4[BLOCK_SIZE2];
  int16_t tmp_block_4x4[BLOCK_SIZE2];
  uint32_t diff_x=0,diff_y=0;
  uint32_t i;
  block_XY_t * p_block_4x4_decoding_order = block_4x4_decoding_order;

  // do prediction
  p264_intra_16x16_luma (intra_mode, picture_out, x, y, linesize);
#ifdef VP264_LUMA_16x16_DEBUG
       PRINT("frame after intra 16x16 prediction\n",diff_x,diff_y);
       mat_printf_16x16_inside_picture(picture_out,x,y,linesize);
#endif

  // perform ihadamard on DC chroma
  p264_ihadamard_4x4 (DC, ihadamard_luma_4x4);

  // scale DC luma coeff
  p264_4x4_lumaDC_scale(ihadamard_luma_4x4,ihadamard_luma_4x4,qp);

  for (i=0;i<16;i++)
  {
    // load relative coordinates of next 4x4 block to be decoded
    diff_x = p_block_4x4_decoding_order->x;
    diff_y = (p_block_4x4_decoding_order++)->y;

    //// block 4x4 (x+diff_x, y+diff_y)
    // zagzig
    zagzig_4x4 (AC,tmp_block_4x4);
    // scale residual block
    p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
    // insert DC coeff
    *tmp_block_4x4 = ihadamard_luma_4x4[diff_y + (diff_x>>2)];
    // itransform residual luma
    p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
    // merge residual data and intra prediction
    p264_merge_4x4 (itransform_block_4x4,picture_out,x+diff_x,y+diff_y, linesize);
#ifdef VP264_LUMA_16x16_DEBUG
       PRINT("frame after block(%d,%d) 4x4 merging\n",diff_x,diff_y);
       mat_printf_16x16_inside_picture(picture_out,x,y,linesize);
#endif
    // next block
    AC += BLOCK_SIZE2;
  }
}

void video_p264_decode_intra_chroma_8x8_MB (int16_t* DC,int16_t* AC,uint8_t* picture_out,uint32_t x,uint32_t y, uint32_t linesize,intra_8x8_chroma_mode_t chroma_mode, uint32_t qp)
{
  // DC : the four DC coeffs of the current chroma MB
  // AC : the four AC[16] coeffs of the current chroma MB
  int16_t ihadamard_chroma_2x2[4];
  int16_t itransform_block_4x4[BLOCK_SIZE2];
  int16_t tmp_block_4x4[BLOCK_SIZE2];

#ifdef VP264_CHROMA_DEBUG
  bool_t do_print = FALSE;
  if (x==0 && y==0)
    do_print = TRUE;
    else
    do_print = FALSE;
#endif

  // do chroma prediction
  p264_intra_8x8_chroma (chroma_mode, picture_out, x, y, linesize);
#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
      PRINT("frame after chroma 8x8 prediction\n");
      mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  // perform ihadamard on DC chroma
  p264_hadamard_2x2 (DC, ihadamard_chroma_2x2);

#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
        PRINT ("raw chroma DC[%d %d; %d %d]\n",DC[0],DC[1],DC[2],DC[3]);
   }
#endif
  // scale DC chroma coeff
  p264_2x2_chromaDC_scale(ihadamard_chroma_2x2,ihadamard_chroma_2x2,qp);

#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
      PRINT ("scaled DC chroma coef [%d %d; %d %d]\n",ihadamard_chroma_2x2[0],ihadamard_chroma_2x2[1],ihadamard_chroma_2x2[2],ihadamard_chroma_2x2[3]);
   }
#endif

  //// block 4x4 (0,0)
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[0];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x,y, linesize);
#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  //// block 4x4 (4,0)
  AC += BLOCK_SIZE2;
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[1];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x+4,y, linesize);
#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  //// block 4x4 (0,4)
  AC += BLOCK_SIZE2;
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[2];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x,y+4, linesize);
#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  //// block 4x4 (4,4)
  AC += BLOCK_SIZE2;
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[3];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x+4,y+4, linesize);
#ifdef VP264_CHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
}

// inter
void video_p264_decode_inter_luma_MB (uint8_t * ref_picture, uint8_t* picture_out, uint32_t x,uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize, MV_XY_t* mv , inter_partition_mode_t * part, uint32_t nb_part, int16_t* AC, uint32_t qp)
{
  int16_t tmp_block_4x4[BLOCK_SIZE2];
  int16_t itransform_block_4x4[BLOCK_SIZE2];
  uint32_t diff_x=0,diff_y=0;
  uint32_t i=16;
  block_XY_t * p_block_4x4_decoding_order = block_4x4_decoding_order;

#ifdef VP264_PLUMA_DEBUG
  bool_t do_print = FALSE;
  if (x==0 && y==0)
    do_print = TRUE;
    else
    do_print = FALSE;
#endif

  // make motion compensation
  while (nb_part--)
  {
    //TODO: this chunk of code is not complete, only 16x16 partition are supported
    p264_inter_mc_luma (*part++, *mv++, ref_picture , picture_out, x, y,picture_width, picture_height, linesize);
  }
#ifdef VP264_PLUMA_DEBUG
   if (do_print == TRUE)
   {
      PRINT("block (%d,%d) ref picture\n",x,y);
      mat_printf_16x16_inside_picture(ref_picture,x,y,linesize);
   }
#endif

#ifdef VP264_PLUMA_DEBUG
   if (do_print == TRUE)
   {
      PRINT("block (%d,%d) after motion compensation - first mv [%d,%d]\n",x,y,(*(mv-1)).x,(*(mv-1)).y);
      mat_printf_16x16_inside_picture(picture_out,x,y,linesize);
   }
#endif

  // residual data processing
  while(i--)
  {
    // load relative coordinates of next 4x4 block to be decoded
    diff_x = p_block_4x4_decoding_order->x;
    diff_y = (p_block_4x4_decoding_order++)->y;

    // zag zig residual block_4x4 (0,0)
    zagzig_4x4 (AC, tmp_block_4x4);

    // scale residual block
    p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);

    // itransform residual luma
    p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);

    // merge residual data and intra prediction
    p264_merge_4x4 (itransform_block_4x4,picture_out,x+diff_x,y+diff_y, linesize);
#ifdef VP264_PLUMA_DEBUG
    if (do_print == TRUE)
    {
      PRINT ("block merging\n");
      mat_printf_16x16_inside_picture(picture_out,x,y,linesize);
    }
#endif
    // jump to next residual data
    AC += BLOCK_SIZE2;
  }
}

void video_p264_decode_inter_chroma_MB (uint8_t * ref_picture, uint8_t* picture_out, uint32_t x,uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize, MV_XY_t* mv , inter_partition_mode_t * part, uint32_t nb_part, int16_t* DC, int16_t* AC, uint32_t qp)
{
  // DC : the four DC coeffs of the current chroma MB
  // AC : the four AC[16] coeffs of the current chroma MB
  int16_t ihadamard_chroma_2x2[4];
  int16_t itransform_block_4x4[BLOCK_SIZE2];
  int16_t tmp_block_4x4[BLOCK_SIZE2];

#ifdef VP264_PCHROMA_DEBUG
  static uint32_t toto=0;
  toto++;
  bool_t do_print = FALSE;
  if (x==0 && y==0 && toto>400)
    do_print = TRUE;
    else
    do_print = FALSE;
#endif

#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
      PRINT("block (%d,%d) ref picture\n",x,y);
      mat_printf_8x8_inside_picture(ref_picture,x,y,linesize);
   }
#endif

  // make motion compensation
  while (nb_part--)
  {
    //TODO: this chunk of code is not complete, only 16x16 partition are supported
    p264_inter_mc_chroma (*part++, *mv++, ref_picture , picture_out, x, y,picture_width, picture_height, linesize);
  }

#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
      PRINT("block (%d,%d) after motion compensation - first mv [%d,%d]\n",x,y,(*(mv-1)).x,(*(mv-1)).y);
      mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif

  // perform ihadamard on DC chroma
  p264_hadamard_2x2 (DC, ihadamard_chroma_2x2);

#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT ("raw chroma DC[%d %d; %d %d]\n",DC[0],DC[1],DC[2],DC[3]);
   }
#endif
  // scale DC chroma coeff
  p264_2x2_chromaDC_scale(ihadamard_chroma_2x2,ihadamard_chroma_2x2,qp);

  //// block 4x4 (0,0)
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[0];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x,y, linesize);
#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  //// block 4x4 (4,0)
  AC += BLOCK_SIZE2;
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[1];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x+4,y, linesize);
#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  //// block 4x4 (0,4)
  AC += BLOCK_SIZE2;
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[2];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x,y+4, linesize);
#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
  //// block 4x4 (4,4)
  AC += BLOCK_SIZE2;
  // zagzig
  zagzig_4x4 (AC,tmp_block_4x4);
  // scale residual block
  p264_4x4_residual_scale(tmp_block_4x4,tmp_block_4x4,qp);
  // insert DC coeff
  *tmp_block_4x4 = ihadamard_chroma_2x2[3];
  // itransform residual chroma
  p264_inverse_4x4(tmp_block_4x4,itransform_block_4x4);
  // merge residual data and intra prediction
  p264_merge_4x4 (itransform_block_4x4,picture_out,x+4,y+4, linesize);
#ifdef VP264_PCHROMA_DEBUG
   if (do_print == TRUE)
   {
     PRINT("frame after block 4x4 merging\n");
     mat_printf_8x8_inside_picture(picture_out,x,y,linesize);
   }
#endif
}


////////////// Debug functions ////////////////

void mat_printf_2x2(int16_t * mat)
{
  uint32_t i,j;
  PRINT("|------------------|\n");
  for (j=0;j<2;j++)
  {
    PRINT ("|");
    for (i=0;i<2;i++)
    {
      PRINT("%8d ",mat[i+(j<<1)]);
    }
    PRINT ("|\n");
  }
  PRINT("|------------------|\n");
}

void mat_printf_4x4(int16_t * mat)
{
  uint32_t i,j;
  PRINT("|------------------------------------|\n");
  for (j=0;j<4;j++)
  {
    PRINT ("|");
    for (i=0;i<4;i++)
    {
      PRINT("%8d ",mat[i+(j<<2)]);
    }
    PRINT ("|\n");
  }
  PRINT("|------------------------------------|\n");
}

void mat_printf_16x16_inside_picture(uint8_t * picture,uint32_t x,uint32_t y, uint32_t linesize)
{
  uint32_t i,j;
  uint8_t pixel;
  PRINT("|------------------------------------|\n");
  picture += y*linesize+x;
  for (j=0;j<16;j++)
  {
    PRINT ("|");
    for (i=0;i<16;i++)
    {
      pixel = *picture++;
      PRINT("%4d ",pixel);
    }
    picture += linesize-16;
    PRINT ("|\n");
  }
  PRINT("|------------------------------------|\n");
}

void mat_printf_8x8_inside_picture(uint8_t * picture,uint32_t x,uint32_t y, uint32_t linesize)
{
  uint32_t i,j;
  uint8_t pixel;
  PRINT("|------------------------------------|\n");
  picture += y*linesize+x;
  for (j=0;j<8;j++)
  {
    PRINT ("|");
    for (i=0;i<8;i++)
    {
      pixel = *picture++;
      PRINT("%4d ",pixel);
    }
    picture += linesize-8;
    PRINT ("|\n");
  }
  PRINT("|------------------------------------|\n");
}

void print_MB_DCT(MB_p264_t* mb_intra,intra_type_t intra_4x4)
{
  if (intra_4x4 == INTRA_4x4)
  {
    PRINT("AC coeff\n");
    uint32_t intra_4x4_index,block_i,block_j;
    for (intra_4x4_index=0;intra_4x4_index<16;intra_4x4_index++)
    {
      for (block_j=0;block_j<4;block_j++)
      {
        PRINT("\n");
        for (block_i=0;block_i<4;block_i++)
        {
           PRINT("%8d ",mb_intra->intra_4x4.AC_Y[intra_4x4_index*16+4*block_j+block_i]);
        }
      }
      PRINT("\n--------------");
    }
    PRINT("\n");
    PRINT("DC U coeff\n");
    PRINT ("%8d ",mb_intra->intra_4x4.DC_U[0]);
    PRINT ("%8d ",mb_intra->intra_4x4.DC_U[1]);
    PRINT ("%8d ",mb_intra->intra_4x4.DC_U[2]);
    PRINT ("%8d \n",mb_intra->intra_4x4.DC_U[3]);


    PRINT("DC V coeff\n");
    PRINT ("%8d ",mb_intra->intra_4x4.DC_V[0]);
    PRINT ("%8d ",mb_intra->intra_4x4.DC_V[1]);
    PRINT ("%8d ",mb_intra->intra_4x4.DC_V[2]);
    PRINT ("%8d \n",mb_intra->intra_4x4.DC_V[3]);

    PRINT("AC U coeff\n");
    for (intra_4x4_index=0;intra_4x4_index<4;intra_4x4_index++)
    {
      for (block_j=0;block_j<4;block_j++)
      {
        PRINT("\n");
        for (block_i=0;block_i<4;block_i++)
        {
           PRINT("%8d ",mb_intra->intra_4x4.AC_U[intra_4x4_index*16+4*block_j+block_i]);
        }
      }
      PRINT("\n--------------");
    }
    PRINT("AC V coeff\n");
    for (intra_4x4_index=0;intra_4x4_index<4;intra_4x4_index++)
    {
      for (block_j=0;block_j<4;block_j++)
      {
        PRINT("\n");
        for (block_i=0;block_i<4;block_i++)
        {
           PRINT("%8d ",mb_intra->intra_4x4.AC_V[intra_4x4_index*16+4*block_j+block_i]);
        }
      }
      PRINT("\n--------------");
    }
  PRINT("\n");

  }
  else
  {
    uint32_t intra_16x16_index,block_i,block_j;
    PRINT("intra 16x16 type\n");
    PRINT("DC Y\n");
    for (intra_16x16_index=0;intra_16x16_index<16;intra_16x16_index++)
      PRINT ("%8d ",mb_intra->intra_16x16.DC_Y[intra_16x16_index]);
    PRINT("\n");
    PRINT("AC Y\n");
    for (intra_16x16_index=0;intra_16x16_index<16;intra_16x16_index++)
    {
      for (block_j=0;block_j<4;block_j++)
      {
        PRINT("\n");
        for (block_i=0;block_i<4;block_i++)
        {
           PRINT("%8d ",mb_intra->intra_16x16.AC_Y[intra_16x16_index*16+4*block_j+block_i]);
        }
      }
      PRINT("\n--------------");
    }
    PRINT("\n");


    PRINT("DC U coeff\n");
    PRINT ("%8d ",mb_intra->intra_16x16.DC_U[0]);
    PRINT ("%8d ",mb_intra->intra_16x16.DC_U[1]);
    PRINT ("%8d ",mb_intra->intra_16x16.DC_U[2]);
    PRINT ("%8d \n",mb_intra->intra_16x16.DC_U[3]);


    PRINT("DC V coeff\n");
    PRINT ("%8d ",mb_intra->intra_16x16.DC_V[0]);
    PRINT ("%8d ",mb_intra->intra_16x16.DC_V[1]);
    PRINT ("%8d ",mb_intra->intra_16x16.DC_V[2]);
    PRINT ("%8d \n",mb_intra->intra_16x16.DC_V[3]);

    PRINT("AC U coeff\n");
    for (intra_16x16_index=0;intra_16x16_index<4;intra_16x16_index++)
    {
      for (block_j=0;block_j<4;block_j++)
      {
        PRINT("\n");
        for (block_i=0;block_i<4;block_i++)
        {
           PRINT("%8d ",mb_intra->intra_16x16.AC_U[intra_16x16_index*16+4*block_j+block_i]);
        }
      }
      PRINT("\n--------------");
    }
    PRINT("AC V coeff\n");
    for (intra_16x16_index=0;intra_16x16_index<4;intra_16x16_index++)
    {
      for (block_j=0;block_j<4;block_j++)
      {
        PRINT("\n");
        for (block_i=0;block_i<4;block_i++)
        {
           PRINT("%8d ",mb_intra->intra_16x16.AC_V[intra_16x16_index*16+4*block_j+block_i]);
        }
      }
      PRINT("\n--------------");
    }
  PRINT("\n");

  }
}
