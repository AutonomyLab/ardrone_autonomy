#include <VLIB/video_controller.h>
#include <VLIB/video_packetizer.h>
#include <VLIB/Platform/video_utils.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>

#include "p264_codec.h"
#include "p264_layers.h"
#include "p264.h"
#include "p264_common.h"

#include <VP_Os/vp_os_print.h>

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

// macro helper to read an intra 4x4 mode using prediction
#define READ_INTRA_CODE(pred,dest_intra) \
    code = 0; \
    video_read_data (stream,&code,1); \
    if (code==0) \
    { \
      dest_intra = pred; \
    } \
    else \
    { \
      code = 0; \
      video_read_data (stream,&code,3); \
      if (code<pred) \
        dest_intra = code; \
      else \
        dest_intra = code+1; \
    }

// make a intra 4x4 prediction based on the upper and left 4x4 block intra mode (see p264 standard)
intra_4x4_mode_t make_boundary_pred(intra_4x4_mode_t* left_intra_4x4, uint32_t left_index, intra_4x4_mode_t * up_intra_4x4, uint32_t up_index,bool_t boundary_block)
{
  intra_4x4_mode_t pred;
  intra_4x4_mode_t left_mode,up_mode;
  if (boundary_block == TRUE)
    pred = DC_4x4_MODE;
  else
  {
    if (left_intra_4x4 == NULL)
      left_mode = DC_4x4_MODE;
    else
      left_mode = left_intra_4x4[left_index];

    if (up_intra_4x4 == NULL)
      up_mode = DC_4x4_MODE;
    else
      up_mode = up_intra_4x4[up_index];

    pred = min(left_mode,up_mode);
  }

  return pred;
}

// write 16 intra_4x4 mode. Data should contains prediction on intra_4x4 (see p264 standard)
// example : data = [ pred (1bits) | code (3bits) ]
// if pred == 0 the prediction made over up and left block is correct. Thus no intra code has to bee transmitted
// if pred != 1, the prediction is wrong. The intra 4x4 mode should be transmitted using only 3 bits (see p264 standard)
void p264_write_intra_4x4 (video_stream_t* const stream, intra_4x4_mode_t* data)
{
    uint32_t i=16;
    while(i--)
    {
      if (((*data)&0x08) == 0)
      {
        // the prediction is good, signal it
        video_write_data (stream,0,1);
      }
      else
      {
        // the prediction is false, transmit the intra 4x4 mode
        video_write_data (stream,*data,4);
      }
      data++;
    }
}

// TODO: this function should be moved to a dedicated file
void p264_read_mv(video_stream_t* const stream, video_gob_t*  gobs, uint32_t i_blockline, uint32_t i_mb, uint32_t num_mb_per_line)
{
// this function reads a 16x16 mvp from a stream and reconstructs the correct mv
// it is assumed only 16x16 partition are used
//  |D|B|C|
//  |A|E
// E is the current mb
//
// case 1 : A is unavailable, use {0,0} as its mv (A becomes available)
// case 2 : only A is available, use its mv as the prediction
// case 3 : C is unavailable, use C=D instead (C becomes available)
// case 4 : A,B,C are available
// for case 1,3,4 the prediction is the median(A,B,C)

// therefore the MB types over a P-frame are
//
//  --- ---       --- ---
// |1&2| 2 | ... | 2 | 2 |
//  --- ---       --- ---
// | 1 | 4 | ... | 4 | 3 |
//  --- ---       --- ---
// |      ...            |
//  --- ---       --- ---

  int32_t s_code;
  int32_t pred_x,pred_y;
  int32_t ax,ay,bx,by,cx,cy;

  if (i_blockline == 0 && i_mb == 0)
  {
    // first MB (case 1&2)
    pred_x = 0;
    pred_y = 0;
  }
  else if (i_blockline == 0)
  {
    // first MB line (case 2)
    pred_x = gobs[i_blockline].macroblocks[i_mb-1].inter_MV[0].x;
    pred_y = gobs[i_blockline].macroblocks[i_mb-1].inter_MV[0].y;
  }
  else if (i_mb == 0)
  {
    // first MB column (case 1)
    ax = 0;
    ay = 0;
    bx = gobs[i_blockline-1].macroblocks[i_mb].inter_MV[0].x;
    by = gobs[i_blockline-1].macroblocks[i_mb].inter_MV[0].y;
    cx = gobs[i_blockline-1].macroblocks[i_mb+1].inter_MV[0].x;
    cy = gobs[i_blockline-1].macroblocks[i_mb+1].inter_MV[0].y;
    pred_x = ax + bx + cx - min(ax,min(bx,cx)) - max(ax,max(bx,cx));
    pred_y = ay + by + cy - min(ay,min(by,cy)) - max(ay,max(by,cy));
  }
  else if (i_mb == (num_mb_per_line-1))
  {
    // last column (case 3)
    ax = gobs[i_blockline].macroblocks[i_mb-1].inter_MV[0].x;
    ay = gobs[i_blockline].macroblocks[i_mb-1].inter_MV[0].y;
    bx = gobs[i_blockline-1].macroblocks[i_mb].inter_MV[0].x;
    by = gobs[i_blockline-1].macroblocks[i_mb].inter_MV[0].y;
    cx = gobs[i_blockline-1].macroblocks[i_mb-1].inter_MV[0].x; // in fact D
    cy = gobs[i_blockline-1].macroblocks[i_mb-1].inter_MV[0].y; // in fact D
    pred_x = ax + bx + cx - min(ax,min(bx,cx)) - max(ax,max(bx,cx));
    pred_y = ay + by + cy - min(ay,min(by,cy)) - max(ay,max(by,cy));
  }
  else
  {
    // others MB (case 4)
    ax = gobs[i_blockline].macroblocks[i_mb-1].inter_MV[0].x;
    ay = gobs[i_blockline].macroblocks[i_mb-1].inter_MV[0].y;
    bx = gobs[i_blockline-1].macroblocks[i_mb].inter_MV[0].x;
    by = gobs[i_blockline-1].macroblocks[i_mb].inter_MV[0].y;
    cx = gobs[i_blockline-1].macroblocks[i_mb+1].inter_MV[0].x;
    cy = gobs[i_blockline-1].macroblocks[i_mb+1].inter_MV[0].y;
    pred_x = ax + bx + cx - min(ax,min(bx,cx)) - max(ax,max(bx,cx));
    pred_y = ay + by + cy - min(ay,min(by,cy)) - max(ay,max(by,cy));
  }

  // read mv, add prediction and save
  s_code=0;
  p264_decode_int(stream, &s_code);
  gobs[i_blockline].macroblocks[i_mb].inter_MV[0].x = s_code+pred_x;
  s_code=0;
  p264_decode_int(stream, &s_code);
  gobs[i_blockline].macroblocks[i_mb].inter_MV[0].y = s_code+pred_y;
}

// TODO: this function should be moved to a dedicated file
// read 16 intra_4x4 mode transmitted with intra prediction (see p264 standard)
void p264_read_intra_4x4 (video_stream_t* const stream, video_gob_t*  gobs, uint32_t i_blockline, uint32_t i_mb)
{
    intra_4x4_mode_t* current_intra_4x4;
    intra_4x4_mode_t* up_intra_4x4=NULL;
    intra_4x4_mode_t* left_intra_4x4=NULL;
    bool_t left_boundary_block = FALSE;
    bool_t up_boundary_block = FALSE;
    video_macroblock_t * mb;
    intra_4x4_mode_t pred = DC_4x4_MODE;
    uint32_t code;

    // retrieve current mb, up and left intra 4x4 prediction if available
    current_intra_4x4 = gobs[i_blockline].macroblocks[i_mb].intra_4x4_mode;
    if (i_blockline > 0)
    {
      mb = &gobs[i_blockline-1].macroblocks[i_mb];
      up_intra_4x4 = mb->intra_4x4_mode;
      // check whether it's an intra 4x4 mb or not
      if (mb->intra_type != INTRA_4x4)
      {
        up_intra_4x4 = NULL;
      }
    }
    else
      up_boundary_block = TRUE;

    if (i_mb > 0)
    {
      mb = &gobs[i_blockline].macroblocks[i_mb-1];
      // check whether it's an intra 4x4 mb or not
      left_intra_4x4 = mb->intra_4x4_mode;
      if (mb->intra_type != INTRA_4x4)
      {
        left_intra_4x4 = NULL;
      }
    }
    else
      left_boundary_block = TRUE;

  // read intra(0,0)
  pred = make_boundary_pred(left_intra_4x4,5,up_intra_4x4,10,up_boundary_block|left_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[0]);
  // read intra(1,0)
  pred = make_boundary_pred(current_intra_4x4,0,up_intra_4x4,11,up_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[1]);
  // read intra(0,1)
  pred = make_boundary_pred(left_intra_4x4,7,current_intra_4x4,0,left_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[2]);
  // read intra(1,1)
  pred = min(current_intra_4x4[1],current_intra_4x4[2]);
  READ_INTRA_CODE(pred,current_intra_4x4[3]);

  // read intra(2,0)
  pred = make_boundary_pred(current_intra_4x4,1,up_intra_4x4,14,up_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[4]);
  // read intra(3,0)
  pred = make_boundary_pred(current_intra_4x4,4,up_intra_4x4,15,up_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[5]);
  // read intra(2,1)
  pred = min(current_intra_4x4[4],current_intra_4x4[3]);
  READ_INTRA_CODE(pred,current_intra_4x4[6]);
  // read intra(3,1)
  pred = min(current_intra_4x4[5],current_intra_4x4[6]);
  READ_INTRA_CODE(pred,current_intra_4x4[7]);

  // read intra(0,2)
  pred = make_boundary_pred(left_intra_4x4,13,current_intra_4x4,2,left_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[8]);
  // read intra(1,2)
  pred = min(current_intra_4x4[8],current_intra_4x4[3]);
  READ_INTRA_CODE(pred,current_intra_4x4[9]);
  // read intra(0,3)
  pred = make_boundary_pred(left_intra_4x4,15,current_intra_4x4,8,left_boundary_block);
  READ_INTRA_CODE(pred,current_intra_4x4[10]);
  // read intra(1,3)
  pred = min(current_intra_4x4[9],current_intra_4x4[10]);
  READ_INTRA_CODE(pred,current_intra_4x4[11]);

  // read intra(2,2)
  pred = min(current_intra_4x4[9],current_intra_4x4[6]);
  READ_INTRA_CODE(pred,current_intra_4x4[12]);
  // read intra(3,2)
  pred = min(current_intra_4x4[12],current_intra_4x4[7]);
  READ_INTRA_CODE(pred,current_intra_4x4[13]);
    // read intra(2,3)
  pred = min(current_intra_4x4[11],current_intra_4x4[12]);
  READ_INTRA_CODE(pred,current_intra_4x4[14]);
    // read intra(3,3)
  pred = min(current_intra_4x4[13],current_intra_4x4[14]);
  READ_INTRA_CODE(pred,current_intra_4x4[15]);

}

void p264_write_block( video_stream_t* const stream, int16_t* data, uint32_t length)
{
  int32_t code, run, num_coeff;

  // count number of DC coeff
  num_coeff = 0;
  int16_t * p_data = data;

  while (length--)
  {
    if (*p_data++ != 0)
      num_coeff++;
  }

  if (num_coeff == 0)
  {
    video_write_data( stream, 1, 1 ); // signal that there's no coeff
  }
  else
  {
    video_write_data( stream, 0, 1 ); // signal that there are coeffs

    run = 0;
    while( num_coeff > 0 )
    {
      code = *data++;
      if( code == 0 )
      {
        run ++;
      }
      else
      {
        num_coeff--;
        p264_encode( stream, code, run, num_coeff );
        run = 0;
      }
    }
  }
}

void p264_read_block( video_stream_t* const stream, int16_t* data)
{
  int32_t  index,  run, last;//, nc;
  uint32_t code;

  code = run = last = 0;
  video_read_data( stream, &code, 1 ); // signal that there's no DC coeff

  if(code == 0)
  {
    index = -1;
    while( last == 0 )
    {
      code = run = last = 0;
      p264_decode( stream, &run, &code, &last);

      if( last == 0 )
      {
        index    += (run+1);
        data[index] = code;
      }
    }
  }
}

C_RESULT p264_write_mb_layer(video_controller_t* controller, video_stream_t* stream, video_macroblock_t* mb, int32_t num_macro_blocks )
{
  int16_t* data;
  //uint32_t code;
  uint32_t i;


  while( num_macro_blocks > 0 )
  {
    //PRINT ("p264_write_mb_layer : stream size %d  stream used %d\n",stream->size,stream->used);

    if (controller->picture_type == VIDEO_PICTURE_INTER)
    {
      // write all partition
      for (i=0;i<mb->nb_partition;i++)
      {
        video_write_data( stream,mb->inter_partition_mode[i],3);
      }

      // write all motion vector
      for (i=0;i<mb->nb_partition;i++)
      {
        int32_t s_code;
        s_code = (int32_t)mb->inter_MV[i].x;
        p264_encode_int(stream, s_code);
        s_code = (int32_t)mb->inter_MV[i].y;
        p264_encode_int(stream, s_code);
      }

      // write residual data as an intra4x4
      // write all 4x4 block luma AC coeff
      i=16;
      data = ((MB_p264_t*)mb->data)->inter.AC_Y;
      while(i--)
      {
        p264_write_block( stream, data, 16);
        data += BLOCK_SIZE2;
      }

      // write 4 DC U coeff
      p264_write_block(stream,((MB_p264_t*)mb->data)->inter.DC_U,4);

      // write AC U coeff
      i=4;
      data = &((MB_p264_t*)mb->data)->inter.AC_U[1];
      while(i--)
      {
        p264_write_block( stream, data, 15);
        data += BLOCK_SIZE2;
      }

      // write 4 DC V coeff
      p264_write_block(stream,((MB_p264_t*)mb->data)->inter.DC_V,4);

      // write AC V coeff
      i=4;
      data = &((MB_p264_t*)mb->data)->inter.AC_V[1];
      while(i--)
      {
        p264_write_block( stream, data, 15);
        data += BLOCK_SIZE2;
      }
    }
    else
    {
      // write MB intra type (16x16 or 4x4)
      video_write_data( stream, mb->intra_type, 1 );
      // write intra chroma type
      video_write_data (stream, mb->intra_chroma_8x8_mode,2);
      if (mb->intra_type == INTRA_4x4)
      {
        // write all luma 4x4 prediction modes
        p264_write_intra_4x4(stream,mb->intra_4x4_mode);

        // write all 4x4 block luma AC coeff
        i=16;
        data = ((MB_p264_t*)mb->data)->intra_4x4.AC_Y;
        while(i--)
        {
          p264_write_block( stream, data, 16);
          data += BLOCK_SIZE2;
        }

        // write 4 DC U coeff
        p264_write_block(stream,((MB_p264_t*)mb->data)->intra_4x4.DC_U,4);

        // write AC U coeff
        i=4;
        data = &((MB_p264_t*)mb->data)->intra_4x4.AC_U[1];
        while(i--)
        {
          p264_write_block( stream, data, 15);
          data += BLOCK_SIZE2;
        }

        // write 4 DC V coeff
        p264_write_block(stream,((MB_p264_t*)mb->data)->intra_4x4.DC_V,4);

        // write AC V coeff
        i=4;
        data = &((MB_p264_t*)mb->data)->intra_4x4.AC_V[1];
        while(i--)
        {
          p264_write_block( stream, data, 15);
          data += BLOCK_SIZE2;
        }
      }
      else
      {
        // write luma 16x16 prediction mode
        video_write_data(stream, mb->intra_luma_16x16_mode,2);

        // write 16 DC luma
        p264_write_block( stream, ((MB_p264_t*)mb->data)->intra_16x16.DC_Y, 16);

        // write 16 luma AC coeff block
        i=16;
        data = &((MB_p264_t*)mb->data)->intra_16x16.AC_Y[1];
        while(i--)
        {
          p264_write_block( stream, data, 15);
          data += BLOCK_SIZE2;
        }

        // write 4 DC U coeff
        p264_write_block(stream,((MB_p264_t*)mb->data)->intra_16x16.DC_U,4);

        // write AC U coeff
        i=4;
        data = &((MB_p264_t*)mb->data)->intra_16x16.AC_U[1];
        while(i--)
        {
          p264_write_block( stream, data, 15);
          data += BLOCK_SIZE2;
        }

        // write 4 DC V coeff
        p264_write_block(stream,((MB_p264_t*)mb->data)->intra_16x16.DC_V,4);

        // write AC V coeff
        i=4;
        data = &((MB_p264_t*)mb->data)->intra_16x16.AC_V[1];
        while(i--)
        {
          p264_write_block( stream, data, 15);
          data += BLOCK_SIZE2;
        }
      }
    }
    mb ++;
    num_macro_blocks --;
  }
  return C_OK;
}


C_RESULT p264_read_mb_layer(video_controller_t* controller, video_stream_t* stream, video_gob_t*  gobs, uint32_t i_blockline, uint32_t i_mb)
{
  int16_t* data;
  uint32_t code;
  //int8_t mv;
  uint32_t i;
  video_macroblock_t* mb;
  mb = &gobs[i_blockline].macroblocks[i_mb];

  vp_os_memset( mb->data, 0, sizeof(MB_p264_t));

  if (controller->picture_type == VIDEO_PICTURE_INTER)
  {
    // for now p264 supports only one partition per macroblock
    mb->nb_partition = 1;
    // read all partition
    for (i=0;i<mb->nb_partition;i++)
    {
      code = 0;
      video_read_data( stream,&code,3);
      mb->inter_partition_mode[i] = code;
    }

    // read all motion vector
    p264_read_mv(stream, gobs, i_blockline, i_mb,controller->mb_blockline);

    // read residual data as an intra 4x4
    // read 16 4x4 block luma AC coeff
    i=16;
    data = ((MB_p264_t*)mb->data)->inter.AC_Y;
    while(i--)
    {
      p264_read_block( stream, data);
      data += BLOCK_SIZE2;
    }

    // read 4 DC U coeff
    p264_read_block(stream,((MB_p264_t*)mb->data)->inter.DC_U);

    // read AC U coeff
    i=4;
    data = &((MB_p264_t*)mb->data)->inter.AC_U[1];
    while(i--)
    {
      p264_read_block( stream, data);
      data += BLOCK_SIZE2;
    }

    // read 4 DC V coeff
    p264_read_block(stream,((MB_p264_t*)mb->data)->inter.DC_V);

    // read AC V coeff
    i=4;
    data = &((MB_p264_t*)mb->data)->inter.AC_V[1];
    while(i--)
    {
      p264_read_block( stream, data);
      data += BLOCK_SIZE2;
    }
  }
  else
  {
    // read MB intra type
    code = 0;
    video_read_data(stream, &code, 1);
    mb->intra_type = (intra_type_t)code;

    // read intra chroma type
    code = 0;
    video_read_data(stream, &code, 2);
    mb->intra_chroma_8x8_mode = (intra_8x8_chroma_mode_t) code;

    if (mb->intra_type == INTRA_4x4)
    {
      // read all 4x4 modes
      p264_read_intra_4x4(stream, gobs, i_blockline, i_mb);

      // read 16 4x4 block luma AC coeff
      i=16;
      data = ((MB_p264_t*)mb->data)->intra_4x4.AC_Y;
      while(i--)
      {
        p264_read_block( stream, data);
        data += BLOCK_SIZE2;
      }

      // read 4 DC U coeff
      p264_read_block(stream,((MB_p264_t*)mb->data)->intra_4x4.DC_U);

      // read AC U coeff
      i=4;
      data = &((MB_p264_t*)mb->data)->intra_4x4.AC_U[1];
      while(i--)
      {
        p264_read_block( stream, data);
        data += BLOCK_SIZE2;
      }

      // read 4 DC V coeff
      p264_read_block(stream,((MB_p264_t*)mb->data)->intra_4x4.DC_V);

      // read AC V coeff
      i=4;
      data = &((MB_p264_t*)mb->data)->intra_4x4.AC_V[1];
      while(i--)
      {
        p264_read_block( stream, data);
        data += BLOCK_SIZE2;
      }
    }
    else
    {
      // write luma 16x16 prediction mode
      code = 0;
      video_read_data(stream, &code ,2);
      mb->intra_luma_16x16_mode = code;

      // read 16 DC luma
      p264_read_block( stream, ((MB_p264_t*)mb->data)->intra_16x16.DC_Y);

      // write 256 luma AC coeff
      // Normally only 15 AC coeff per block 4x4 has to be sent
      i=16;
      data = &((MB_p264_t*)mb->data)->intra_16x16.AC_Y[1];
      while(i--)
      {
        p264_read_block( stream, data);
        data += BLOCK_SIZE2;
      }

      // read 4 DC U coeff
      p264_read_block(stream,((MB_p264_t*)mb->data)->intra_16x16.DC_U);

      // read AC U coeff
      i=4;
      data = &((MB_p264_t*)mb->data)->intra_16x16.AC_U[1];
      while(i--)
      {
        p264_read_block( stream, data);
        data += BLOCK_SIZE2;
      }

      // read 4 DC V coeff
      p264_read_block(stream,((MB_p264_t*)mb->data)->intra_16x16.DC_V);

      // read AC V coeff
      i=4;
      data = &((MB_p264_t*)mb->data)->intra_16x16.AC_V[1];
      while(i--)
      {
        p264_read_block( stream, data);
        data += BLOCK_SIZE2;
      }
    }
  }
  return C_OK;
}

