#include <VLIB/P264/p264_intra_pred.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

C_RESULT p264_intra_4x4_luma (intra_4x4_mode_t mode, uint8_t *picture, uint32_t picture_width, uint32_t x, uint32_t y, uint32_t linesize)
// all mode partially validated (not all availability cases for each mode)
{
// A-M are the boundary pixels of the 4x4 block a-p
//
// M A B C D E F G H
// I a b c d
// J e f g h
// K i j k l
// L m n o p
//
// a-p = Pixel(x,y)
  C_RESULT res = C_OK;
	uint32_t A,B,C,D,E,F,G,H,I,J,K,L,M;
	uint32_t IJKL;
  uint32_t i;
  uint32_t availability_flag=0;
  uint8_t * picture_start;

  // determine block availability
  if (x==0)
  {
    // block 4x4 is located in the first column of the frame, A pixels are not available
    availability_flag |= A_UNAVAILABLE;
  }
  if (y==0)
  {
    // block 4x4 is located in the first row of the frame, B and C  pixels are not available
    availability_flag |= B_UNAVAILABLE | C_UNAVAILABLE;
  }
  if (x==picture_width-4)
  {
    // block 4x4 is located in the last column of the frame, C pixels are not available
    availability_flag |= C_UNAVAILABLE;
  }
  if ((((x>>2)&0x01) == 1) &&  (((y>>2)&0x01) == 1))
  {
    // block 4x4 is located at the 'x' position in the following MB representation:
    // - - - -
    // - x - x
    // - - - -
    // - x - x
    // therefore C is not available.
    availability_flag |= C_UNAVAILABLE;
  }
  else if ((((x>>2)&0x03) == 3) &&  (((y>>2)&0x03) == 2))
  {
    // block 4x4 is located at the 'x' position in the following MB representation:
    // - - - -
    // - - - -
    // - - - x
    // - - - -
    // therefore C is not available.
    availability_flag |= C_UNAVAILABLE;
  }

  // check availability and mode
  switch (mode)
  {
    case VERTICAL_4x4_MODE :
    case DIAGONAL_DL_4x4_MODE :
    case VERTICAL_LEFT_4x4_MODE :
      if (availability_flag & B_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 4x4 mode_%d prediction on (%d,%d), B unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    case HORIZONTAL_4x4_MODE :
    case HORIZONTAL_UP_4x4_MODE :
      if (availability_flag & A_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 4x4 mode_%d prediction on (%d,%d) A unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    case DC_4x4_MODE :
      // no restriction
      break;

    case DIAGONAL_DR_4x4_MODE :
    case VERTICAL_RIGHT_4x4_MODE :
    case HORIZONTAL_DOWN_4x4_MODE :
      if (availability_flag & A_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 4x4 mode_%d prediction on (%d,%d) A unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      if (availability_flag & B_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 4x4 mode_%d prediction on (%d,%d) B unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    default :
      PRINT ("%s : mode_%d at block (%d,%d) is not a valid intra 4x4 prediction mode\n",__FUNCTION__,mode,x,y);
      res = C_FAIL;
  }

  if (SUCCEED(res))
  {
    // make prediction
    switch (mode)
    {
      case VERTICAL_4x4_MODE :
        // jump to pixel A
        picture += (linesize*(y-1)+x);
        // load A-D pixel values
        A = *picture++;
        B = *picture++;
        C = *picture++;
        D = *picture++;
        // extend ABCD line
        for (i=0;i<4;i++)
        {
          // jump to next line
          picture += (linesize-4);
          // extend
          *picture++ = A;
          *picture++ = B;
          *picture++ = C;
          *picture++ = D;
        }
        break;

      case HORIZONTAL_4x4_MODE :
        // jump to pixel I
        picture += (linesize*y+x-1);
        for (i=0; i<4; i++)
        {
          // extend I-L pixels to a-d,e-h,i-l,m-p;
          IJKL = *picture++;
          *picture++ = IJKL;
          *picture++ = IJKL;
          *picture++ = IJKL;
          *picture   = IJKL;
          // jump to next row (pixel J,K,L)
          picture += (linesize-4);
        }
        break;

      case DC_4x4_MODE :
        picture_start = picture;
        uint32_t mean=0;
        // compute mean over A-D and I-L pixels if available
        if (!(availability_flag & B_UNAVAILABLE))
        {
          //jump to pixel A
          picture += (linesize*(y-1)+x);
          // sum pixel A-D
          mean += *picture++;
          mean += *picture++;
          mean += *picture++;
          mean += *picture;
        }
        if (!(availability_flag & A_UNAVAILABLE))
        {
          picture = picture_start;
          //jump to pixel I
          picture += (linesize*y+x-1);
          for (i=0;i<4;i++)
          {
            mean += *picture;
            //jump to next pixel
            picture += linesize;
          }
        }

        if (!(availability_flag & A_UNAVAILABLE) && !(availability_flag & B_UNAVAILABLE))
          mean = (mean+4)>>3;
        else if (!(availability_flag & A_UNAVAILABLE) || !(availability_flag & B_UNAVAILABLE))
          mean = (mean+2)>>2;
        else
          mean=128;

        // jump to pixel a and make prediction
        picture = picture_start+linesize*y+x;
        for (i=0;i<4;i++)
        {
          *picture++ = mean;
          *picture++ = mean;
          *picture++ = mean;
          *picture++ = mean;
          // jump to next line
          picture = picture+linesize-4;
        }
        break;

      case DIAGONAL_DL_4x4_MODE :
        picture_start = picture;
        // jump to pixel A
        picture += (linesize*(y-1)+x);
        // load A-D pixel values
        A = *picture++;
        B = *picture++;
        C = *picture++;
        D = *picture++;
        // check block C availability
        if (availability_flag & C_UNAVAILABLE)
        {
          // E,F,G,H not available, copy D value
          E = F = G = H = D;
        }
        else
        {
          // load E-H pixel values
          E = *picture++;
          F = *picture++;
          G = *picture++;
          H = *picture++;
        }

        // jump to pixel a
        picture = picture_start + linesize*y+x;

         // make prediction
        *picture++ = (A + C + 2*(B) + 2) >> 2; // a
        *picture++ = (B + D + 2*(C) + 2) >> 2; // b
        *picture++ = (C + E + 2*(D) + 2) >> 2; // c
        *picture   = (D + F + 2*(E) + 2) >> 2; // d
        picture += linesize-3;
        *picture++ = (B + D + 2*(C) + 2) >> 2; // e
        *picture++ = (C + E + 2*(D) + 2) >> 2; // f
        *picture++ = (D + F + 2*(E) + 2) >> 2; // g
        *picture   = (E + G + 2*(F) + 2) >> 2; // h
        picture += linesize-3;
        *picture++ = (C + E + 2*(D) + 2) >> 2; // i
        *picture++ = (D + F + 2*(E) + 2) >> 2; // j
        *picture++ = (E + G + 2*(F) + 2) >> 2; // k
        *picture   = (F + H + 2*(G) + 2) >> 2; // l
        picture += linesize-3;
        *picture++ = (D + F + 2*(E) + 2) >> 2; // m
        *picture++ = (E + G + 2*(F) + 2) >> 2; // n
        *picture++ = (F + H + 2*(G) + 2) >> 2; // o
        *picture   = (G + 3*(H) + 2) >> 2; // p
        break;

      case DIAGONAL_DR_4x4_MODE :
        picture_start = picture;
        // jump to pixel M
        picture += (linesize*(y-1)+x-1);
        // load M-D pixel values
        M = *picture++;
        A = *picture++;
        B = *picture++;
        C = *picture++;
        D = *picture;
        // jump to pixel I & load it
        picture += linesize-4;
        I = *picture;
        // jump to pixel J & load it
        picture += linesize;
        J = *picture;
        // jump to pixel K & load it
        picture += linesize;
        K = *picture;
        // jump to pixel L & load it
        picture += linesize;
        L = *picture;
        // jump to pixel a
        picture = picture_start + linesize*y+x;
        // make prediction
        *picture++ = (I + 2*M + A + 2) >> 2; // a
        *picture++ = (M + 2*A + B + 2) >> 2; // b
        *picture++ = (A + 2*B + C + 2) >> 2; // c
        *picture   = (B + 2*C + D + 2) >> 2; // d
        picture += linesize-3;
        *picture++ = (J + 2*I + M + 2) >> 2; // e
        *picture++ = (I + 2*M + A + 2) >> 2; // f
        *picture++ = (M + 2*A + B + 2) >> 2; // g
        *picture   = (A + 2*B + C + 2) >> 2; // h
        picture += linesize-3;
        *picture++ = (K + 2*J + I + 2) >> 2; // i
        *picture++ = (J + 2*I + M + 2) >> 2; // j
        *picture++ = (I + 2*M + A + 2) >> 2; // k
        *picture   = (M + 2*A + B + 2) >> 2; // l
        picture += linesize-3;
        *picture++ = (L + 2*K + J + 2) >> 2; // m
        *picture++ = (K + 2*J + I + 2) >> 2; // n
        *picture++ = (J + 2*I + M + 2) >> 2; // o
        *picture   = (I + 2*M + A + 2) >> 2; // p
        break;

      case VERTICAL_RIGHT_4x4_MODE :
        picture_start = picture;
        // jump to pixel M
        picture += (linesize*(y-1)+x-1);
        // load M-D pixel values
        M = *picture++;
        A = *picture++;
        B = *picture++;
        C = *picture++;
        D = *picture;
        // jump to pixel I & load it
        picture += linesize-4;
        I = *picture;
        // jump to pixel J & load it
        picture += linesize;
        J = *picture;
        // jump to pixel K & load it
        picture += linesize;
        K = *picture;
        // jump to pixel L & load it
        picture += linesize;
        L = *picture;
        // jump to pixel a
        picture = picture_start + linesize*y+x;
        // make prediction
        *picture++ = (M + A + 1) >> 1; // a
        *picture++ = (A + B + 1) >> 1; // b
        *picture++ = (B + C + 1) >> 1; // c
        *picture   = (C + D + 1) >> 1; // d
        picture += linesize-3;
        *picture++ = (I + 2*M + A + 2) >> 2; // e
        *picture++ = (M + 2*A + B + 2) >> 2; // f
        *picture++ = (A + 2*B + C + 2) >> 2; // g
        *picture   = (B + 2*C + D + 2) >> 2; // h
        picture += linesize-3;
        *picture++ = (M + 2*I + J + 2) >> 2; // i
        *picture++ = (M + A + 1) >> 1; // j
        *picture++ = (A + B + 1) >> 1; // k
        *picture   = (B + C + 1) >> 1; // l
        picture += linesize-3;
        *picture++ = (I + 2*J + K + 2) >> 2; // m
        *picture++ = (I + 2*M + A + 2) >> 2; // n
        *picture++ = (M + 2*A + B + 2) >> 2; // o
        *picture   = (A + 2*B + C + 2) >> 2; // p
        break;

      case HORIZONTAL_DOWN_4x4_MODE :
        picture_start = picture;
        // jump to pixel M
        picture += (linesize*(y-1)+x-1);
        // load M-D pixel values
        M = *picture++;
        A = *picture++;
        B = *picture++;
        C = *picture++;
        D = *picture;
        // jump to pixel I & load it
        picture += linesize-4;
        I = *picture;
        // jump to pixel J & load it
        picture += linesize;
        J = *picture;
        // jump to pixel K & load it
        picture += linesize;
        K = *picture;
        // jump to pixel L & load it
        picture += linesize;
        L = *picture;
        // jump to pixel a
        picture = picture_start + linesize*y+x;
        // make prediction
        *picture++ = (M + I + 1) >> 1; // a
        *picture++ = (I + 2*M + A + 2) >> 2; // b
        *picture++ = (M + 2*A + B + 2) >> 2; // c
        *picture   = (A + 2*B + C + 2) >> 2; // d
        picture += linesize-3;
        *picture++ = (I + J + 1) >> 1; // e
        *picture++ = (M + 2*I + J + 2) >> 2; // f
        *picture++ = (M + I + 1) >> 1; // g
        *picture   = (I + 2*M + A + 2) >> 2; // h
        picture += linesize-3;
        *picture++ = (J + K + 1) >> 1; // i
        *picture++ = (I + 2*J + K + 2) >> 2; // j
        *picture++ = (I + J + 1) >> 1; // k
        *picture   = (M + 2*I + J + 2) >> 2; // l
        picture += linesize-3;
        *picture++ = (K + L + 1) >> 1; // m
        *picture++ = (J + 2*K + L + 2) >> 2; // n
        *picture++ = (J + K + 1) >> 1; // o
        *picture   = (I + 2*J + K + 2) >> 2; // p

        break;

      case VERTICAL_LEFT_4x4_MODE :
        picture_start = picture;
        // jump to pixel A
        picture += (linesize*(y-1)+x);
        // load A-D pixel values
        A = *picture++;
        B = *picture++;
        C = *picture++;
        D = *picture++;
        // check block C availability
        if (availability_flag & C_UNAVAILABLE)
        {
          // E,F,G,H not available, copy D value
          E = F = G = H = D;
        }
        else
        {
          // load E-H pixel values
          E = *picture++;
          F = *picture++;
          G = *picture++;
          H = *picture;
        }
        // jump to pixel a
        picture = picture_start + linesize*y+x;
        // make prediction
        *picture++ = (A + B + 1) >> 1; // a
        *picture++ = (B + C + 1) >> 1; // b
        *picture++ = (C + D + 1) >> 1; // c
        *picture   = (D + E + 1) >> 1; // d
        picture += linesize-3;
        *picture++ = (A + 2*B + C + 2) >> 2; // e
        *picture++ = (B + 2*C + D + 2) >> 2; // f
        *picture++ = (C + 2*D + E + 2) >> 2; // g
        *picture   = (D + 2*E + F + 2) >> 2; // h
        picture += linesize-3;
        *picture++ = (B + C + 1) >> 1; // i
        *picture++ = (C + D + 1) >> 1; // j
        *picture++ = (D + E + 1) >> 1; // k
        *picture   = (E + F + 1) >> 1; // l
        picture += linesize-3;
        *picture++ = (B + 2*C + D + 2) >> 2; // m
        *picture++ = (C + 2*D + E + 2) >> 2; // n
        *picture++ = (D + 2*E + F + 2) >> 2; // o
        *picture   = (E + 2*F + G + 2) >> 2; // p
        break;

      case HORIZONTAL_UP_4x4_MODE :
        picture_start = picture;
        // jump to pixel I & load it
        picture += (linesize*y+x-1);
        I = *picture;
        // jump to pixel J & load it
        picture += linesize;
        J = *picture;
        // jump to pixel K & load it
        picture += linesize;
        K = *picture;
        // jump to pixel L & load it
        picture += linesize;
        L = *picture;
        // jump to pixel a
        picture = picture_start + linesize*y+x;
        // make prediction
        *picture++ = (I + J + 1) >> 1; // a
        *picture++ = (I + 2*J + K + 2) >> 2; // b
        *picture++ = (J + K + 1) >> 1; // c
        *picture   = (J + 2*K + L + 2) >> 2; // d
        picture += linesize-3;
        *picture++ = (J + K + 1) >> 1; // e
        *picture++ = (J + 2*K + L + 2) >> 2; // f
        *picture++ = (K + L + 1) >> 1; // g
        *picture   = (K + 2*L + L + 2) >> 2; // h
        picture += linesize-3;
        *picture++ = (K + L + 1) >> 1; // i
        *picture++ = (K + 2*L + L + 2) >> 2; // j
        *picture++ = L; // k
        *picture   = L; // l
        picture += linesize-3;
        *picture++ = L; // m
        *picture++ = L; // n
        *picture++ = L; // o
        *picture   = L; // p
        break;

      default :
        PRINT ("%s : mode_%d at block (%d,%d) is not a valid intra 4x4 prediction mode\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
    }
  }
	return res;
}

C_RESULT p264_intra_16x16_luma (intra_16x16_luma_mode_t mode, uint8_t *picture, uint32_t x, uint32_t y, uint32_t linesize)
// Hori,Verti, Plane mode validated, DC missing
{
// Ai-Bi-M are the boundary pixels of the 16x16 block p(,)
//
//   M      B0     B1  ...    B15
//  A0  p(0,0) p(0,1)      p(0,15)
//  A1  p(1,0)
//
// ...             ...
//
// A15  p(15,0)            p(15,15)
//

  C_RESULT res = C_OK;
  uint32_t availability_flag=0;
  uint8_t * picture_start;
  uint32_t i,j;

  // determine block availability
  if (x==0)
  {
    // block 16x16 is located in the first column of the frame, A pixels are not available
    availability_flag |= A_UNAVAILABLE;
  }
  if (y==0)
  {
    // block 16x16 is located in the first row of the frame, B pixels are not available
    availability_flag |= B_UNAVAILABLE;
  }

  // check availability and mode
  switch (mode)
  {
    case VERTICAL_16x16_LUMA_MODE :
      if (availability_flag & B_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 16x16 mode_%d prediction on (%d,%d), B unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    case HORIZONTAL_16x16_LUMA_MODE :
      if (availability_flag & A_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 16x16 mode_%d prediction on (%d,%d) A unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    case DC_16x16_LUMA_MODE :
      // no restriction
      break;

    case PLANE_16x16_LUMA_MODE :
     if (availability_flag & A_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 16x16 mode_%d prediction on (%d,%d) A unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      if (availability_flag & B_UNAVAILABLE)
      {
        PRINT("%s : could no compute intra 16x16 mode_%d prediction on (%d,%d) B unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    default :
      PRINT ("%s : mode_%d is not a valid intra 16x16 prediction mode\n",__FUNCTION__,mode);
      res = C_FAIL;
  }

  if (SUCCEED(res))
  {
    // make prediction
    switch (mode)
    {
      case  DC_16x16_LUMA_MODE :
      {
        picture_start = picture;
        uint32_t mean=0;
        // compute mean over Ai and Bi pixels if available
        if (!(availability_flag & B_UNAVAILABLE))
        {
          //jump to pixel B0
          picture += (linesize*(y-1)+x);
          // sum pixel Bi
          i=16;
          while(i--)
          {
            mean += *picture++;
          }
        }

        if (!(availability_flag & A_UNAVAILABLE))
        {
          picture = picture_start;
          //jump to pixel A0
          picture += (linesize*y+x-1);
          // sum pixel Ai
          i=16;
          while(i--)
          {
            mean += *picture;
            picture += linesize;
          }
        }

        if (!(availability_flag & A_UNAVAILABLE) && !(availability_flag & B_UNAVAILABLE))
          mean = (mean+16)>>5;
        else if (!(availability_flag & A_UNAVAILABLE) || !(availability_flag & B_UNAVAILABLE))
          mean = (mean+8)>>4;
        else
          mean=128;

        // jump to pixel p(0,0) and make prediction
        picture = picture_start+linesize*y+x;
        j=16;
        while(j--)
        {
          i=16;
          while(i--)
          {
            *picture++ = mean;
          }
          picture += (linesize-16);
        }
      }
        break;

      case HORIZONTAL_16x16_LUMA_MODE :
      {
        //jump to pixel A0
        picture += (linesize*y+x-1);
        // extend pixel Ai horizontally
        uint8_t pixel_buf;
        j=16;
        while(j--)
        {
          // load Ai
          pixel_buf = *picture++;
          // extend Ai
          i=16;
          while(i--)
          {
            *picture++ = pixel_buf;
          }
          // jump to Ai+1
          picture += (linesize-17);
        }
      }
        break;

      case VERTICAL_16x16_LUMA_MODE :
      {
        // jump to pixel B0
        picture += (linesize*(y-1)+x);
        uint8_t line_buf[16];
        // load Bi line
        vp_os_memcpy(line_buf,picture,16*sizeof(uint8_t));
        // extend Bi line vertically
        i=16;
        while(i--)
        {
          // jump to next line
          picture += linesize;
          // copy line
          vp_os_memcpy(picture,line_buf,16*sizeof(uint8_t));
        }
      }
        break;

      case PLANE_16x16_LUMA_MODE :
      {
        int32_t A15,B15;
        picture_start = picture;
        // compute H :
        // H' = 1* (B8 - B6) +
        //      2* (B9 - B5) +
        //      3*(B10 - B4) +
        //      4*(B11 - B3) +
        //      5*(B12 - B2) +
        //      6*(B13 - B1) +
        //      7*(B14 - B0) +
        //      8*(B15 - M)
        //  H = (5*H' + 32) / 64
        // jump to M
        picture += (linesize*(y-1)+x-1);
        int32_t H=0;
        for (i=8;i>=1;i--)
        {
          H -= i*(*picture++);
        }
        for (i=1;i<=8;i++)
        {
          H += i*(*++picture);
        }
        B15 = *(picture);
        H = (5*H + 32)>>6;

        // compute V :
        // V' = 1* (A8 - A6) +
        //      2* (A9 - A5) +
        //      3*(A10 - A4) +
        //      4*(A11 - A3) +
        //      5*(A12 - A2) +
        //      6*(A13 - A1) +
        //      7*(A14 - A0) +
        //      8*(A15 - M)
        //  V = (5*V' + 32) / 64
        int32_t V=0;
        // jump to M
        picture = picture_start + (linesize*(y-1)+x-1);
        for (i=8;i>=1;i--)
        {
          V -= i*(*picture);
          picture += linesize;
        }
        for (i=1;i<=8;i++)
        {
          picture += linesize;
          V += i*(*picture);
        }
        A15 = *(picture);
        V = (5*V + 32)>>6;

        // fill pixel p(i,j) of the 16x16 block with the following rule :
        // a = 16 * (A15 + B15 + 1) - 7*(V+H)
        // b(i,j) = a + V * j + H * i
        // p(i,j) = SATURATE_U8(b(i,j) / 32)
        // SATURATE_U8() function indicates that the result of the operation should be bounded to an unsigned 8-bit range (0..255)

        int32_t a = ((A15 + B15 + 1)<<4) - 7*(V+H);

        // jump to p(0,0)
        picture = picture_start + (linesize*y+x);
        // make prediction
        int32_t pixel_buf;
        for (j=0;j<16;j++)
        {
          for (i=0;i<16;i++)
          {
            pixel_buf = (a + V*j + H*i)>>5;
            // saturation
            if (pixel_buf > 0xFF)
              pixel_buf = 0xFF;
            else if (pixel_buf < 0)
              pixel_buf = 0;
            *picture++ = pixel_buf;
          }
          // jump to next line
          picture += linesize - 16;
        }
      }
        break;

      default :
        PRINT ("%s : mode_%d is not a valid intra 16x16 prediction mode\n",__FUNCTION__,mode);
        res = C_FAIL;
    }
  }
  return res;
}


// DC_8x8_CHROMA_MODE is slightly different from DC_16x16_LUMA_MODE
// each 8x8 block is divided into 4 blocks
//  -------------------
// | all     | single1 |
//  --------- ---------
// | single2 | all     |
//  -------------------
//
// In "all" block, adjacent 4x4 blocks A and B are used to compute the DC value as in DC_16x16_LUMA_MODE.
//   If B is not available, A is used instead of B.
//   If A is not available, B is used instead of A.
//   If neither A nor B are available, DC is set to 128.
// In "single1" priority is given to B block to compute the DC value. If B is not available, A is used instead of B.
//   If neither A nor B are available, DC is set to 128.
// In "single2" priority is given to A block to compute the DC value. If A is not available, B is used instead of A.
//   If neither A nor B are available, DC is set to 128.
//
// Next scheme summarize the block used to compute the DC value in raster scan ordered image :
//          B blocks
//  A   -----------------
//     | x    x | A    A |
//  b  | x    x | A    A |
//  l   -------- --------  ...
//  o  | B    B | AB   B |
//  c  | B    B | A   AB |
//  k   -------- --------
//  s          ...

static uint8_t chroma_DC_all(uint8_t *picture, uint32_t x, uint32_t y, uint32_t linesize)
{
  // this function assumes that (x,y) corresponds to a "all" DC block 4x4
  uint32_t mean=0;
  uint8_t *picture_start = picture;
  if (x<=4 && y<=4)
  {
    // top left 8x8 chroma block, A&B are not available
    mean = 128;
  }
  else
  {
    if (y>4)
    {
      // 8x8 chroma block is not located in the first row
      // sum B
      picture = picture_start + linesize*((y&0xFFF8)-1) + x;
      mean += *picture++;
      mean += *picture++;
      mean += *picture++;
      mean += *picture;
    }
    if (x>4)
    {
      // 8x8 chroma block is not located in the first column
      // sum A
      picture = picture_start + linesize*y + (x&0xFFF8)-1;
      mean += *picture;
      picture += linesize;
      mean += *picture;
      picture += linesize;
      mean += *picture;
      picture += linesize;
      mean += *picture;
    }
    if (x<=4 || y<=4)
    {
      // only one adjacent block (A or B) is used to compute DC value
      mean = mean>>2;
    }
    else
    {
      // both A and B blocks are used to compute DC value
      mean = mean>>3;
    }
  }
  return mean;
}

static uint8_t chroma_DC_single(uint8_t *picture, uint32_t x, uint32_t y, uint32_t linesize)
{
  // this function assumes that (x,y) corresponds to a "single" DC block 4x4
  uint32_t mean=0;
  if (x<=4 && y<=4)
  {
    // top left 8x8 chroma block, A&B are not available
    mean = 128;
  }
  else
  {
    if ((x<=4) ||  ((y>4) && ((y&0x07) == 0)))
    {
      // block 4x4 is located either on the first chroma 8x8 column or at the top right of a 8x8 chroma
      // sum B
      picture += linesize*((y&0xFFF8)-1) + x;
      mean += *picture++;
      mean += *picture++;
      mean += *picture++;
      mean += *picture;
      mean = mean>>2;
    }
    else
    {
      // sum A
      picture += linesize*y + (x&0xFFF8) - 1;
      mean += *picture;
      picture += linesize;
      mean += *picture;
      picture += linesize;
      mean += *picture;
      picture += linesize;
      mean += *picture;
      mean = mean>>2;
    }
  }
  return mean;
}

static void fill_4x4_DC(uint8_t* picture, uint32_t x, uint32_t y, uint32_t linesize, uint8_t value)
{
  picture += linesize*y+x;
  uint32_t i;
  for (i=0;i<4;i++)
  {
    *picture++ = value;
    *picture++ = value;
    *picture++ = value;
    *picture++ = value;
    // jump to next line
    picture += linesize-4;
  }
}

C_RESULT p264_intra_8x8_chroma (intra_8x8_chroma_mode_t mode, uint8_t *picture, uint32_t x, uint32_t y, uint32_t linesize)
// CHROMA DC mode validated on one MB
{
// Ai-Bi-M are the boundary pixels of the 16x16 block p(y,x)
//
//   M      B0     B1  ...    B7
//  A0  p(0,0) p(0,1)      p(0,7)
//  A1  p(1,0)
//
// ...             ...
//
// A7  p(7,0)            p(7,7)
//

  C_RESULT res = C_OK;
  uint32_t availability_flag=0;
  uint8_t * picture_start;
  int32_t i,j;

  // determine block availability
  if (x==0)
  {
    // block 8x8 is located in the first column of the frame, A pixels are not available
    availability_flag |= A_UNAVAILABLE;
  }
  if (y==0)
  {
    // block 8x8 is located in the first row of the frame, B pixels are not available
    availability_flag |= B_UNAVAILABLE;
  }

  // check availability and mode
  switch (mode)
  {
    case VERTICAL_8x8_CHROMA_MODE :
      if (availability_flag & B_UNAVAILABLE)
      {
        PRINT("%s : could no compute chroma intra 8x8 mode_%d prediction on (%d,%d), B unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    case HORIZONTAL_8x8_CHROMA_MODE :
      if (availability_flag & A_UNAVAILABLE)
      {
        PRINT("%s : could no compute chroma intra 8x8 mode_%d prediction on (%d,%d) A unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    case DC_8x8_CHROMA_MODE :
      // no restriction
      break;

    case PLANE_8x8_CHROMA_MODE :
     if (availability_flag & A_UNAVAILABLE)
      {
        PRINT("%s : could no compute chroma intra 8x8 mode_%d prediction on (%d,%d) A unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      if (availability_flag & B_UNAVAILABLE)
      {
        PRINT("%s : could no compute chroma intra 8x8 mode_%d prediction on (%d,%d) B unavailable\n",__FUNCTION__,mode,x,y);
        res = C_FAIL;
      }
      break;

    default :
      PRINT ("%s : mode_%d is not a valid chroma intra 8x8 prediction mode\n",__FUNCTION__,mode);
      res = C_FAIL;
  }

  if (SUCCEED(res))
  {
    // make prediction
    switch (mode)
    {
      case  DC_8x8_CHROMA_MODE :
      {
        // DC_8x8_CHROMA_MODE is slightly different from DC_16x16_LUMA_MODE
        // each 8x8 block is divided into 4 blocks
        //  -------------------
        // | all     | single1 |
        //  --------- ---------
        // | single2 | all     |
        //  -------------------
        //
        // In "all" block, adjacent 4x4 blocks A and B are used to compute the DC value as in DC_16x16_LUMA_MODE.
        //   If B is not available, A is used instead of B.
        //   If A is not available, B is used instead of A.
        //   If neither A nor B are available, DC is set to 128.
        // In "single1" priority is given to B block to compute the DC value. If B is not available, A is used instead of B.
        //   If neither A nor B are available, DC is set to 128.
        // In "single2" priority is given to A block to compute the DC value. If A is not available, B is used instead of A.
        //   If neither A nor B are available, DC is set to 128.
        //
        // Next scheme summarize the block used to compute the DC value in raster scan ordered image :
        //          B blocks
        //  A   -----------------
        //     | x    x | A    A |
        //  b  | x    x | A    A |
        //  l   -------- --------  ...
        //  o  | B    B | AB   B |
        //  c  | B    B | A   AB |
        //  k   -------- --------
        //  s          ...

        uint32_t mean;

        // compute DC value of block_4x4(0,0)
        mean = chroma_DC_all(picture,x,y,linesize);
        // fill the block with DC
        fill_4x4_DC(picture,x,y,linesize,mean);

        // compute DC value of block_4x4(4,0)
        mean = chroma_DC_single(picture,x+4,y,linesize);
        // fill the block with DC
        fill_4x4_DC(picture,x+4,y,linesize,mean);

        // compute DC value of block_4x4(0,4)
        mean = chroma_DC_single(picture,x,y+4,linesize);
        // fill the block with DC
        fill_4x4_DC(picture,x,y+4,linesize,mean);

        // compute DC value of block_4x4(4,4)
        mean = chroma_DC_all(picture,x+4,y+4,linesize);
        // fill the block with DC
        fill_4x4_DC(picture,x+4,y+4,linesize,mean);
      }
        break;

      case HORIZONTAL_8x8_CHROMA_MODE :
      {
        //jump to pixel A0
        picture += (linesize*y+x-1);
        // extend pixel Ai horizontally
        uint8_t pixel_buf;
        j=8;
        while(j--)
        {
          // load Ai
          pixel_buf = *picture++;
          // extend Ai
          i=8;
          while(i--)
          {
            *picture++ = pixel_buf;
          }
          // jump to Ai+1
          picture += (linesize-9);
        }
      }
        break;

      case VERTICAL_8x8_CHROMA_MODE :
      {
        // jump to pixel B0
        picture += (linesize*(y-1)+x);
        uint8_t line_buf[8];
        // load Bi line
        vp_os_memcpy(line_buf,picture,8*sizeof(uint8_t));
        // extend Bi line vertically
        i=8;
        while(i--)
        {
          // jump to next line
          picture += linesize;
          // copy line
          vp_os_memcpy(picture,line_buf,8*sizeof(uint8_t));
        }
      }
        break;

      case PLANE_8x8_CHROMA_MODE :
      {
        int32_t A7,B7;
        picture_start = picture;
        // compute H :
        // H' = 1* (B4 - B2) +
        //      2* (B5 - B1) +
        //      3* (B6 - B0) +
        //      4* (B7 -  M)
        //  H = (34*H' + 32) / 64
        // jump to M
        picture += (linesize*(y-1)+x-1);
        int32_t H=0;
        for (i=4;i>=1;i--)
        {
          H -= i*(*picture++);
        }
        for (i=1;i<=4;i++)
        {
          H += i*(*++picture);
        }
        B7 = *(picture);
        H = (34*H + 32)>>6;

        // compute V :
        // V' = 1* (A4 - A2) +
        //      2* (A5 - A1) +
        //      3* (A6 - A0) +
        //      4* (A7 -  M)
         //  V = (34*V' + 32) / 64
        int32_t V=0;
        // jump to M
        picture = picture_start + (linesize*(y-1)+x-1);
        for (i=4;i>=1;i--)
        {
          V -= i*(*picture);
          picture += linesize;
        }
        for (i=1;i<=4;i++)
        {
          picture += linesize;
          V += i*(*picture);
        }
        A7 = *(picture);
        V = (34*V + 32)>>6;

        // fill pixel p(i,j) of the 16x16 block with the following rule :
        // a = 16 * (A7 + B7 + 1) - 3*(V+H)
        // b(i,j) = a + V * j + H * i
        // p(i,j) = SATURATE_U8(b(i,j) / 32)
        // SATURATE_U8() function indicates that the result of the operation should be bounded to an unsigned 8-bit range (0..255)

        int32_t a = ((A7 + B7 + 1)<<4) - 3*(V+H);

        // jump to p(0,0)
        picture = picture_start + (linesize*y+x);
        // make prediction
        int32_t pixel_buf;
        for (j=0;j<8;j++)
        {
          for (i=0;i<8;i++)
          {
            pixel_buf = (a + V*j + H*i)>>5;
            // saturation
            if (pixel_buf > 0xFF)
              pixel_buf = 0xFF;
            else if (pixel_buf < 0)
              pixel_buf = 0;
            *picture++ = pixel_buf;
          }
          // jump to next line
          picture += linesize - 8;
        }
      }
        break;

      default :
        PRINT ("%s : mode_%d is not a valid chrome intra 8x8 prediction mode\n",__FUNCTION__,mode);
        res = C_FAIL;
    }
  }
  return res;
}
