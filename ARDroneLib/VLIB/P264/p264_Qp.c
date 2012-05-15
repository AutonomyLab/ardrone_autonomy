#include <VLIB/P264/p264_Qp.h>

// define QP_core_table used to scale p264 4x4 transform coefficients
// QP is the quantizer parameter varying from 0 to 51.
// p264 rescaling rules :
// - luma 4x4 residual block -> use Coeff*QP%6_core_table * 2^(floor(QP/6))
//
// - 4x4 luma DC block       -> use Coeff*QP%6_core_table[0] * 2^(floor(QP/6)-2)                     (Qp>=12)
//                           -> use [Coeff*QP%6_core_table[0] + 2^(1-floor(QP/6)] >> (2-floor(QP/6)) (Qp<12)
//
// - 2x2 chroma DC block     -> use Coeff*QP%6_core_table[0] / 2                  (Qp<6)
//                           -> use Coeff*QP%6_core_table[0] * 2^(floor(QP/6)-1)  (Qp>=6)

static uint16_t QP_core_table[6][16] = {
// QP0 : Qstep0 * Si (* 2^6)
{
 10, 13, 10, 13,
 13, 16, 13, 16,
 10, 13, 10, 13,
 13, 16, 13, 16
},
// QP1 : Qstep1 * Si (* 2^6)
{
 11, 14, 11, 14,
 14, 18, 14, 18,
 11, 14, 11, 14,
 14, 18, 14, 18
},
// QP2 : Qstep2 * Si (* 2^6)
{
  13, 16, 13, 16,
  16, 20, 16, 20,
  13, 16, 13, 16,
  16, 20, 16, 20
},
// QP3 : Qstep3 * Si (* 2^6)
{
  14, 18, 14, 18,
  18, 23, 18, 23,
  14, 18, 14, 18,
  18, 23, 18, 23
},
// QP4 : Qstep4 * Si (* 2^6)
{
  16, 20, 16, 20,
  20, 25, 20, 25,
  16, 20, 16, 20,
  20, 25, 20, 25
},
// QP5 : Qstep5 * Si (* 2^6)
{
  18, 23, 18, 23,
  23, 29, 23, 29,
  18, 23, 18, 23,
  23, 29, 23, 29
}
};


void p264_4x4_residual_scale(int16_t* in, int16_t* out, uint32_t Qp)
// validate for several Qp on one value block
{
  // rescaling rules :
  // - luma 4x4 residual block -> use Coeff*QP%6_core_table * 2^(floor(QP/6))
  uint32_t shift = Qp/6;
  uint16_t* p_Qp_core_table = QP_core_table[Qp%6];
  uint32_t i = 16;

  while(i--)
  {
    *out++ = (((int32_t)(*in++))*((int32_t)*p_Qp_core_table++))<<shift;
  }
}

void p264_2x2_chromaDC_scale(int16_t* in, int16_t* out, uint32_t Qp)
// validate for several Qp on one value block
{
  // in and out could be the same
  // - 2x2 chroma DC block     -> use Coeff*QP%6_core_table[0] / 2                  (Qp<6)
  //                           -> use Coeff*QP%6_core_table[0] * 2^(floor(QP/6)-1)  (Qp>=6)
  uint32_t shift = Qp/6-1;
  uint32_t scale = QP_core_table[Qp%6][0];
  if (Qp < 6)
  {
    *out++ = (((int32_t)(*in++))*scale)>>1;
    *out++ = (((int32_t)(*in++))*scale)>>1;
    *out++ = (((int32_t)(*in++))*scale)>>1;
    *out++ = (((int32_t)(*in++))*scale)>>1;
  }
  else
  {
    *out++ = (((int32_t)(*in++))*scale)<<shift;
    *out++ = (((int32_t)(*in++))*scale)<<shift;
    *out++ = (((int32_t)(*in++))*scale)<<shift;
    *out++ = (((int32_t)(*in++))*scale)<<shift;
  }
}

void p264_4x4_lumaDC_scale(int16_t* in, int16_t* out, uint32_t Qp)
{
  // - 4x4 luma DC block       -> use Coeff*QP%6_core_table[0] * 2^(floor(QP/6)-2)                     (Qp>=12)
  //                           -> use [Coeff*QP%6_core_table[0] + 2^(1-floor(QP/6)] >> (2-floor(QP/6)) (Qp<12)
  uint32_t shift;
  int32_t round;
  uint32_t scale = QP_core_table[Qp%6][0];
  uint32_t i = 16;
  if (Qp < 12)
  {
	//  [QP%6_core_table[0] + 2^(1-floor(QP/6)] >> (2-floor(QP/6))
    shift = 2-Qp/6;
    round = 2<<(1-Qp/6);
    while(i--)
    {
	  *out++ = (((int32_t)(*in++))*scale + round )>>shift;
    }
  }
  else
  {
    shift = Qp/6-2;
    while(i--)
    {
    	*out++ = (((int32_t)(*in++))*scale)<<shift;
    }
  }
}

