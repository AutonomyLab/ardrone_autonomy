#include <VLIB/Utils/video_bin_dct.h>

static void bin_fdct(const uint16_t* data_in, int16_t* data_out)
{
}

static void bin_idct(const int16_t* data_in, uint16_t* data_out)
{
  int i;
  int16_t x0, x1, x2, x3, x4, x5, x6, x7;
  int16_t tx0, tx3, tx4, tx5;
  const int16_t* in  = data_in;
  int16_t* out = (int16_t*) data_out;

  // Compute lines
  for( i = 8; i > 0; i-- )
  {
    x0 = in[0];  x1 = in[1];  x2 = in[2];  x3 = in[3];
    x4 = in[4];  x5 = in[5];  x6 = in[6];  x7 = in[7];

    // First step
    x4 = x0/2 - x4;
    x2 = x2 - 3*x6/8;
    x3 = x3 + x5/2;

    // Second step
    x0 = x0 - x4;
    x6 = x6 + 3*x2/8;
    x7 = x7 + x1/8;
    x5 = x5 - 7*x3/8;

    // Third step
    x0 = x0 + x2;
    x2 = x0 - x2*2;
    x4 = x4 + x6;
    x6 = x4 - x6*2;
    x7 = x7 + x5;
    x5 = x7 - x5*2;
    x1 = x1 + x3;
    x3 = x1 - x3*2;

    // Fourth step
    x5 = 5*x3/8 - x5;

    // Fifth step
    x3 = x3 - 3*x5/8;

    // Final step
    out[0] = (x0 + x1) >> 1;
    out[1] = (x4 + x3) >> 1;
    out[2] = (x6 + x5) >> 1;
    out[3] = (x2 + x7) >> 1;
    out[4] = (x2 - x7) >> 1;
    out[5] = (x6 - x5) >> 1;
    out[6] = (x4 - x3) >> 1;
    out[7] = (x0 - x1) >> 1;

    in  += 8;
    out += 8;
  }

  in  = (const int16_t*) data_out;
  out = (int16_t*) data_out;

  // Compute columns
  for( i = 8; i > 0; i-- )
  {
    tx0 = in[0];  x1 = in[8];  x2 = in[16];  tx3 = in[24];
    tx4 = in[32]; tx5 = in[40];  x6 = in[48];  x7 = in[56];

    // First step
    x4 = tx0/2 - tx4;
    x0 = tx0/2 + tx4;
    x3 = tx5/2 + tx3;
    x5 = tx5/2 - tx3;

    x2 =   x2   - 3*x6/8;
    x6 = 3*x2/8 +   x6;

    x7 += x1/8;
    x5 += x3/8;

    // Second step
    x0 = x0 + x2;
    x2 = x0 - x2*2;
    x4 = x4 + x6;
    x6 = x4 - x6*2;
    x7 = x7 + x5;
    x5 = x7 - x5*2;
    x1 = x1 + x3;
    x3 = x1 - x3*2;

    // Third step
    x5 = 5*x3/8 -   x5;
    x3 =   x3   - 3*x5/8;

    // Final step - TODO Check if we can replace out by data_out
    out[0]  = x0 + x1;
    out[8]  = x4 + x3;
    out[16] = x6 + x5;
    out[24] = x2 + x7;
    out[32] = x2 - x7;
    out[40] = x6 - x5;
    out[48] = x4 - x3;
    out[56] = x0 - x1;

    in  += 1;
    out += 1;
  }
}

int16_t* video_fdct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks)
{
  while( num_macro_blocks > 0 )
  {
    bin_fdct((uint16_t*)in, out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_fdct((uint16_t*)in, out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_fdct((uint16_t*)in, out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_fdct((uint16_t*)in, out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_fdct((uint16_t*)in, out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_fdct((uint16_t*)in, out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    num_macro_blocks--;
  }

  return out;
}

int16_t* video_idct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks)
{
  while( num_macro_blocks > 0 )
  {
    bin_idct(in, (uint16_t*)out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_idct(in, (uint16_t*)out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_idct(in, (uint16_t*)out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_idct(in, (uint16_t*)out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_idct(in, (uint16_t*)out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    bin_idct(in, (uint16_t*)out);

    in  += MCU_BLOCK_SIZE;
    out += MCU_BLOCK_SIZE;

    num_macro_blocks--;
  }

  return out;
}
