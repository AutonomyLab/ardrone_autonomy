#include <VLIB/P264/p264_transform.h>

void p264_hadamard_2x2 (int16_t * in, int16_t * out)
// validated
{
  out[0] = in[0] + in[1] + in[2] + in[3];
  out[1] = in[0] - in[1] + in[2] - in[3];
  out[2] = in[0] + in[1] - in[2] - in[3];
  out[3] = in[0] - in[1] - in[2] + in[3];
}

void p264_ihadamard_4x4(int16_t *tblock, int16_t *block)
// validated
{
  int32_t i;
  int16_t tmp[16];
  int16_t *pTmp = tmp;//, *pblock;
  int16_t p0,p1,p2,p3;
  int16_t t0,t1,t2,t3;

  // Horizontal
  for (i = 0; i < 4; i++)
  {
    t0 = tblock[(i<<2)];
    t1 = tblock[(i<<2)+1];
    t2 = tblock[(i<<2)+2];
    t3 = tblock[(i<<2)+3];

    p0 = t0 + t2;
    p1 = t0 - t2;
    p2 = t1 - t3;
    p3 = t1 + t3;

    *(pTmp++) = p0 + p3;
    *(pTmp++) = p1 + p2;
    *(pTmp++) = p1 - p2;
    *(pTmp++) = p0 - p3;
  }

  //  Vertical
  for (i = 0; i < 4; i++)
  {
    pTmp = tmp + i;
    t0 = *pTmp;
    t1 = *(pTmp += 4);
    t2 = *(pTmp += 4);
    t3 = *(pTmp += 4);

    p0 = t0 + t2;
    p1 = t0 - t2;
    p2 = t1 - t3;
    p3 = t1 + t3;

    block[0+i] = p0 + p3;
    block[4+i] = p1 + p2;
    block[8+i] = p1 - p2;
    block[12+i] = p0 - p3;
  }
}

void p264_inverse_4x4(int16_t *tblock, int16_t *block)
// validated
{
  int32_t i;//, ii;
  int16_t tmp[16];
  int16_t *pTmp = tmp;//, *pblock;
  int16_t p0,p1,p2,p3;
  int16_t t0,t1,t2,t3;

  // Horizontal
  for (i = 0; i < 4; i++)
  {
    t0 = tblock[(i<<2)];
    t1 = tblock[(i<<2)+1];
    t2 = tblock[(i<<2)+2];
    t3 = tblock[(i<<2)+3];

    p0 =  t0 + t2;
    p1 =  t0 - t2;
    p2 = (t1 >> 1) - t3;
    p3 =  t1 + (t3 >> 1);

    *(pTmp++) = p0 + p3;
    *(pTmp++) = p1 + p2;
    *(pTmp++) = p1 - p2;
    *(pTmp++) = p0 - p3;
  }

  //  Vertical
  for (i = 0; i < 4; i++)
  {
    pTmp = tmp + i;
    t0 = *pTmp;
    t1 = *(pTmp += 4);
    t2 = *(pTmp += 4);
    t3 = *(pTmp += 4);

    p0 = t0 + t2;
    p1 = t0 - t2;
    p2 =(t1 >> 1) - t3;
    p3 = t1 + (t3 >> 1);

    block[0+i] = p0 + p3;
    block[4+i] = p1 + p2;
    block[8+i] = p1 - p2;
    block[12+i] = p0 - p3;
  }
}
