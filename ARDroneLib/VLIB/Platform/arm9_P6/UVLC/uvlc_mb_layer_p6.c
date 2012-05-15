#include "../video_utils_p6.h"
#include "uvlc_mb_layer_p6.h"

#ifdef HAS_UVLC_WRITE_BLOCK
void uvlc_write_block( video_stream_t* const stream, int16_t* data, int32_t num_coeff)
{
   int32_t code, run;

  // DC coeff
  code = *data++;
  video_write_data( stream, code, 10 );
  num_coeff--;
  // AC coeff
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
        uvlc_encode( stream, code, run, num_coeff );
        run = 0;
      }
    }
}
#endif // HAS_UVLC_WRITE_BLOCK
