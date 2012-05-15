#include <VLIB/video_mem32.h>
#include <VLIB/Platform/video_utils.h>

#include <VP_Os/vp_os_types.h>

// Default implementation

C_RESULT video_zeromem32( uint32_t* dst, uint32_t length )
{
  while( length )
  {
    *dst = 0;

    dst ++;
    length--;
  }

  return C_OK;
}

C_RESULT video_copy32(uint32_t* dst, uint32_t* src, uint32_t nb)
{
  uint32_t i;

  for( i = 0; i < nb; i++ )
  {
    dst[i] = src[i];
  }

  return C_OK;
}

C_RESULT video_copy32_swap(uint32_t* dst, uint32_t* src, uint32_t nb)
{
  uint32_t i;

  for( i = 0; i < nb; i++ )
  {
    dst[i] = bswap( src[i] );
  }

  return C_OK;
}
