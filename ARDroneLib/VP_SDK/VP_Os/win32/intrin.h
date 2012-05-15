
#ifndef __INTRIN__H__
#define __INTRIN__H__

#ifdef _WIN32
	//For ByteSwap
	#include <stdlib.h>
#endif

#ifdef USE_MINGW32

// #undef always_inline

#ifdef __GNUC__

static INLINE uint32_t _BitScanReverse(uint32_t* index, uint32_t mask)
{
  __asm__("bsrl %[mask], %[index]" : [index] "=r" (*index) : [mask] "mr" (mask));

  return mask ? 1 : 0;
}


static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
  __asm("bswap %0":
      "=r" (value):
      "0" (value));

  return value;
}

#endif // __GNUC__

#else // USE_MINGW32
#endif // USE_MINGW32


static inline uint32_t clz(uint32_t code)
{
  uint32_t index = 0;
  if( code )
  {
    _BitScanReverse(&index, code);
    index ^= 31;
  }

  return index;
}


#define bswap _byteswap_ulong


#endif // ! __INTRIN__H__

