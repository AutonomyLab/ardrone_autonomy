
#ifndef __INTRIN__H__
#define __INTRIN__H__

#include <VP_Os/vp_os_types.h>

// #undef always_inline
#if defined(__GNUC__)

#if TARGET_CPU_X86 == 1

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

#endif // TARGET_CPU_X86

#if TARGET_CPU_ARM == 1

#ifdef TARGET_OS_IPHONE

static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
  uint32_t ret;

  __asm __volatile (
                    " rev %0, %1\n"
                    : "=r" (ret)
                    : "r" (value)
                    );

  return ret;
}

#define usat16( value, imm )      __asm __volatile( "usat16 %0, %1, %0" : "=r" (value) : "0" (value), "I" (imm) )
#define usat( value, imm, shift ) __asm __volatile( "usat %0, %2, %0, asr %3" :"=r" (value) :"0" (value), "I" (imm), "I" (shift) )

#define uxtb( ret, value, imm )   __asm __volatile ( "uxtb    %0, %1, ror %2" : "=r" (ret) : "r" (value), "I" (imm) )
#define uxtb16( ret, value, imm ) __asm __volatile ( "uxtb16  %0, %1, ror %2" : "=r" (ret) : "r" (value), "I" (imm) )
#define uxth( ret, value, imm )   __asm __volatile ( "uxth    %0, %1, ror %2" : "=r" (ret) : "r" (value), "I" (imm) )

#else

static INLINE uint32_t _byteswap_ulong(uint32_t value)
{
  int32_t tmp;

  __asm __volatile(
    "eor	%1, %2, %2, ror #16\n"
    "bic	%1, %1, #0x00ff0000\n"
    "mov	%0, %2, ror #8\n"
    "eor	%0, %0, %1, lsr #8"
    : "=r" (value), "=r" (tmp)
    : "r" (value)
  );

  return value;
}

#endif // TARGET_OS_IPHONE

#define clz   __builtin_clz

#endif // TARGET_CPU_ARM

#define bswap _byteswap_ulong

#endif // __GNUC__

#endif // ! __INTRIN__H__

