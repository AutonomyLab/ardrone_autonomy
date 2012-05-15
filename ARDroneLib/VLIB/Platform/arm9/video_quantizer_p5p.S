#include "video_utils_p5p.h"

#ifdef HAS_DO_QUANTIZE_INTRA_MB

#ifdef _ECOS
#include "config-tcm.h"

        .section ".text.itcm","ax"
#endif // ! _ECOS

        .global do_quantize_intra_mb
        .type   do_quantize_intra_mb, %function

/*  This implementation compute two quantizations at a time
    using ARM926EJ-S DSP extension (smul<x><y> = 1 cycle if no dependency follow)

    Registers usage
      r0 : [in ptr] data ptr
      r1 : [in] quantification factor (16 bits)
      r2 : [out ptr] number of non zero factor
      r3 : data read from memory [r0] & value quantified from lsb of r3
      r4 : value quantified from msb of r3
      r5 : number of non zero factor
  ip/r12 : bloc counter (a macroblock has 6 blocks)
  lr/r14 : number of coefficient in block left to compute (there's 64 coefficiens per block (1 dc et 63 ac)
 */
do_quantize_intra_mb:
        stmdb   sp!, {r4, r5, lr}
        mov     ip, #6                /* initialize bloc counter i = 6 */
        ldr     r3, [r0]              /* read dc coefficient & first ac coefficient */

do_quantize_intra_l0:
        mov     r5, #1                /* last = 1 */
        smulbt  r4, r1, r3            /* coeff *= invQuant */
        mov     r3, r3, lsl #16       /* set r3 msb to zero */
        add     r3, r3, #0x40000      /* coeff = (*ptr + 4) >> 3 */
        movs    r3, r3, asr #19
        moveq   r3, #1                /* if( coeff == 0 ) coeff = 1 */
        cmp     r4, #0
        beq do_quantize_intra_l01
        rsblt   r4, r4, #0
        mov     r4, r4, asr #16       /* |coeff| >>= 16 */
        rsblt   r4, r4, #0
        cmp     r4, #0
        addne   r5, r5, #1            /* if( coeff != 0 ) last++ */
        orrne   r3, r3, r4, lsl #16
do_quantize_intra_l01:
        str     r3, [r0]
        ldr     r3, [r0, #4]!         /* read ac(3) & ac(2) coefficients */
        mov     lr, #31               /* 31 pairs to read */

do_quantize_intra_l1:
        cmp     r3, #0                /* do nothing if both coefficients are zero */
        beq     do_quantize_intra_l2
        smulbt  r4, r1, r3            /* coeff *= invQuant */
        smulbb  r3, r1, r3            /* coeff *= invQuant */
        cmp     r4, #0
        beq     do_quantize_intra_l11
        rsblt   r4, r4, #0
        mov     r4, r4, asr #16       /* |coeff| >>= 16 */
        rsblt   r4, r4, #0
        movs    r4, r4, lsl #16       /* keep only 16 lower significant bits */
        addne   r5, r5, #1            /* if( coeff != 0 ) last++ */
do_quantize_intra_l11:
        cmp     r3, #0
        beq     do_quantize_intra_l12
        rsblt   r3, r3, #0
        mov     r3, r3, asr #16
        rsblt   r3, r3, #0
        movs    r3, r3, lsl #16       /* keep only 16 lower significant bits */
        addne   r5, r5, #1            /* if( coeff != 0 ) last++ */
do_quantize_intra_l12:
        orr     r3, r4, r3, lsr #16
        str     r3, [r0]
do_quantize_intra_l2:
        subs    lr, lr, #1
        ldrne   r3, [r0, #4]!         /* read ac(i+1) & ac(i) coefficients */
        bne     do_quantize_intra_l1

        str     r5, [r2], #4          /* store number of non zero coefficient for current bloc */
        subs    ip, ip, #1            /* i-- */
        ldrne   r3, [r0, #4]!         /* read dc coefficient & first ac coefficient */
        bne     do_quantize_intra_l0
do_quantize_intra_exit:
        add     r0, #4                /* keep consistency because last reads are conditionals */
        ldmia   sp!, {r4, r5, pc}

#endif // HAS_DO_QUANTIZE_INTRA_MB
