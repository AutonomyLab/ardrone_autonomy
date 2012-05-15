#include "../video_utils_p6.h"

#ifdef HAS_UVLC_ENCODE

        .text
        .align  4


        .global uvlc_encode
        .type   uvlc_encode, %function

/*
  Registers usage
    r0 : stream
    r1 : value_code
    r2 : value_length
    r3 : not_last (do not modify)
    ip : run
    lr : level
*/
uvlc_encode:
        stmdb   sp!, {r4, r5, lr}         /* save regiter on the stack */

        mov     ip, r2                    /* ip = run */
        clz     r4, ip                    /* r4 = clz(run) */

        movs    lr, r1                    /* lr = level */
        rsbmi   lr, lr, #0                /* lr = |level| */
        movmi   r5, #1
        movpl   r5, #0                    /* save sign value */
        mov     r1, #1                    /* value_code = 1 */

  /* Serialize run */

        rsbs    r4, r4, #31               /* length = 31 - clz(run) */
        subgt   ip, ip, r1, lsl r4        /* run = run - (1 << length) */
        add     r4, r4, #1
        add     r2, r4, #1                /* value_length = length + 1 */
        subgt   r4, r4, #1
        orrgt   r1, ip, r1, lsl r4
        addgt   r2, r2, r4

  /* Serialize level */
        mov     ip, #1
        clz     r4, lr
        rsbs    r4, r4, #31               /* r4 = 31 - clz(level) number of bits used in level */
        subgt   lr, ip, lsl r4            /* code -= (1 << length) */
        addgt   r4, r4, #1
        add     r4, r4, #1
        add     r2, r2, r4                /* value_length += length */
        orr     r1, ip, r1, lsl r4        /* value_code = 1 | (value_code << length) */
        subs    r4, r4, #2
        addgt   r2, r2, r4                /* value_length += length */
        orrgt   r1, lr, r1, lsl r4        /* value_code = code | (value_code << length) */
        add     r2, r2, #1
        orr     r1, r5, r1, lsl #1

  /* Serialize not_last if needed */
        ldmia   sp!, {r4, r5, lr}         /* restore saved registers */

        cmp     r3, #0
        addeq   r2, r2, #3                /* value_length += 3 */
        moveq   ip, #5
        orreq   r1, ip, r1, lsl #3        /* value_code = 0x5 | (value_code << 3) */

        b       video_write_data          /* jump to video_write_data to serialize data */

#endif
