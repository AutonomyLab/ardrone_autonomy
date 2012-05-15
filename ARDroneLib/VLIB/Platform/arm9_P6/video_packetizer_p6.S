#include "video_utils_p6.h"

#ifdef HAS_VIDEO_WRITE_DATA

        .text
        .align  4


        .global video_write_data
        .type   video_write_data, %function

/* Registers usage for video_write_data (r0 & r3 must be preserved)
       r0 = stream
       r1 = code
       r2 = length
       r3 = video_stream->code
       r4 = video_stream->used
       r5 = video_stream->bytes
       ip = video_stream->length
       lr = video_stream->index
*/

video_write_data:
        ldr     ip, [r0]
        stmdb   sp!, {r3, r4, r5, lr}         /* save regiter on the stack */
        cmp     r2, ip                        /* length > stream->length ? */
        ldmib   r0, {r3, r4, r5, lr}

        ble     video_write_data_1
        add     r4, r4, #4                    /* stream->used += 4 */
        str     r4, [r0, #8]                  /* write back stream->used */
        mov     r3, r3, lsl ip                /* stream->code <<= stream->length */
        mov     r4, #1
        sub     r2, r2, ip                    /* length -= stream->length */
        rsb     r4, r4, r4, lsl r2            /* code   &= (1 << length) - 1 (rd = shifter operand - rn) */
        orr     r3, r3, r1, lsr r2            /* stream->code  |= code >> (length - stream->length) */
        and     r1, r1, r4                    /* code   &= (1 << length) - 1 */
        str     r3, [r5, lr, lsl #2]          /* stream->bytes[stream->index] = stream->code */
        add     lr, lr, #1                    /* stream->index++ */
        str     lr, [r0, #16]                 /* write back stream->index */
        mov     r3, #0                        /* stream->code = 0 */
        mov     ip, #32                       /* stream->length   = 32 */

video_write_data_1:
        cmp     r2, #0
        orrne   r3, r1, r3, lsl r2            /* stream->code = (stream->code << length) | code */
        rsbne   r2, r2, ip                    /* stream->length -= length */
        stmia   r0, {r2, r3}
        ldmia   sp!, {r3, r4, r5, pc}

#endif // HAS_VIDEO_WRITE_DATA
