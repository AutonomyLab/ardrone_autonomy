#include "../video_utils_p5p.h"

#ifdef HAS_UVLC_WRITE_BLOCK

        .global uvlc_write_block
        .type   uvlc_write_block, %function

/* This code is an inline version of uvlc_write_block using video_zztable_t81

void uvlc_write_block( video_stream_t* const stream, int16_t* data, int32_t num_coeff ) */

#define zz(num1, num2)                            \
video_zztable_t81_next_##num1:                    ;\
        cmp     r1, #0                            ;\
        addeq   r2, r2, #1;                        \
        ldreqsh r1, [r4, #2*num2];                \
        beq     video_zztable_t81_next_##num2;     \
        sub     r3, r3, #1;                        \
        bl      uvlc_encode;                       \
        cmp     r3, #0;                            \
        ldrgtsh r1, [r4, #2*num2];                \
        ldmleia sp!, {r4, pc};                     \
        mov     r2, #0;

uvlc_write_block:
        stmdb   sp!, {r4, lr}

        mov     r4, r1
        ldrsh   r1, [r1, #0]
        sub     r3, r2, #1
        mov     r2, #10
        bl      video_write_data
        cmp     r3, #0
        ldrgtsh r1, [r4, #2]
        mov     r2, #0
        ldmleia sp!, {r4, pc}

        zz(1, 8)
        zz(8, 16)
        zz(16, 9)
        zz(9, 2)
        zz(2, 3)
        zz(3, 10)
        zz(10, 17)
        zz(17, 24)
        zz(24, 32)
        zz(32, 25)
        zz(25, 18)
        zz(18, 11)
        zz(11, 4)
        zz(4, 5)
        zz(5, 12)
        zz(12, 19)
        zz(19, 26)
        zz(26, 33)
        zz(33, 40)
        zz(40, 48)
        zz(48, 41)
        zz(41, 34)
        zz(34, 27)
        zz(27, 20)
        zz(20, 13)
        zz(13, 6)
        zz(6, 7)
        zz(7, 14)
        zz(14, 21)
        zz(21, 28)
        zz(28, 35)
        zz(35, 42)
        zz(42, 49)
        zz(49, 56)
        zz(56, 57)
        zz(57, 50)
        zz(50, 43)
        zz(43, 36)
        zz(36, 29)
        zz(29, 22)
        zz(22, 15)
        zz(15, 23)
        zz(23, 30)
        zz(30, 37)
        zz(37, 44)
        zz(44, 51)
        zz(51, 58)
        zz(58, 59)
        zz(59, 52)
        zz(52, 45)
        zz(45, 38)
        zz(38, 31)
        zz(31, 39)
        zz(39, 46)
        zz(46, 53)
        zz(53, 60)
        zz(60, 61)
        zz(61, 54)
        zz(54, 47)
        zz(47, 55)
        zz(55, 62)
        zz(62, 63)


video_zztable_t81_next_63:
        mov     r3, #0
        ldmia   sp!, {r4, lr}
        b       uvlc_encode

/* This code is an optimized version of uvlc_write_block using video_zztable_t81

void uvlc_write_block( video_stream_t* const stream, int16_t* data, int32_t num_coeff ) */
/*
uvlc_write_block:
        stmdb   sp!, {r4, r5, r6, lr}

        mov     r4, r1
        ldrsh   r1, [r1, #0]
        sub     r3, r2, #1
        mov     r2, #8
        bl      video_write_data
        mov     r2, #0
        cmp     r3, #0
        ldmleia sp!, {r4, r5, r6, pc}
        ldr     r5, =video_zztable_t81

video_zztable_t81_next_1:
        ldr     r6, [r5, #4]!
        mov     r6, r6, lsl #1
        ldrsh   r1, [r4, r6]
        cmp     r1, #0
        addeq   r2, r2, #1
        beq     video_zztable_t81_next_1
        sub     r3, r3, #1
        bl      uvlc_encode
        cmp     r3, #0
        ldmleia sp!, {r4, r5, r6, pc}
        mov     r2, #0
        b video_zztable_t81_next_1
*/

#endif // HAS_UVLC_WRITE_BLOCK
