#include "video_utils_p5p.h"
#include <VLIB/video_picture_defines.h>

#ifdef HAS_VIDEO_BLOCKLINE_TO_MACRO_BLOCKS

#include "config-tcm.h"

        .section ".text.itcm","ax"
        .global video_blockline_to_macro_blocks
        .global video_blockline_patch_block_1
        .global video_blockline_patch_block_2_start
        .global video_blockline_patch_block_2
        .global video_blockline_patch_block_3_start
        .global video_blockline_patch_block_3
        .global video_blockline_patch_block_4_start
        .global video_blockline_patch_block_4
        .global video_blockline_patch_fix_y
        .global video_blockline_patch_block_cb
        .global video_blockline_patch_fix_cb
        .global video_blockline_patch_block_cr
        .global video_blockline_patch_fix_cr
        .type   video_blockline_to_macro_blocks, %function

/* Registers usage
      r0 : ctx
      r1 : dst
      r2 : num_macro_blocks
      r3 : y_src
      r4 : cb_src
      r5 : cr_src
  r6, r7, r8, r9 : Pixels in 16 bits format (write in dst)
  r10, r11 : Pixels in 8 bits format (read from y_src, cb_src or cr_src)
  ip/r12 : not used for instance
  lr/r14 : line counter in internal loop (8 lines per block)
*/

video_blockline_to_macro_blocks:
        stmdb   sp!, {r4, r5, r6, r7, r8, r9, r10, r11, lr}
        ldm     r0, { r3, r4, r5 }

video_blockline_to_macro_blocks_loop:

        @@ Luminances
        @@ Copy first block
        mov     lr, #8

copy_block_1: @ data conversion from 8 bits to 16 bits

        ldmia   r3!, {r10-r11} @ Get 8 pixels from y_src

        @ isolate the first four pixel
        and     r6, r10, #0x00FF
        and     r8, r10, #0xFF00

        mov     r10, r10, LSR #16

        and     r7, r10, #0x00FF
        and     r9, r10, #0xFF00

        @ Combine them
        orr     r6, r8, LSL #8
        orr     r7, r9, LSL #8

        @ isolate next four pixels
        and     r8,  r11, #0x00FF
        and     r10, r11, #0xFF00

        mov     r11, r11, LSR #16

        and     r9,  r11, #0x00FF
        and     r11, r11, #0xFF00

        @ Combine them
        orr     r8, r10, LSL #8
        orr     r9, r11, LSL #8

        @ Store result of conversion to dst
        stmia   r1!, {r6-r9}

        subs    lr, lr, #1

        @ Proceed to next line
video_blockline_patch_block_1:
        addne   r3, #(0)
        bne copy_block_1


        @@ Copy second block
video_blockline_patch_block_2_start:
        sub     r3, #(0)
        mov     lr, #8

copy_block_2: @ data conversion from 8 bits to 16 bits

        ldm     r3, {r10-r11} @ Get 8 pixels from y_src

        @ isolate the first four pixel
        and     r6, r10, #0x00FF
        and     r8, r10, #0xFF00

        mov     r10, r10, LSR #16

        and     r7, r10, #0x00FF
        and     r9, r10, #0xFF00

        @ Combine them
        orr     r6, r8, LSL #8
        orr     r7, r9, LSL #8

        @ isolate next four pixels
        and     r8,  r11, #0x00FF
        and     r10, r11, #0xFF00

        mov     r11, r11, LSR #16

        and     r9,  r11, #0x00FF
        and     r11, r11, #0xFF00

        @ Combine them
        orr     r8, r10, LSL #8
        orr     r9, r11, LSL #8

        @ Store result of conversion to dst
        stmia   r1!, {r6-r9}

        subs    lr, lr, #1

        @ Proceed to next line
video_blockline_patch_block_2:
        addne   r3, #(0)
        bne copy_block_2


        @@ Copy third block
video_blockline_patch_block_3_start:
        add     r3, #(0)
        mov     lr, #8

copy_block_3: @ data conversion from 8 bits to 16 bits

        ldmia   r3!, {r10-r11} @ Get 8 pixels from y_src

        @ isolate the first four pixel
        and     r6, r10, #0x00FF
        and     r8, r10, #0xFF00

        mov     r10, r10, LSR #16

        and     r7, r10, #0x00FF
        and     r9, r10, #0xFF00

        @ Combine them
        orr     r6, r8, LSL #8
        orr     r7, r9, LSL #8

        @ isolate next four pixels
        and     r8,  r11, #0x00FF
        and     r10, r11, #0xFF00

        mov     r11, r11, LSR #16

        and     r9,  r11, #0x00FF
        and     r11, r11, #0xFF00

        @ Combine them
        orr     r8, r10, LSL #8
        orr     r9, r11, LSL #8

        @ Store result of conversion to dst
        stmia   r1!, {r6-r9}

        subs    lr, lr, #1

        @ Proceed to next line
video_blockline_patch_block_3:
        addne   r3, #(0)
        bne copy_block_3


        @@ Copy fourth block
video_blockline_patch_block_4_start:
        sub     r3, #(0)
        mov     lr, #8

copy_block_4: @ data conversion from 8 bits to 16 bits

        ldmia   r3!, {r10-r11} @ Get 8 pixels from y_src

        @ isolate the first four pixel
        and     r6, r10, #0x00FF
        and     r8, r10, #0xFF00

        mov     r10, r10, LSR #16

        and     r7, r10, #0x00FF
        and     r9, r10, #0xFF00

        @ Combine them
        orr     r6, r8, LSL #8
        orr     r7, r9, LSL #8

        @ isolate next four pixels
        and     r8,  r11, #0x00FF
        and     r10, r11, #0xFF00

        mov     r11, r11, LSR #16

        and     r9,  r11, #0x00FF
        and     r11, r11, #0xFF00

        @ Combine them
        orr     r8, r10, LSL #8
        orr     r9, r11, LSL #8

        @ Store result of conversion to dst
        stmia   r1!, {r6-r9}

        subs    lr, lr, #1

        @ Proceed to next line
video_blockline_patch_block_4:
        addne   r3, #(0)
        bne copy_block_4

video_blockline_patch_fix_y:
        sub     r3, #(0) @ Fix r3 for next iteration


        @@ Chrominances
        @@ Copy fifth block
        mov     lr, #8

copy_block_cb: @ data conversion from 8 bits to 16 bits

        ldmia   r4!, {r10-r11} @ Get 8 pixels from cb_src

        @ isolate the first four pixel
        and     r6, r10, #0x00FF
        and     r8, r10, #0xFF00

        mov     r10, r10, LSR #16

        and     r7, r10, #0x00FF
        and     r9, r10, #0xFF00

        @ Combine them
        orr     r6, r8, LSL #8
        orr     r7, r9, LSL #8

        @ isolate next four pixels
        and     r8,  r11, #0x00FF
        and     r10, r11, #0xFF00

        mov     r11, r11, LSR #16

        and     r9,  r11, #0x00FF
        and     r11, r11, #0xFF00

        @ Combine them
        orr     r8, r10, LSL #8
        orr     r9, r11, LSL #8

        @ Store result of conversion to dst
        stmia   r1!, {r6-r9}

        subs    lr, lr, #1

        @ Proceed to next line
video_blockline_patch_block_cb:
        addne   r4, #(0)
        bne copy_block_cb

video_blockline_patch_fix_cb:
        sub     r4, #(0) @ Fix r4 for next iteration


        @@ Copy sixth block
        mov     lr, #8

copy_block_cr: @ data conversion from 8 bits to 16 bits

        ldmia   r5!, {r10-r11} @ Get 8 pixels from cr_src

        @ isolate the first four pixel
        and     r6, r10, #0x00FF
        and     r8, r10, #0xFF00

        mov     r10, r10, LSR #16

        and     r7, r10, #0x00FF
        and     r9, r10, #0xFF00

        @ Combine them
        orr     r6, r8, LSL #8
        orr     r7, r9, LSL #8

        @ isolate next four pixels
        and     r8,  r11, #0x00FF
        and     r10, r11, #0xFF00

        mov     r11, r11, LSR #16

        and     r9,  r11, #0x00FF
        and     r11, r11, #0xFF00

        @ Combine them
        orr     r8, r10, LSL #8
        orr     r9, r11, LSL #8

        @ Store result of conversion to dst
        stmia   r1!, {r6-r9}

        subs    lr, lr, #1

        @ Proceed to next line
video_blockline_patch_block_cr:
        addne   r5, #(0)
        bne copy_block_cr

video_blockline_patch_fix_cr:
        sub     r5, #(0) @ Fix r5 for next iteration


        subs    r2, r2, #1
        bne     video_blockline_to_macro_blocks_loop

        stm     r0, { r3, r4, r5 }
        ldmia   sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}

#endif // HAS_VIDEO_BLOCKLINE_TO_MACRO_BLOCKS
