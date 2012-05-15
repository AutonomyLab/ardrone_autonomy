#ifndef _VIDEO_DCT_P5P_H_
#define _VIDEO_DCT_P5P_H_

// #include <VP_Os/vp_os_types.h>
#include <VLIB/video_dct.h>

////////////////////////////////////////////////////
// Parrot proprietary DCT registers
////////////////////////////////////////////////////

// Parrot DCT address: 0xD00B0000
#define DCT_STATUS              0x000       // Status Register
#define DCT_ITEN                0x004       // Interrupt Enable Register
#define DCT_ITACK               0x008       // Interrupt Acknowledge Register
#define DCT_CONTROL             0x00C       // Control Register
#define DCT_DMA                 0x010       // Dma Register
#define DCT_ORIG_Y_ADDR         0x014       // Address Register
#define DCT_ORIG_CU_ADDR        0x018       // Address Register
#define DCT_ORIG_CV_ADDR        0x01C       // Address Register
#define DCT_DEST_Y_ADDR         0x020       // Address Register
#define DCT_DEST_CU_ADDR        0x024       // Address Register
#define DCT_DEST_CV_ADDR        0x028       // Address Register
#define DCT_LINEOFFSET          0x02C       // Line size
#define DCT_DEBUG               0x030       // Debug register
#define DCT_SIGNATURE           0x034       // Signature Register


// Registers bitwise definitions
// Status register
#define DCT_STATUS_END_OK    (1<<0)        // DCT Done
#define DCT_STATUS_ERROR     (1<<1)        // DCT Error

// Interrupt enable register
#define DCT_ITEN_END_OK      (1<<0)        // IT Done enable
#define DCT_ITEN_ERROR       (1<<1)        // IT Error enable

// Interrupt Acknowledge register
#define DCT_ITACK_END_OK     (1<<0)        // IT Done acknowledge
#define DCT_ITACK_ERROR      (1<<1)        // IT Error acknowledge

// DCT control mode (forward or inverse dct)
#define DCT_CTRLMODE_FDCT     0
#define DCT_CTRLMODE_IDCT     1


//! write to a DCT register
#define dct_write_reg( _reg_, _value_ ) \
 (*((volatile CYG_WORD32 *)(PARROT5_DCT +(_reg_))) = (CYG_WORD32)(_value_))

//! read a DCT register
#define dct_read_reg(_reg_ ) \
 (*((volatile CYG_WORD32 *)(PARROT5_DCT+(_reg_))))

typedef enum {
  DCT_DMA_INCR    = 0,                    //!<  4 bytes DMA burst
  DCT_DMA_INCR4   = 1,                    //!< 16 bytes DMA burst
  DCT_DMA_INCR8   = 2,                    //!< 32 bytes DMA burst
  DCT_DMA_INCR16  = 3,                    //!< 64 bytes DMA burst
} DCT_DMA_BURST_MODE;

C_RESULT video_dct_p5p_init(void);

int16_t* video_fdct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks);
int16_t* video_idct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks);

#endif // ! _VIDEO_DCT_P5P_H_
