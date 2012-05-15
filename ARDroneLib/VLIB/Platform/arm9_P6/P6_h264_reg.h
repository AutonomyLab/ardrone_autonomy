////////////////////////////////////////////////////
// Parrot proprietary DCT registers
////////////////////////////////////////////////////

// Parrot DCT address: 0xD00B0000
#define P6_BASE_ADDRESS         0xD00B0000

// H264 general registers
#define H264_STATUS             0x000       // Status Register
#define H264_ITEN               0x004       // Interrupt Enable Register
#define H264_ITACK              0x008       // Interrupt Acknowledge Register
#define H264_DMA                0x010       // Dma Register
#define H264_DMAINT				0x02C
#define H264_RESET				0x03C
#define H264_START				0x00C
#define H264_CONFIG				0x028
#define H264_LINESIZE           0x01C
#define H264_FRAMESIZE          0x020
#define H264_MB_ADDR            0x024
#define H264_QP                 0x018       // ME/MC Y/CC quantizer

// DCT registers
#define DCT_CONTROL             0x040       // Control Register
#define DCT_ORIG_Y_ADDR         0x044       // Address Register
#define DCT_ORIG_CU_ADDR        0x048       // Address Register
#define DCT_ORIG_CV_ADDR        0x04C       // Address Register
#define DCT_DEST_Y_ADDR         0x050       // Address Register
#define DCT_DEST_CU_ADDR        0x054       // Address Register
#define DCT_DEST_CV_ADDR        0x058       // Address Register
#define DCT_LINEOFFSET          0x05C       // Line size
#define DCT_Q_ADDR				0x064		// quantization table
//#define DCT_DEBUG               0x030?       // Debug register
//#define DCT_SIGNATURE           0x034?       // Signature Register


// Registers bitwise definitions
// Status register
#define DCT_STATUS_END_OK    (1<<0)        // DCT Done
//#define DCT_STATUS_ERROR     (1<<1)        // DCT Error ?

// Interrupt enable register
#define DCT_ITEN_END_OK      (1<<0)        // IT Done enable
//#define DCT_ITEN_ERROR       (1<<1)        // IT Error enable ?

// Interrupt Acknowledge register
#define DCT_ITACK_END_OK     (1<<0)        // IT Done acknowledge
//#define DCT_ITACK_ERROR      (1<<1)        // IT Error acknowledge ?

// DCT control mode (forward or inverse dct)
#define DCT_CTRLMODE_FDCT     0
#define DCT_CTRLMODE_IDCT     1

typedef enum {
  DCT_DMA_INCR    = 0,                    //!<  4 bytes DMA burst
  DCT_DMA_INCR4   = 1,                    //!< 16 bytes DMA burst
  DCT_DMA_INCR8   = 2,                    //!< 32 bytes DMA burst
  DCT_DMA_INCR16  = 3,                    //!< 64 bytes DMA burst
} DCT_DMA_BURST_MODE;


// ME registers
#define ME_ALGORITHM             0x100     // I/P frame choice, disable/enable pred_intra
#define ME_RESULT                0x148     // in I-frame mode : intra mode result (intra 16x16 mod0/1/2/3 or intra 4x4)
#define ME_CMB_FRAME_ADDRY       0x11C     // input frame (to be encoded) Y addr
#define ME_CMB_FRAME_ADDRCU      0x120     // input frame (to be encoded) U addr
#define ME_CMB_FRAME_ADDRCV      0x124     // input frame (to be encoded) V addr
#define ME_IPRED0                0x16C     // intra4*4 pred mode results (first 8 4*4 blocks)
#define ME_IPRED1                0x170     // intra4*4 pred mode results (last 8 4*4 blocks)
#define ME_ANALYSIS              0x118     // define MB split pattern (P-frame)
#define ME_PAT_LIST              0x104     // define/enable search patterns (P-frame)
#define ME_PAT_RESIZE            0x108     //
#define ME_SW_FRAME_ADDRY        0x128     // luma reference frame for P-frame
#define ME_SW_FRAME_ADDRCC       0x12C     // chroma reference frame for P-frame
#define ME_SW_CONFIG             0x140     // search window size
#define ME_PRED_BASE_REG         0x14C     // MV result
#define ME_RD                    0x114

// ME values
#define ME_ALGO_DIS_MVP_INTRA_PRED    (1<<5)  // diasable intra pred and inter MV pred
#define ME_ALGO_I_FRAME               (1<<4)  // configure ME to encode en I frame
#define ME_ALGO_P_FRAME               (0<<4)  // configure ME to encode en I frame

#define ME_ANAL_4x4              (1<<22)
#define ME_ANAL_4x8              (1<<21)
#define ME_ANAL_8x4              (1<<20)
#define ME_ANAL_8x8              (1<<19)
#define ME_ANAL_8x16             (1<<18)
#define ME_ANAL_16x8             (1<<17)
#define ME_ANAL_16x16            (1<<16)

#define ME_PAT_SMALL_DIAMOND    0x08
#define ME_PAT_BIG_DIAMOND      0x09
#define ME_PAT_SMALL_SQUARE     0x0A
#define ME_PAT_BIG_SQUARE       0x0B

//DEB registers
#define DEB_ME_FRAME_ADDRY       0x0A0     // deb reconstructed picture (reference)
#define DEB_ME_FRAME_ADDRCC      0x0A4     // deb reconstructed picture (reference)
#define DEB_ME_TMP_ADDR          0x0A8
#define DEB_MC_TMP_ADDR          0x0AC
#define DEB_CONFIG               0x080
#define DEB_PARAM                0x084
#define DEB_QPS_LEFT             0x08C
#define DEB_QPS_TOP              0x090

//MC registers
#define MC_MB_INFO               0x200

#define IS_INTRA_4x4(a)   ((a&0x1F) == 0)
#define IS_INTRA_16x16(a) ((a&0x1F) != 0)
#define CHROMA_MODE(a)    ((a>>16)&0x03)
#define INTRA_16x16_MODE(a) ((a&0x1F)-1 )
#define INTER_16_8_PARTITION(a) (a&0x1F)

#define I_MB_COMPLETION_FLAG 0x0010101
#define I_MB_COMPLETION_MASK 0x00FFFFF
#define P_MB_COMPLETION_FLAG 0x1010101
#define P_MB_COMPLETION_MASK 0xFFFFFFF
