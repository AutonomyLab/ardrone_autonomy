#ifndef _P263_LAYERS_H_
#define _P263_LAYERS_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_macroblock.h>

#define MAKE_START_CODE(gob)      ( 0x000020 | (gob) )
#define PICTURE_START_CODE        MAKE_START_CODE(0)

#define PICTURE_FORMAT(a)         ( (a) & 7 )
#define PICTURE_TYPE(a)           ( ((a) & 1) + ((a) & 16) )  /* This 'trick' works because if bit 1 is 0 */
                                                              /* then bit 5 must be 0 */
#define PICTURE_EXTENDED_TYPE(a)  ( ((a) >> 6) & 7 )

#define HAS_CUSTOM_PICTURE_FORMAT(a)            ( ((a) >> 15) == 6 )
#define HAS_CUSTOM_PCF(a)                       ( ((a) >> 14) == 1 )
#define HAS_UNRESTRICTED_MOTION_VECTOR(a)       ( ((a) >> 13) == 1 )
#define HAS_SYNTAX_BASED_ARITHMETIC_CODING(a)   ( ((a) >> 12) == 1 )
#define HAS_ADVANCED_PREDICTION(a)              ( ((a) >> 11) == 1 )
#define HAS_ADVANCED_INTRA_CODING(a)            ( ((a) >> 10) == 1 )
#define HAS_DEBLOCKING_FILTER(a)                ( ((a) >>  9) == 1 )
#define HAS_SLICE_STRUCTURED(a)                 ( ((a) >>  8) == 1 )
#define HAS_REFERENCE_PICTURE_SELECTION(a)      ( ((a) >>  7) == 1 )
#define HAS_INDEPENDENT_SEGMENT_DECODING(a)     ( ((a) >>  6) == 1 )
#define HAS_ALTERNATIVE_INTER_VLC(a)            ( ((a) >>  5) == 1 )
#define HAS_MODIFIED_QUANTIZATION(a)            ( ((a) >>  4) == 1 )
#define HAS_REFERENCE_PICTURE_RESAMPLING(a)     ( ((a) >>  5) == 1 )

typedef enum _p263_picture_format {
  P263_PICTURE_FORMAT_FORBIDDEN = 0,
  P263_PICTURE_FORMAT_SUBQCIF,
  P263_PICTURE_FORMAT_QCIF,
  P263_PICTURE_FORMAT_CIF,
  P263_PICTURE_FORMAT_4QCIF,
  P263_PICTURE_FORMAT_16CIF,
  P263_PICTURE_FORMAT_RESERVED,
  P263_PICTURE_FORMAT_EXTENDED
} p263_picture_format;

typedef struct _p263_gob_layer_t {
  video_macroblock_t* macroblocks;  // macroblocks data
  uint32_t gquant;                  // Quantizer Information (GQUANT)
  //uint32_t gn;                      // Group Number (GN)
  //uint32_t gsbi;                    // GOB Sub-Bitstream Indicator (GSBI)
  //uint32_t gfid;                    // GOB Frame ID (GFID) 
} p263_gob_layer_t;

typedef struct _p263_picture_layer_t {
  uint32_t  opptype;                // optional part of PLUSPTYPE (18 bits)
  uint32_t  tr;                     // Temporal reference (8 bits)
  uint32_t  cpcfc;                  // Custom Picture Clock Frequency Code (CPCFC) (8 bits)
  uint32_t  etr;                    // Extended Temporal Reference (ETR) (2 bits)
  uint32_t  uui;                    // Unlimited Unrestricted Motion Vectors Indicator (UUI) (Variable length)
  uint32_t  ptype;                  // Type information (8 ou 13 bits)
  uint32_t  epar;                   // Extended Pixel Aspect Ratio (EPAR) (16 bits)
  uint32_t  plusptype;              // Type information extended (12 bits)
  uint32_t  cpfmt;                  // Custom Picture Format (CPFMT) (23 bits)
  uint32_t  trp;                    // Temporal Reference for Prediction (TRP) (10 bits)
  uint32_t  sss;                    // Slice Structured Submode bits (SSS) (2 bits)
  uint32_t  elnum;                  // Enhancement Layer Number (ELNUM) (4 bits)
  uint32_t  rlnum;                  // Reference Layer Number (RLNUM) (4 bits)
  uint32_t  rpsmf;                  // Reference Picture Selection Mode Flags (RPSMF) (3 bits)
  uint32_t  trpi;                   // Temporal Reference for Prediction Indication (TRPI) (1 bit)
  uint32_t  bci;                    // Back-Channel message Indication (BCI) (Variable length)
  uint32_t  bcm;                    // Back-Channel Message (BCM) (Variable length)
  uint32_t  rprp;                   // Reference Picture Resampling Parameters (RPRP) (Variable length)
  uint32_t  pquant;                 // Quantizer Information (PQUANT) (5 bits)
  uint32_t  cpm;                    // Continuous Presence Multipoint and Video Multiplex (CPM) (1 bit)
  uint32_t  psbi;                   // Picture Sub-Bitstream Indicator (PSBI) (2 bits)
  uint32_t  trb;                    // Temporal Reference for B-pictures in PB-frames (TRB) (3/5 bits)
  uint32_t  dbquant;                // Quantization information for B-pictures in PB-frames (DBQUANT) (2 bits)
  uint32_t  pei;                    // Extra Insertion Information (PEI) (1 bit)
  uint8_t*  psupp;                  // Supplemental Enhancement Information (PSUPP) (0/8/16 ... bits)
  p263_gob_layer_t* gobs;           // List of gobs
} p263_picture_layer_t;

// Macroblock types and included data elements for normal pictures
#define MB_TYPE_COD_BIT     0x01
#define MB_TYPE_MCBPC_BIT   0x02
#define MB_TYPE_CBPY_BIT    0x03
#define MB_TYPE_DQUANT_BIT  0x04
#define MB_TYPE_MVD_BIT     0x05
#define MB_TYPE_MVD24_BIT   0x06

#define MAKE_MB_TYPE( COD, MCBPC, CBPY, DQUANT, MVD, MVD24 )  \
  ( (COD    << MB_TYPE_COD_BIT    ) |                         \
    (MCBPC  << MB_TYPE_MCBPC_BIT  ) |                         \
    (CBPY   << MB_TYPE_CBPY_BIT   ) |                         \
    (DQUANT << MB_TYPE_DQUANT_BIT ) |                         \
    (MVD    << MB_TYPE_MVD_BIT    ) |                         \
    (MVD24  << MB_TYPE_MVD24_BIT  ) )

#define MB_TYPE_HAS_COD(t)    ( (t) & ( 1 << MB_TYPE_COD_BIT   ))
#define MB_TYPE_HAS_MCBPC(t)  ( (t) & ( 1 << MB_TYPE_MCBPC_BIT ))
#define MB_TYPE_HAS_CBPY(t)   ( (t) & ( 1 << MB_TYPE_CBPY_BIT  ))
#define MB_TYPE_HAS_DQUANT(t) ( (t) & ( 1 << MB_TYPE_DQUANT_BIT))
#define MB_TYPE_HAS_MVD(t)    ( (t) & ( 1 << MB_TYPE_MVD_BIT   ))
#define MB_TYPE_HAS_MVD24(t)  ( (t) & ( 1 << MB_TYPE_MVD24_BIT ))

#define STANDARD_MB_TYPES_NUM 7

typedef uint32_t p263_mb_type_t;
extern p263_mb_type_t standard_mb_types[STANDARD_MB_TYPES_NUM];

C_RESULT p263_read_picture_layer( video_controller_t* controller, video_stream_t* stream );
C_RESULT p263_read_gob_layer( video_controller_t* controller, video_stream_t* stream );
C_RESULT p263_read_mb_layer( video_controller_t* controller, video_stream_t* stream, video_macroblock_t* mb );

#endif // _P263_LAYERS_H_
