#include <VP_Os/vp_os_types.h>
#include <VLIB/video_macroblock.h>
#include <VLIB/video_controller.h>

// init h264 ip
C_RESULT video_p264_p6_init(void);

// prepare ip to encode a new frame
C_RESULT video_p264_prepare_slice ( video_controller_t* controller, const vp_api_picture_t* blockline);

// encode a MB
int32_t video_p264_encode_MB(uint32_t num_macro_blocks,video_macroblock_t* next_macroblock ,int32_t qp);


// release h264 ip
C_RESULT video_p264_p6_close(void);
