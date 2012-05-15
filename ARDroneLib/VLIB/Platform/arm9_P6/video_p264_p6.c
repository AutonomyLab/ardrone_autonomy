#include <stdio.h>
#include "libuiomap.h"
#include "dma_malloc.h"
#include "video_config.h"
#include "video_p264_p6.h"
#include "video_utils_p6.h"
#include <VP_Os/vp_os_malloc.h>
#include "P6_h264_reg.h"
#include <VLIB/P264/p264_common.h>
#include <VLIB/P264/p264_zigzag.h>
#include <stdio.h>
#include <VP_Os/vp_os_print.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <parrot/p264/p264_p6_ioctl.h>


#define CHECK_P264_IOCTL(a) if((a)<0) { PRINT("p264 ioctl failed (%s)\n",#a); return C_FAIL;}
//#define H264_P6_DEBUG
//#define H264_P6_PFRAME_DEBUG

// h264 picture parameters
static uint8_t* current_Y;
static uint8_t* current_Cb;
static uint8_t* current_Cr;
static uint32_t current_width;
static uint32_t current_height;
static bool_t I_encoding;

static uint8_t* h264_ref_frame;  // frame decoded by encoder as a reference
static uint8_t* h264_deb_frame;  // frame decoded by encoder as a reference
static p264_p6_raw_reg_results_t h264_raw_result[400]; // chroma_mode, intra_mode, MV, ...

static int h264_fd = -1;

C_RESULT video_p264_p6_init(void)
{
  C_RESULT res = C_OK;
  h264_fd = open("/dev/p264_p6", O_RDWR|O_NONBLOCK);
  if (!h264_fd)
  {
    PRINT("p264 driver not found...\n");
    res = C_FAIL;
  }

  // init local variable
  // h264 picture parameters
  current_Y  = NULL;
  current_Cb = NULL;
  current_Cr = NULL;
  current_width = 0;
  current_height = 0;
  h264_deb_frame = NULL;
  h264_ref_frame = NULL;

  return res;
}

#ifdef HAS_P264_FTRANSFORM
C_RESULT video_p264_prepare_slice ( video_controller_t* controller, const vp_api_picture_t* blockline)
{
  // picture dimensions change ?
  if (controller->width != current_width || controller->height != current_height)
  {
    // realloc a YUV 4:2:0 reference frame
    h264_ref_frame = (uint8_t*)dma_realloc(h264_ref_frame,controller->width*controller->height*3/2);
    h264_deb_frame = (uint8_t*)dma_realloc(h264_deb_frame,controller->width*controller->height*3/2);
    current_width = controller->width;
    current_height = controller->height;

    if (current_width > CIF_WIDTH || current_height > CIF_HEIGHT)
      PRINT("resolutions above CIF are not supported by P6 h264 ip\n");
    // set search window size
    // ... use default value

    uint32_t value = P264_RES(current_width,current_height);
    CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_DIM, &value))

#ifdef H264_P6_DEBUG
    PRINT ("H264 IP : new frame dimensions %dx%d\n",current_width,current_height);
#endif
  }

  // swap DEB and ref frames
  uint8_t* p_tmp;
  p_tmp = h264_deb_frame;
  h264_deb_frame = h264_ref_frame;
  h264_ref_frame = p_tmp;

  // set DEB new frame
  uint32_t value = (uint32_t)dma_virt2phy(h264_deb_frame);
  CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_DEB_FRAME, &value))

  dma_flush_inv( (uint32_t)h264_deb_frame, current_width*current_height*3/2);

  // set reference frame
  value = (uint32_t)dma_virt2phy(h264_ref_frame);
  CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_REF_FRAME, &value))
  dma_flush_inv( (uint32_t)h264_ref_frame, current_width*current_height*3/2);

  CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_FRAME_TYPE, &(controller->picture_type)))

  // devrait pouvoir sauter
  if (controller->picture_type == VIDEO_PICTURE_INTRA)
    I_encoding = TRUE;
  else
    I_encoding = FALSE;

  // retrieve picture pointers
  current_Y = blockline->y_buf;
  current_Cb = blockline->cb_buf;
  current_Cr = blockline->cr_buf;

  p264_p6_input_buf_t input_buf;
  input_buf.phys_Y = (uint32_t)dma_virt2phy(current_Y);
  input_buf.phys_Cb = (uint32_t)dma_virt2phy(current_Cb);
  input_buf.phys_Cr = (uint32_t)dma_virt2phy(current_Cr);
  CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_INPUT_BUF, &input_buf))

  // flush input frame
  dma_flush_inv( (uint32_t)current_Y, current_width*current_height);
  dma_flush_inv( (uint32_t)current_Cb, (current_width*current_height>>2));
  dma_flush_inv( (uint32_t)current_Cr, (current_width*current_height>>2));

  // flush output data
  dma_flush_inv( (uint32_t)controller->cache, (controller->width>>4) * (controller->height>>4) * sizeof(MB_p264_t));

  return C_OK;
}

static void intra_pred_4x4_p6_to_list (uint32_t intra_pred_4x4_0, uint32_t intra_pred_4x4_1, intra_4x4_mode_t* out)
{
  uint32_t i;
  for (i=0;i<8;i++)
  {
    *out++ = (intra_4x4_mode_t) (intra_pred_4x4_0 & 0x0F);
    intra_pred_4x4_0 = intra_pred_4x4_0>>4;
  }
  for (i=0;i<8;i++)
  {
    *out++ = (intra_4x4_mode_t) (intra_pred_4x4_1 & 0x0F);
    intra_pred_4x4_1 = intra_pred_4x4_1>>4;
  }
}

void P6_get_MV (MV_XY_t *mv,uint32_t num_mv, uint32_t* raw_mv_tab)
{
  uint32_t mv_2k_result;
  mv_2k_result = raw_mv_tab[(num_mv&0x0E)>>1];
  if ((num_mv&0x01) == 0)
  {
    mv->x = (mv_2k_result&0x000000FF)>>0;
    mv->x >>= 2;
    mv->y = (mv_2k_result&0x0000FF00)>>8;
    mv->y >>= 2;
  }
  else
  {
    mv->x = (mv_2k_result&0x00FF0000)>>16;
    mv->x >>= 2;
    mv->y = (mv_2k_result&0xFF000000)>>24;
    mv->y >>= 2;
  }
}


// encode num_macro_blocks MB
C_RESULT video_p264_encode_MB(uint32_t num_macro_blocks, video_macroblock_t* macroblock ,int32_t qp)
{
  if (num_macro_blocks > 0)
  {
    // launch (num_macro_block - num_pending) encoding
    MB_p264_t* MB_P6 = (MB_p264_t*)macroblock->data;

    // set qp
    CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_QP, &qp))

    // set output data addr
    p264_p6_output_buf_t output_buf;
    output_buf.phys_output = (uint32_t)dma_virt2phy(MB_P6);
    output_buf.reg_output =  h264_raw_result;
    CHECK_P264_IOCTL(ioctl(h264_fd, P264_SET_OUTPUT_BUF, &output_buf))

    // launch encoding
    CHECK_P264_IOCTL(ioctl(h264_fd, P264_ENCODE_NEXT_MB, &num_macro_blocks))

    return C_OK;
  }
  else
    return C_FAIL;
}

// get encoded num_macro_blocks MB
int32_t video_p264_get_encoded_MB(uint32_t num_macro_blocks, video_macroblock_t* macroblock)
{
  // retrieve and process available MBs
  if (num_macro_blocks > 0)
  {
    // wait hardware completion
    CHECK_P264_IOCTL(ioctl(h264_fd, P264_WAIT_ENCODE, &num_macro_blocks))

    if (num_macro_blocks != 0)
    {
      // at this point num_macro_block h264_raw_result is filled with me_result and MV
      uint32_t i;
      for (i=0;i<num_macro_blocks;i++)
      {
        MB_p264_t* MB_P6 = (MB_p264_t*)(macroblock[i].data);

        if (I_encoding == FALSE)
        {
          // save MB partition and MV
          if (INTER_16_8_PARTITION(h264_raw_result[i].me_result) == 0)
          {
            // 16x16 partition
            // save partition mode
            macroblock[i].inter_partition_mode[0] = INTER_PART_16x16;
            // save corresponding MV
            P6_get_MV(&(macroblock[i].inter_MV[0]),0,h264_raw_result[i].pred_result);
            macroblock[i].nb_partition = 1;
          }
          else if (INTER_16_8_PARTITION(h264_raw_result[i].me_result) == 1)
          {
            // 16x8 partition
            // save partition mode
            macroblock[i].inter_partition_mode[0] = INTER_PART_16x8;
            macroblock[i].inter_partition_mode[1] = INTER_PART_16x8;
            // save corresponding MV
            P6_get_MV(&macroblock[i].inter_MV[0],0,h264_raw_result[i].pred_result);
            P6_get_MV(&macroblock[i].inter_MV[1],8,h264_raw_result[i].pred_result);
            macroblock[i].nb_partition = 2;
          }
          else if (INTER_16_8_PARTITION(h264_raw_result[i].me_result) == 2)
          {
            // 16x8 partition
            // save partition mode
            macroblock[i].inter_partition_mode[0] = INTER_PART_8x16;
            macroblock[i].inter_partition_mode[1] = INTER_PART_8x16;
            // save corresponding MV
            P6_get_MV(&macroblock[i].inter_MV[0],0,h264_raw_result[i].pred_result);
            P6_get_MV(&macroblock[i].inter_MV[1],4,h264_raw_result[i].pred_result);
            macroblock[i].nb_partition = 2;
          }
          // 8x8 8x4 4x8 4x4 cases missing
          else
            PRINT ("wrong partition (or not supported) result %d\n",INTER_16_8_PARTITION(h264_raw_result[i].me_result));

          // patch DC chroma coeff
          // P6 h264 IP performs a zigzag on DC coeff. It should not.
          MB_P6->inter.DC_U[3] = MB_P6->inter.dummy_DC_U[0];
          MB_P6->inter.DC_V[3] = MB_P6->inter.dummy_DC_V[0];

    #ifdef H264_P6_PFRAME_DEBUG
          uint32_t i;
          for (i=0;i<prev_macroblock->nb_partition;i++)
          {
            PRINT ("partition[%d]=%d - MV (%d,%d)\n",i,macroblock[i].inter_partition_mode[i],macroblock[i].inter_MV[i].x,macroblock[i].inter_MV[i].y);
          }
    #endif
        }
        else
        {
    #ifdef H264_P6_DEBUG
          PRINT ("me result 0x%x\n",h264_raw_result[i].me_result);
          PRINT ("--> Y intra mode %d\n",(h264_raw_result[i].me_result&0x1F));
          PRINT ("--> Chroma intra mode %d\n",CHROMA_MODE(h264_raw_result[i].me_result));
    #endif

          if (IS_INTRA_4x4(h264_raw_result[i].me_result))
          {
            // patch DC chroma coeff
            // P6 h264 IP performs a zigzag on DC coeff. It should not.
            MB_P6->intra_4x4.DC_U[3] = MB_P6->intra_4x4.dummy_DC_U[0];
            MB_P6->intra_4x4.DC_V[3] = MB_P6->intra_4x4.dummy_DC_V[0];
            // set macroblock type
            macroblock[i].intra_type = INTRA_4x4;
            // save 4x4 intra luma result
            intra_pred_4x4_p6_to_list (h264_raw_result[i].intra_pred_4x4_0, h264_raw_result[i].intra_pred_4x4_1, macroblock[i].intra_4x4_mode);
    #ifdef H264_P6_DEBUG
            PRINT ("intra 4x4 pred result 0x%x\n",h264_raw_result[i].intra_pred_4x4_0);
            PRINT ("intra 4x4 pred result 0x%x\n",h264_raw_result[i].intra_pred_4x4_1);
    #endif
          }
          else
          {
            // patch DC chroma coeff
            // P6 h264 IP performs a zigzag on DC coeff. It should not.
            MB_P6->intra_16x16.DC_U[3] = MB_P6->intra_16x16.dummy_DC_U[0];
            MB_P6->intra_16x16.DC_V[3] = MB_P6->intra_16x16.dummy_DC_V[0];
            // set macroblock type
            macroblock[i].intra_type = INTRA_16x16;
            // save 16x16 luma result
            macroblock[i].intra_luma_16x16_mode = INTRA_16x16_MODE(h264_raw_result[i].me_result);
    #ifdef H264_P6_DEBUG
            printf("luma 16x16 pred %d\n",INTRA_16x16_MODE(h264_raw_result[i].me_result));
    #endif

            // zagzig DC luma (P6 bug fix)
            int16_t tmp_zagzig[16];
            zagzig_4x4 (MB_P6->intra_16x16.DC_Y, tmp_zagzig);
            vp_os_memcpy(MB_P6->intra_16x16.DC_Y,tmp_zagzig,16*sizeof(int16_t));
          }

          // save chroma mode
          macroblock[i].intra_chroma_8x8_mode = CHROMA_MODE(h264_raw_result[i].me_result);
        }
      }
    }
  }
  return num_macro_blocks;
}
#endif


C_RESULT video_p264_p6_close(void)
{
  if (h264_ref_frame != NULL)
    dma_free (h264_ref_frame);
  h264_ref_frame = NULL;
  if (h264_deb_frame != NULL)
    dma_free (h264_deb_frame);

  if (h264_fd)
    close(h264_fd);

  h264_deb_frame = NULL;
  current_Y  = NULL;
  current_Cb = NULL;
  current_Cr = NULL;
  current_width = 0;
  current_height = 0;
  return C_OK;
}
