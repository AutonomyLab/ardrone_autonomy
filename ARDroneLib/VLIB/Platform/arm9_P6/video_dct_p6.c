#include "libuiomap.h"
#include "dma_malloc.h"
#include "video_config.h"
#include "video_dct_p6.h"
#include "video_utils_p6.h"
#include <VLIB/video_quantizer.h>
#include <VP_Os/vp_os_malloc.h>
#include <stdio.h>
#include <VP_Os/vp_os_print.h>

#if (MAX_NUM_MACRO_BLOCKS_PER_CALL > 10)
# error "MAX_NUM_MACRO_BLOCKS_PER_CALL must not be greater than 10 on P6"
#endif

#if (VIDEO_DCT_USE_INTRAM != 0)
#error "intram not supported"
#endif

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)
#error "dct_interrupt not supported"
#endif


struct uiomap ui_h264_reg;
struct uiomap ui_sysc_reg;

static uint16_t* local_quant_table;
static uint32_t local_address_quant_table;
static int32_t actual_quant;

static uint32_t dct_in_progress;

C_RESULT video_dct_p6_init(void)
{
  int32_t err;
  //reset some variables
  local_quant_table=NULL;
  local_address_quant_table=0;
  actual_quant = -1;

  err= uiomap_ioremap(&ui_h264_reg, 0xd0060000, 0x10000);
  if (err)
  {
	PRINT("error remapping h264 regs: %s\n", strerror(-err));
	return C_FAIL;
  }
  err= uiomap_ioremap(&ui_sysc_reg, 0xd0000000, 0x10);
  if (err)
  {
    PRINT("error activating h264 clock : %s\n", strerror(-err));
    return C_FAIL;
  }

  // allocate local buffer
  if (local_quant_table == NULL)
  {
    local_quant_table = (uint16_t*) dma_malloc (MCU_BLOCK_SIZE*sizeof (uint16_t));
  }

  if (local_quant_table == NULL)
  {
	  PRINT ("dct p6 error during local buffers allocation\n");
	  return C_FAIL;
  }

  local_address_quant_table = (int32_t) dma_virt2phy(local_quant_table);
  if (local_address_quant_table == 0)
  {
  	  PRINT ("dct p6 error when retrieving local physic address\n");
  	  return C_FAIL;
  }

  // active h264 clock
  uint32_t value;
  value = uiomap_readl(&ui_sysc_reg, 0x0C);
  uiomap_writel(&ui_sysc_reg,value | (1<<4) ,0x0C);

  // reset IP
  uiomap_writel(&ui_h264_reg, 0x1, DCT_RESET);

  // registers configuration
  //uiomap_writel(&ui_h264_reg, 0, DCT_CONFIG); // DCT
  uiomap_writel(&ui_h264_reg, (1<<1)|(1<<6), DCT_CONFIG); // DCT+Quantization
  uiomap_writel (&ui_h264_reg, 0, DCT_ITEN ); // no interrupt
  uiomap_writel(&ui_h264_reg, 0x33, DCT_DMA);
  uiomap_writel(&ui_h264_reg, 0x33, DCT_DMAINT);
  uiomap_writel(&ui_h264_reg, local_address_quant_table, DCT_Q_ADDR); // quantization table

  uiomap_writel (&ui_h264_reg,0, DCT_ORIG_CU_ADDR);
  uiomap_writel (&ui_h264_reg,0, DCT_ORIG_CV_ADDR);
  uiomap_writel (&ui_h264_reg,0, DCT_DEST_CU_ADDR);
  uiomap_writel (&ui_h264_reg,0, DCT_DEST_CV_ADDR);
  uiomap_writel (&ui_h264_reg,0, DCT_LINEOFFSET);

  dct_in_progress = 0;


  return C_OK;
}

#ifdef HAS_FDCT_QUANT_COMPUTE
int16_t* video_fdct_quant_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks,int32_t quant)
{
  uint32_t status = 0;
  uint32_t  ctrl = 0;
  uint32_t  itack = 0;
  num_macro_blocks *= 6;

  if( dct_in_progress > 0 )
  {
     // Check if we have to wait for a previous run to complete
	  do
	  {
		  status = uiomap_readl(&ui_h264_reg,DCT_STATUS);
	  }
     while( status == 0 );
  }

  switch( status )
  {
    case DCT_STATUS_END_OK:
      itack = DCT_ITACK_END_OK;
      break;

    /*case DCT_STATUS_ERROR:
        itack = DCT_ITACK_ERROR;
      break;*/

    default:
      PRINT("dct status 0x%X\n",status);
      itack = 0;
      break;
  }


  // check if we have to update quantization table
  if (actual_quant != quant)
  {
	  actual_quant = quant;

	// VLIB backward compatibility
	if (quant == TABLE_QUANTIZATION)
	  quant = TBL_QUANT_QUALITY;

    // build quantization table
	// P6 DCT quantization table = 2^14/iquant_tab
    uint32_t i;
    uint16_t* p = local_quant_table;
    for (i=0;i<64;i++)
      *p++ = (2<<14) / QUANT_I(i,quant);
    dma_flush_inv((uint32_t)local_quant_table, MCU_BLOCK_SIZE*sizeof(int16_t));
  }

  ctrl |= ((num_macro_blocks - 1) & 0x3F ) << 1;
  ctrl |= DCT_CTRLMODE_FDCT;

  uiomap_writel (&ui_h264_reg,(uint32_t)dma_virt2phy(in),  DCT_ORIG_Y_ADDR);
  uiomap_writel (&ui_h264_reg,(uint32_t)dma_virt2phy(out), DCT_DEST_Y_ADDR);

  uiomap_writel (&ui_h264_reg, ctrl, DCT_CONTROL);

  dma_flush_inv( (uint32_t)in, 2*num_macro_blocks*MCU_BLOCK_SIZE);
  uiomap_writel(&ui_h264_reg, 0, DCT_START);
  dma_flush_inv( (uint32_t)out, 2*num_macro_blocks*MCU_BLOCK_SIZE);
  dct_in_progress = 1;

  return out + MCU_BLOCK_SIZE*num_macro_blocks;
}
#endif // HAS_FDCT_COMPUTE

C_RESULT video_dct_p6_close(void)
{
	uiomap_iounmap(&ui_h264_reg);
	uiomap_iounmap(&ui_sysc_reg);
	dma_free(local_quant_table);
	return C_OK;
}

#ifdef HAS_IDCT_COMPUTE
#error "hardware idct not yet supported on P6"
#endif // HAS_IDCT_COMPUTE
