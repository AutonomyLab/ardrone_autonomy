#include <cyg/io/io.h>
#include <cyg/hal/hal_io.h>         // I/O macros
#include <cyg/hal/drv_api.h>        // cyg_interrupt
#include <cyg/hal/hal_platform_ints.h>
#include <cyg/kernel/diag.h>

#include "video_config.h"
#include "video_dct_p5p.h"
#include "video_utils_p5p.h"

#if (MAX_NUM_MACRO_BLOCKS_PER_CALL > 10)
# error "MAX_NUM_MACRO_BLOCKS_PER_CALL must not be greater than 10 on P5P"
#endif

#if (VIDEO_DCT_USE_INTRAM == 0)
#include <cyg/utils/mmu.h>
#endif

static cyg_uint32 dct_in_progress;

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)

static cyg_interrupt dct_interrupt;
static cyg_handle_t dct_interrupt_handle;

//! DCT driver interrupt service routine (ISR)
static cyg_uint32 dct_isr( cyg_vector_t vector, cyg_addrword_t data )
{
  cyg_drv_interrupt_mask(vector);
  cyg_drv_interrupt_acknowledge(vector);

  return CYG_ISR_CALL_DSR;  // request DSR
}

//! DCT driver deffered service routine (DSR)
static void dct_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
  cyg_uint32 itack;

  RTMON_USTOP( VIDEO_VLIB_DCT_COMPUTE_UEVENT );

  switch( dct_read_reg( DCT_STATUS ) )
  {
    case DCT_STATUS_END_OK:
      itack = DCT_ITACK_END_OK;
      break;

    case DCT_STATUS_ERROR:
      diag_printf("[DCT] DMA access failure\n");
      itack = DCT_ITACK_ERROR;
      break;

    default:
      itack = 0;
      break;
  }

  dct_write_reg( DCT_ITACK,  itack );

  cyg_drv_interrupt_unmask(vector);
}
#endif // VIDEO_DCT_INTERRUPT_ENABLE


C_RESULT video_dct_p5p_init(void)
{
  cyg_uint32 value;

  HAL_READ_UINT32( PARROT5_SYS + _SYS_CEN, value);
  HAL_WRITE_UINT32( PARROT5_SYS + _SYS_CEN, (value | SYS_CEN_CAMIFCLK) );

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)
  diag_printf("Configuring DCT with interrupt acknowledge\n");
  cyg_drv_interrupt_create( CYGNUM_HAL_INTERRUPT_DCT,
                            1,
                            (cyg_addrword_t) 0,
                            dct_isr,
                            dct_dsr,
                            &dct_interrupt_handle,
                            &dct_interrupt );

  cyg_drv_interrupt_attach(dct_interrupt_handle);

  cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_DCT);

  dct_write_reg( DCT_ITEN , DCT_ITEN_END_OK | DCT_ITEN_ERROR );
#else
  dct_write_reg ( DCT_ITEN,  0 );
#endif // VIDEO_DCT_INTERRUPT_ENABLE

  dct_in_progress = 0;

  return C_OK;
}

#ifdef HAS_FDCT_COMPUTE

int16_t* video_fdct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks)
{
  cyg_uint32  ctrl = 0;
#if (VIDEO_DCT_INTERRUPT_ENABLE == 0)
  cyg_uint32  itack = 0;
#endif

  num_macro_blocks *= 6;

#if (VIDEO_DCT_USE_INTRAM == 0)
//   arm_mmu_clean_dcache_range((CYG_ADDRESS)in, num_macro_blocks*MCU_BLOCK_SIZE*2 );
//   arm_mmu_invalidate_cache_range((CYG_ADDRESS)in, num_macro_blocks*MCU_BLOCK_SIZE*2 );
//
//   arm_mmu_clean_dcache_range((CYG_ADDRESS)out, num_macro_blocks*MCU_BLOCK_SIZE*2 );
//   arm_mmu_invalidate_cache_range((CYG_ADDRESS)out, num_macro_blocks*MCU_BLOCK_SIZE*2 );

  arm_mmu_flush_dcache();
#endif

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)
  cyg_drv_dsr_lock();
#else

  if( dct_in_progress > 0 )
  {
    RTMON_USTART( VIDEO_VLIB_DCT_WAIT_UEVENT );

    // Check if we have to wait for a previous run to complete
    while( dct_read_reg( DCT_STATUS ) == 0 );

    RTMON_USTOP( VIDEO_VLIB_DCT_WAIT_UEVENT );
  }

  switch( dct_read_reg( DCT_STATUS ) )
  {
    case DCT_STATUS_END_OK:
      // diag_printf("DCT_STATUS_END_OK\n");
      itack = DCT_ITACK_END_OK;
      break;

    case DCT_STATUS_ERROR:
      diag_printf("[DCT] DMA access failure\n");
      itack = DCT_ITACK_ERROR;
      break;

    default:
      itack = 0;
      break;
  }

  if( itack > 0 )
    dct_write_reg( DCT_ITACK,  itack ); // Acknowledge previous run

#endif // VIDEO_DCT_INTERRUPT_ENABLE

  ctrl |= ((num_macro_blocks - 1) & 0x3F ) << 1;
  ctrl |= DCT_CTRLMODE_FDCT;

  dct_write_reg ( DCT_ORIG_Y_ADDR , arm_mmu_v2p( (cyg_addrword_t)in ) );
  dct_write_reg ( DCT_ORIG_CU_ADDR, 0 );
  dct_write_reg ( DCT_ORIG_CV_ADDR, 0 );

  dct_write_reg ( DCT_DEST_Y_ADDR , arm_mmu_v2p( (cyg_addrword_t)out ) );
  dct_write_reg ( DCT_DEST_CU_ADDR, 0 );
  dct_write_reg ( DCT_DEST_CV_ADDR, 0 );

  dct_write_reg ( DCT_LINEOFFSET, 0 );

  dct_write_reg ( DCT_CONTROL, ctrl );

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)
  cyg_drv_dsr_unlock();

  RTMON_USTART( VIDEO_VLIB_DCT_COMPUTE_UEVENT );
#endif

  dct_in_progress = 1;

  return out + MCU_BLOCK_SIZE*num_macro_blocks;
}

#endif // HAS_FDCT_COMPUTE

#ifdef HAS_IDCT_COMPUTE

int16_t* video_idct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks)
{
  cyg_uint32  ctrl = 0;
#if (VIDEO_DCT_INTERRUPT_ENABLE == 0)
  cyg_uint32  itack = 0;
#endif

#if (VIDEO_DCT_USE_INTRAM == 0)
  arm_mmu_flush_dcache();
#endif

  num_macro_blocks *= 6;

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)
  cyg_drv_dsr_lock();
#else

  if( dct_in_progress > 0 )
  {
    // Check if we have to wait for a previous run to complete
    while( dct_read_reg( DCT_STATUS ) == 0 );
  }

  switch( dct_read_reg( DCT_STATUS ) )
  {
    case DCT_STATUS_END_OK:
      // diag_printf("DCT_STATUS_END_OK\n");
      itack = DCT_ITACK_END_OK;
      break;

    case DCT_STATUS_ERROR:
      diag_printf("[DCT] DMA access failure\n");
      itack = DCT_ITACK_ERROR;
      break;

    default:
      itack = 0;
      break;
  }

  if( itack > 0 )
    dct_write_reg( DCT_ITACK,  itack ); // Acknowledge previous run

#endif // VIDEO_DCT_INTERRUPT_ENABLE

  ctrl |= ((num_macro_blocks - 1) & 0x3F ) << 1;
  ctrl |= DCT_CTRLMODE_IDCT;

  dct_write_reg ( DCT_ORIG_Y_ADDR , arm_mmu_v2p( (cyg_addrword_t)in ) );
  dct_write_reg ( DCT_ORIG_CU_ADDR, 0 );
  dct_write_reg ( DCT_ORIG_CV_ADDR, 0 );

  dct_write_reg ( DCT_DEST_Y_ADDR , arm_mmu_v2p( (cyg_addrword_t)out ) );
  dct_write_reg ( DCT_DEST_CU_ADDR, 0 );
  dct_write_reg ( DCT_DEST_CV_ADDR, 0 );

  dct_write_reg ( DCT_LINEOFFSET, 0 );

  dct_write_reg ( DCT_CONTROL, ctrl );

#if (VIDEO_DCT_INTERRUPT_ENABLE == 1)
  cyg_drv_dsr_unlock();

  RTMON_USTART( VIDEO_VLIB_DCT_COMPUTE_UEVENT );
#endif

  dct_in_progress = 1;

  return out + MCU_BLOCK_SIZE*num_macro_blocks;
}

#endif // HAS_IDCT_COMPUTE
