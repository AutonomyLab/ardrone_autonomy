#include <RAMCODE/ramcode.h>
#include "video_gen_sad.h"


sad_fn gen_sad( uint32_t* buffer, uint32_t buffer_length,
                uint32_t bytes_per_entry_ref, uint32_t bytes_per_entry_src,
                uint16_t offset_ref, uint16_t offset_src,
                uint32_t square_length )
{

  /* Register usage in generated code
      lr/r14 : sad accumulation
      r0 : ref - r2 : data read with r0
      r1 : src - r3 : data read with r1
      ip : counter
  */

  uint32_t *b;
  int8_t *buffer_save, *buffer_current;
  sad_fn sad = (sad_fn) NULL;

  uint32_t prologue[]   = {
                            0xe52de004, // str lr, [sp, #-4]!
                            0xe3a0c000, // mov ip, 0x0
                            0xe3a0e000  // mov lr, 0x0
                          };

  uint32_t epilogue[]   = {
                            0xe1a0000e, // mov r0, lr
                            0xe49df004  // ldr pc, [sp], #4
                          };

  uint32_t sad_acc[]    = {
                            0xe0732002, // rsbs r2, r3, r2
                            0xb2622000, // rsblt r2, r2, #0
                            0xe08ee002  // add lr, lr, r2
                          };

  uint32_t dec_count[]  = {
                            0xe25cc001  // subs ip, ip, #1
                          };

  offset_ref += ( 1 - square_length );
  offset_src += ( 1 - square_length );

  if( offset_ref < 1024 && offset_src < 1024 )
  {
    sad = (sad_fn) buffer;

    b = &buffer[0];
    buffer = emit_buffer( buffer, prologue, sizeof(prologue) );
    b[1] |= square_length;

    switch( bytes_per_entry_ref )
    {
      case MEM_FMT_8:
        buffer = emit_load8_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_0, ARM_REG_2, 1 );
        break;

      case MEM_FMT_16:
        buffer = emit_load16_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_0, ARM_REG_2, 1 );
        break;

      case MEM_FMT_32:
        buffer = emit_load32_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_0, ARM_REG_2, 1 );
        break;
    }

    switch( bytes_per_entry_src )
    {
      case MEM_FMT_8:
        buffer = emit_load8_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_1, ARM_REG_3, 1 );
        break;

      case MEM_FMT_16:
        buffer = emit_load16_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_1, ARM_REG_3, 1 );
        break;

      case MEM_FMT_32:
        buffer = emit_load32_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_1, ARM_REG_3, 1 );
        break;
    }

    buffer_save = (int8_t*)&buffer[0];

    square_length --;

    while( square_length > 0 )
    {
      emit_buffer( buffer, sad_acc, sizeof(sad_acc) );

      switch( bytes_per_entry_ref )
      {
        case MEM_FMT_8:
          buffer = emit_load8_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_0, ARM_REG_2, 1 );
          break;

        case MEM_FMT_16:
          buffer = emit_load16_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_0, ARM_REG_2, 1 );
          break;

        case MEM_FMT_32:
          buffer = emit_load32_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_0, ARM_REG_2, 1 );
          break;
      }

      switch( bytes_per_entry_src )
      {
        case MEM_FMT_8:
          buffer = emit_load8_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_1, ARM_REG_3, 1 );
          break;

        case MEM_FMT_16:
          buffer = emit_load16_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_1, ARM_REG_3, 1 );
          break;

        case MEM_FMT_32:
          buffer = emit_load32_pi( COND_FIELD_ALWAYS, buffer, ARM_REG_1, ARM_REG_3, 1 );
          break;
      }

      square_length --;
    }

    buffer = emit_buffer( buffer, sad_acc, sizeof(sad_acc) );
    buffer = emit_buffer( buffer, dec_count, sizeof(dec_count) );

    switch( bytes_per_entry_ref )
    {
      case MEM_FMT_8:
        buffer = emit_load8_pi( COND_FIELD_NE, buffer, ARM_REG_0, ARM_REG_2, offset_ref );
        break;

      case MEM_FMT_16:
        buffer = emit_load16_pi( COND_FIELD_NE, buffer, ARM_REG_0, ARM_REG_2, offset_ref );
        break;

      case MEM_FMT_32:
        buffer = emit_load32_pi( COND_FIELD_NE, buffer, ARM_REG_0, ARM_REG_2, offset_ref );
        break;
    }

    switch( bytes_per_entry_src )
    {
      case MEM_FMT_8:
        buffer = emit_load8_pi( COND_FIELD_NE, buffer, ARM_REG_1, ARM_REG_3, offset_src );
        break;

      case MEM_FMT_16:
        buffer = emit_load16_pi( COND_FIELD_NE, buffer, ARM_REG_1, ARM_REG_3, offset_src );
        break;

      case MEM_FMT_32:
        buffer = emit_load32_pi( COND_FIELD_NE, buffer, ARM_REG_1, ARM_REG_3, offset_src );
        break;
    }

    buffer_current = (int8_t*)&buffer[0];
    buffer = emit_branch( COND_FIELD_NE, buffer, buffer_save - buffer_current - 8 );

    buffer = emit_buffer( buffer, epilogue, sizeof(epilogue) );

    arm_mmu_flush_dcache();
  }

  return sad;
}
