#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>

#include <VLIB/video_controller.h>
#include <VLIB/video_codec.h>
#include <VLIB/video_picture.h>
#include <VP_Os/vp_os_print.h>
#include <VLIB/Platform/video_config.h>

extern C_RESULT video_utils_set_format( uint32_t width, uint32_t height );

C_RESULT video_controller_update( video_controller_t* controller, bool_t complete )
{
  video_codec_t* video_codec = controller->video_codec;

  controller->current_bits  += controller->in_stream.index << 5; // Index is an index in a int32_t array (32 bits)

  if( complete )
  {
    controller->num_frames   += 1;
    controller->output_bits   = controller->current_bits;
    controller->current_bits  = 0;
  }

  video_codec->update( controller );

  return C_OK;
}

C_RESULT video_controller_set_mode( video_controller_t* controller, uint32_t mode )
{
  controller->mode = mode;

  return C_OK;
}

C_RESULT video_controller_set_bitrate( video_controller_t* controller, uint32_t target )
{
  controller->target_bitrate = target;

  return C_OK;
}

C_RESULT video_controller_set_target_size( video_controller_t* controller, uint32_t target )
{
  controller->target_size = target;

  return C_OK;
}

static void video_realloc_buffers( video_controller_t* controller, int32_t num_prev_blockline )
{
  int32_t i, j;
  int16_t* cache;
  video_gob_t* gob;
  video_macroblock_t* mb;

  // Realloc global cache (YUV420 format)
  if (controller->codec_type == P264_CODEC)
  { 
    // realloc nb_macroblocks (MB_p264_t type) per picture
    controller->cache = (int16_t*) vp_os_aligned_realloc( controller->cache,
                                                          (controller->width>>4) * (controller->height>>4) * sizeof(MB_p264_t),
                                                          VLIB_ALLOC_ALIGN );
    
    vp_os_memset( controller->cache, 0, (controller->width>>4) * (controller->height>>4) * sizeof(MB_p264_t));
  }
  else
  {
    controller->cache = (int16_t*) vp_os_aligned_realloc( controller->cache,
                                                          3 * controller->width * controller->height * sizeof(int16_t) / 2,
                                                          VLIB_ALLOC_ALIGN );
                                                          
    vp_os_memset( controller->cache, 0, 3 * controller->width * controller->height * sizeof(int16_t) / 2 );
  }

  // Realloc buffers
  cache = controller->cache;
  i = controller->num_blockline;
  j = controller->mb_blockline;

  controller->gobs = (video_gob_t*) vp_os_realloc(controller->gobs, i * sizeof(video_gob_t));
  vp_os_memset( controller->gobs, 0, i * sizeof(video_gob_t) );

  gob = &controller->gobs[0];
  gob->macroblocks = (video_macroblock_t*) vp_os_realloc( gob->macroblocks, i * j * sizeof(video_macroblock_t));
  vp_os_memset( gob->macroblocks, 0, i * j * sizeof(video_gob_t) );

  video_macroblock_t * macroblock_root = gob->macroblocks;

  for(; i > 0; i-- )
  {
    j = controller->mb_blockline;

    gob->macroblocks = macroblock_root;
    macroblock_root +=  controller->mb_blockline;

    mb = &gob->macroblocks[0];
    for(; j > 0; j-- )
    {
      mb->data = cache;

      if (controller->codec_type == P264_CODEC)
      {
        cache   += sizeof(MB_p264_t)/sizeof(mb->data[0]);
      }
      else
      {
        cache   += MCU_BLOCK_SIZE*6;
      }
      mb ++;
    }

    gob ++;
  }
}

C_RESULT video_controller_cleanup( video_controller_t* controller )
{
  video_gob_t* gob;

  if( controller->gobs != NULL )
  {
    gob = &controller->gobs[0];
    vp_os_free(gob->macroblocks);
    gob->macroblocks = NULL;
    vp_os_free( controller->gobs );
    controller->gobs = NULL;
  }

  if( controller->cache != NULL )
  {
    vp_os_aligned_free( controller->cache );
      controller->cache = NULL;
  }

  return C_OK;
}

C_RESULT video_controller_set_format( video_controller_t* controller, int32_t width, int32_t height )
{
  int32_t num_prev_blockline;

  VP_OS_ASSERT( (width != 0) && (height != 0) );

  if( width != controller->width || controller->height != height )
  {
    controller->resolution_changed = TRUE;
    controller->width   = width;
    controller->height  = height;

    num_prev_blockline = controller->num_blockline;

    controller->num_blockline = height >> 4;
    controller->mb_blockline  = width >> 4;

    video_realloc_buffers( controller, num_prev_blockline );

    video_utils_set_format( width, height );
  }
  else
    controller->resolution_changed = FALSE;

  return C_OK;
}

C_RESULT  video_controller_set_picture_type( video_controller_t* controller, uint32_t type )
{
  controller->picture_type = type;

  return C_OK;
}

C_RESULT  video_controller_set_motion_estimation( video_controller_t* controller, bool_t use_me )
{
  controller->use_me = use_me;

  return C_OK;
}
