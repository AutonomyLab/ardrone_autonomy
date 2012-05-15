#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <config.h>
#include <ardrone_tool/Video/buffer_to_picture_stage.h>

static int32_t copy_input_to_buffer( uint8_t* buffer, int32_t input_size, int32_t max_size, buffer_to_picture_config_t *cfg )
{
  int32_t size_to_copy;

  size_to_copy = input_size;
  if( size_to_copy > max_size )
    size_to_copy = max_size;
  vp_os_memcpy( buffer, cfg->input_ptr, size_to_copy );

  cfg->cumulated_size += size_to_copy;
  cfg->input_ptr      += size_to_copy;

  return size_to_copy;
}

C_RESULT buffer_to_picture_open(buffer_to_picture_config_t *cfg)
{
  cfg->num_picture_decoded = 0;

  return C_OK;
}

C_RESULT buffer_to_picture_transform(buffer_to_picture_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;
    out->buffers      = (int8_t**)(int8_t*) cfg->picture;
    out->indexBuffer  = 0;
    out->lineSize     = 0;

    out->status = VP_API_STATUS_PROCESSING;

    cfg->y_buf_ptr      = cfg->picture->y_buf;
#ifdef USE_VIDEO_YUV
    cfg->cr_buf_ptr     = cfg->picture->cr_buf;
    cfg->cb_buf_ptr     = cfg->picture->cb_buf;
#endif
    cfg->cumulated_size = 0;
    cfg->input_ptr      = NULL;
  }

  if( in->status == VP_API_STATUS_ENDED )
    out->status = in->status;

  if( out->status == VP_API_STATUS_PROCESSING )
    cfg->input_ptr = (uint8_t*)in->buffers[in->indexBuffer];

  if(out->status == VP_API_STATUS_PROCESSING || out->status == VP_API_STATUS_STILL_RUNNING)
  {
    int32_t copied_size, y_size, c_size = 0;
    // If out->size == 1 it means picture is ready
    out->size = 0;
    out->status = VP_API_STATUS_PROCESSING;

    y_size  = cfg->y_blockline_size;
#ifdef USE_VIDEO_YUV
    c_size  = cfg->y_blockline_size / 4;
#endif

    while(in->size > 0 && cfg->y_current_size != cfg->y_buffer_size)
    {
      if( in->size > 0 && cfg->cumulated_size < y_size )
      {
        copied_size = copy_input_to_buffer( cfg->y_buf_ptr, in->size, y_size - cfg->cumulated_size, cfg );

        cfg->y_buf_ptr += copied_size;
        in->size       -= copied_size;
      }

#ifdef USE_VIDEO_YUV
      if( in->size > 0 && cfg->cumulated_size >= y_size && cfg->cumulated_size < y_size + c_size )
      {
        copied_size = copy_input_to_buffer( cfg->cb_buf_ptr, in->size, y_size + c_size - cfg->cumulated_size, cfg );

        cfg->cb_buf_ptr += copied_size;
        in->size        -= copied_size;
      }

      if( in->size > 0 && cfg->cumulated_size >= y_size + c_size && cfg->cumulated_size < y_size + 2*c_size )
      {
        copied_size = copy_input_to_buffer( cfg->cr_buf_ptr, in->size, y_size + 2*c_size - cfg->cumulated_size, cfg );

        cfg->cr_buf_ptr += copied_size;
        in->size        -= copied_size;
      }
#endif

      if( cfg->cumulated_size == y_size + 2*c_size )
      {
        cfg->cumulated_size  = 0;
        cfg->y_current_size += cfg->y_blockline_size;
      }
    }

    // All buffers are full but there's still data
    if( in->size > 0 )
      out->status = VP_API_STATUS_STILL_RUNNING;

    if( cfg->y_current_size == cfg->y_buffer_size )
    {
      // we got one picture (handle case 1)
      out->size = 1;

      cfg->num_picture_decoded++;

      DEBUG_PRINT_SDK( "%d picture received\n", (int)cfg->num_picture_decoded );

      cfg->y_current_size = 0;
      cfg->y_buf_ptr      = cfg->picture->y_buf;
#ifdef USE_VIDEO_YUV
      cfg->cr_buf_ptr     = cfg->picture->cr_buf;
      cfg->cb_buf_ptr     = cfg->picture->cb_buf;
#endif
    }
  }

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}

C_RESULT buffer_to_picture_close(buffer_to_picture_config_t *cfg)
{
  return C_OK;
}

const vp_api_stage_funcs_t buffer_to_picture_funcs =
{
  NULL,
  (vp_api_stage_open_t)buffer_to_picture_open,
  (vp_api_stage_transform_t)buffer_to_picture_transform,
  (vp_api_stage_close_t)buffer_to_picture_close
};
