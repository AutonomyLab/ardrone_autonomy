#include <stdio.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>
#include <VP_Stages/vp_stages_i_camif.h>

//
// Buffer to Picture conversion
//
static int32_t copy_input_to_buffer( uint8_t* buffer, int32_t input_size, int32_t max_size, vp_stages_buffer_to_picture_config_t *cfg )
{
  int32_t size_to_copy;

  size_to_copy = input_size;
  if( size_to_copy > max_size )
    size_to_copy = max_size;
  vp_os_memcpy( buffer, cfg->input_ptr, size_to_copy );

  if( cfg != NULL )
  {
    cfg->cumulated_size += size_to_copy;
    cfg->input_ptr      += size_to_copy;
  }

  return size_to_copy;
}


C_RESULT vp_stages_buffer_to_picture_open(vp_stages_buffer_to_picture_config_t *cfg)
{
  cfg->num_picture_decoded = 0;

  return C_OK;
}


C_RESULT vp_stages_buffer_to_picture_transform(vp_stages_buffer_to_picture_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;
    out->buffers      = (uint8_t**) &cfg->picture;
    out->indexBuffer  = 0;
    out->lineSize     = 0;

    out->status = VP_API_STATUS_PROCESSING;

    cfg->y_buf_ptr      = cfg->picture->y_buf;
    if(!cfg->luma_only)
    {
      cfg->cr_buf_ptr   = cfg->picture->cr_buf;
      cfg->cb_buf_ptr   = cfg->picture->cb_buf;
    }
    cfg->cumulated_size = 0;
    cfg->input_ptr      = NULL;

    if( cfg->custom_data_size > 0 )
    {
      cfg->custom_data_ptr = vp_os_malloc( cfg->custom_data_size );
    }
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

    y_size  = cfg->block_mode_enable ? cfg->y_blockline_size : cfg->y_buffer_size;
    c_size  = cfg->luma_only ? 0 : y_size / 4;

    while(in->size > 0 && cfg->y_current_size < cfg->y_buffer_size)
    {
      if( in->size > 0 && cfg->cumulated_size < y_size )
      {
        copied_size = copy_input_to_buffer( cfg->y_buf_ptr, in->size, y_size - cfg->cumulated_size, cfg );

        cfg->y_buf_ptr += copied_size;
        in->size       -= copied_size;
      }

      if(!cfg->luma_only)
      {
        if( in->size > 0 && cfg->cumulated_size >= y_size && (cfg->cumulated_size < y_size + c_size) )
        {
          copied_size = copy_input_to_buffer( cfg->cb_buf_ptr, in->size, y_size + c_size - cfg->cumulated_size, cfg );

          cfg->cb_buf_ptr += copied_size;
          in->size        -= copied_size;
        }

        if( in->size > 0 && (cfg->cumulated_size >= y_size + c_size) && (cfg->cumulated_size < y_size + 2*c_size) )
        {
          copied_size = copy_input_to_buffer( cfg->cr_buf_ptr, in->size, y_size + 2*c_size - cfg->cumulated_size, cfg );

          cfg->cr_buf_ptr += copied_size;
          in->size        -= copied_size;
        }
      }

      if( cfg->cumulated_size == y_size + 2*c_size )
      {
        cfg->cumulated_size  = 0;
        cfg->y_current_size += y_size;
      }
    }

    if( cfg->custom_data_size > 0 && cfg->y_current_size >= cfg->y_buffer_size )
    {
      // we got one picture (handle case 1)
      if( in->size > 0 && cfg->custom_data_read != cfg->custom_data_size )
      {
        copied_size = copy_input_to_buffer( cfg->custom_data_ptr + cfg->custom_data_read,
                                            in->size, cfg->custom_data_size - cfg->custom_data_read,
                                            cfg );

        cfg->custom_data_read += copied_size;
        in->size -= copied_size;

        cfg->y_current_size += copied_size;
        cfg->cumulated_size  = 0;
      }
    }

    // All buffers are full but there's still data
    if( in->size > 0 )
      out->status = VP_API_STATUS_STILL_RUNNING;

    if( cfg->y_current_size == (cfg->y_buffer_size+cfg->custom_data_size) )
    {
      // we got one picture (handle case 1)
      out->size = 1;

      cfg->num_picture_decoded++;

      cfg->custom_data_read = 0;
      cfg->y_current_size   = 0;
      cfg->y_buf_ptr        = cfg->picture->y_buf;
      if(!cfg->luma_only)
      {
        cfg->cb_buf_ptr   = cfg->picture->cb_buf;
        cfg->cr_buf_ptr   = cfg->picture->cr_buf;
      }

      if( cfg->custom_data_handler != NULL )
      {
        cfg->custom_data_handler( (void*)&cfg->custom_data_ptr[0], cfg->custom_data_size );
      }
    }
  }

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}


C_RESULT vp_stages_buffer_to_picture_close(vp_stages_buffer_to_picture_config_t *cfg)
{
  return C_OK;
}



//
// Picture to Buffer conversion
//
C_RESULT vp_stages_picture_to_buffer_open(vp_stages_picture_to_buffer_config_t *cfg)
{
  return C_OK;
}

C_RESULT vp_stages_picture_to_buffer_transform(vp_stages_picture_to_buffer_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  int32_t y_size, c_size;

  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;

    cfg->y_buffer_size     = cfg->picture->width*cfg->picture->height;
    cfg->y_blockline_size  = cfg->picture->width*CAMIF_BLOCKLINES; // each blockline have 16 lines

    y_size  = cfg->block_mode_enable ? cfg->y_blockline_size : cfg->y_buffer_size;
    c_size  = cfg->luma_only ? 0 : y_size / 4;

    // We alloc an array big enough to hold all ours data
    out->buffers      = (uint8_t**) vp_os_malloc( (y_size+2*c_size+cfg->custom_data_size)*sizeof(uint8_t) + sizeof(uint8_t*));
    out->indexBuffer  = 0;
    out->status       = VP_API_STATUS_PROCESSING;

    out->buffers[0]   = (uint8_t *)(out->buffers+1);
  }
  else
  {
    y_size  = cfg->block_mode_enable ? cfg->y_blockline_size : cfg->y_buffer_size;
    c_size  = cfg->luma_only ? 0 : y_size / 4;
  }

  out->size = 0;

  if( out->status == VP_API_STATUS_PROCESSING && in->size > 0 )
  {
    uint8_t *y_ptr;

    y_ptr = cfg->picture->y_buf;

    if( cfg->block_mode_enable )
    {
      // Find blockline
      y_ptr += cfg->picture->blockline*y_size;
    }

    vp_os_memcpy( out->buffers[0], y_ptr, y_size );
    out->size = y_size;

    if( !cfg->luma_only )
    {
      uint8_t *cb_ptr, *cr_ptr;

      cb_ptr = cfg->picture->cb_buf;
      cr_ptr = cfg->picture->cr_buf;

      if( cfg->block_mode_enable )
      {
        cb_ptr += cfg->picture->blockline*c_size;
        cr_ptr += cfg->picture->blockline*c_size;
      }

      vp_os_memcpy( out->buffers[0] + y_size, cb_ptr, c_size);
      vp_os_memcpy( out->buffers[0] + y_size + c_size, cr_ptr, c_size);

      out->size += 2*c_size;
    }

    if( cfg->custom_data_handler && cfg->picture->complete )
    {
      cfg->custom_data_handler( out->buffers[0] + out->size, cfg->custom_data_size );

      out->size += cfg->custom_data_size;
    }
  }

  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}

C_RESULT vp_stages_picture_to_buffer_close(vp_stages_picture_to_buffer_config_t *cfg)
{
  return C_OK;
}
