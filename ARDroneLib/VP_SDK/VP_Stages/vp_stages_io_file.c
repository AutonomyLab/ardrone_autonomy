/**
 *  \brief    VP Stages. File stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

//#define VERBOSE
///////////////////////////////////////////////
// INCLUDES

#ifdef FFMPEG_SUPPORT
#undef FFMPEG_SUPPORT
#endif

#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Os/vp_os_malloc.h>

C_RESULT
vp_stages_input_file_stage_open(vp_stages_input_file_config_t *cfg)
{
  int i;
  int res;
  int filesize=0;

  /* Fill known information about the output picture */
  /* Only YUV420P and YUV422 are supported */

  if (strstr(cfg->name,".yuyv")!=NULL)
  { cfg->vp_api_picture.format = PIX_FMT_YUV422;  }
  else if (strstr(cfg->name,".uyvy")!=NULL)
  { cfg->vp_api_picture.format = PIX_FMT_UYVY422; }
  else if (strstr(cfg->name,".vyuy")!=NULL)
  { cfg->vp_api_picture.format = PIX_FMT_VYUY422; }
  else if (strstr(cfg->name,".yuv")!=NULL)
  { cfg->vp_api_picture.format = PIX_FMT_YUV420P; }
  else if (strstr(cfg->name,".y")!=NULL)
  { cfg->vp_api_picture.format = PIX_FMT_GRAY8; }
  else
  { cfg->vp_api_picture.format = PIX_FMT_YUV420P; }

  cfg->vp_api_picture.y_pad = cfg->vp_api_picture.c_pad = cfg->vp_api_picture.framerate = 0;
  cfg->vp_api_picture.blockline = 0;
  cfg->vp_api_picture.complete = 1;

  cfg->vp_api_picture.width = cfg->width;
  cfg->vp_api_picture.height = cfg->height;
  cfg->vp_api_picture.y_line_size = (cfg->vp_api_picture.width) * ( (PIX_FMT_YUV420P == cfg->vp_api_picture.format) ? (1) : (2) );
  cfg->vp_api_picture.cb_line_size = /* ... */
  cfg->vp_api_picture.cr_line_size = (PIX_FMT_YUV420P == cfg->vp_api_picture.format) ? (cfg->vp_api_picture.y_line_size/2) : 0;

 /* Open the file to read */
  cfg->f = fopen(cfg->name, "rb");
  if(cfg->f == NULL)
  {
    PRINT("Missing input file <%s>\n",cfg->name);
    return (VP_FAILURE);
  }

  /* Get the file size */
  fseek(cfg->f, 0L, SEEK_END);
  filesize = ftell(cfg->f);
  fseek(cfg->f, 0L, SEEK_SET);

  cfg->buffer_size = vp_api_picture_get_buffer_size(&(cfg->vp_api_picture));

  cfg->moving_picture = (filesize>cfg->buffer_size)? 1:0 ;

  cfg->nb_buffers=1;
  cfg->buffers=NULL;
  cfg->data = NULL;

  if (cfg->preload!=0)
  {
    /* Preload enough frames to reach the required amount of preloaded data */
    cfg->nb_buffers = (cfg->preload/cfg->buffer_size);
    if (cfg->nb_buffers<1) { cfg->nb_buffers=1; }
  }
  else
  {
    /* Use only one buffer in which frames will be read from the disk at each transform() call */
    cfg->nb_buffers = 1;
  }

  /* Allocate memory for the buffer list */
  if (NULL==cfg->buffers)
  {
    cfg->buffers = vp_os_malloc(sizeof(*cfg->buffers)*cfg->nb_buffers);
    if (cfg->buffers==NULL)
    {
      PRINT("%s:%d - Failed preloading %s : not enough memory for buffer pointers(required %d pointers)\n",
          __FUNCTION__,__LINE__,
          cfg->name,cfg->nb_buffers/(1024*1024));
    }
  }

  /* Allocate memory for the buffers */

  if (NULL==cfg->data)
  {
#ifdef USE_ELINUX
    posix_memalign((void**)&cfg->data,256, cfg->buffer_size*cfg->nb_buffers );
#else
    cfg->data = vp_os_malloc(cfg->buffer_size*cfg->nb_buffers);
#endif
    if (cfg->data==NULL)
    {
      PRINT("Failed preloading %s : not enough memory (required %d MB)\n",
          cfg->name,cfg->buffer_size*cfg->nb_buffers/(1024*1024));
    }

    for (i=0;i<cfg->nb_buffers;i++) { cfg->buffers[i] = (NULL!=cfg->data) ? (cfg->data + i*cfg->buffer_size):NULL; }
  }

  /* Load as much data as possible into memory */

  if (cfg->data && cfg->preload!=0)
  {
    PRINT("Preloading file %s to memory ...\n",cfg->name);
    for (i=0;i<cfg->nb_buffers;i++)
    {
#ifndef USE_ELINUX
    PRINT("."); fflush(stdout);
#endif
      res = fread(cfg->buffers[i], cfg->buffer_size , 1 , cfg->f);
      if (res<1)
      {
        /* Reading a frame failed */
        cfg->nb_buffers=i;  /* Store the number of successfully read frames */
      }
    }
    PRINT("ok.\n");
    fclose(cfg->f);
    cfg->f=NULL;
  }

  cfg->current_buffer=0;

  PRINT("Using %s input file %s\n  resolution %dx%d\n  total size : %d MB\n",
          (PIX_FMT_YUV420P==cfg->vp_api_picture.format)?"420P":"422",
          cfg->name,
          cfg->width,cfg->height,filesize/(1024*1024));

  return (VP_SUCCESS);
}

C_RESULT
vp_stages_input_file_stage_transform(vp_stages_input_file_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

#ifdef VERBOSE
  PRINT("%s:%d Reading a frame from YUV file %s ...\n",__FUNCTION__,__LINE__,cfg->name);
#endif

  /* Init */
  if (out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers =  cfg->nb_buffers;
    out->size = cfg->buffer_size; // cfg->y_size + (cfg-> use_chrominances ? (cfg->subsample422 ? cfg->c_size : 2 * cfg->c_size) : 0);

    out->status = VP_API_STATUS_PROCESSING;
    if (cfg->f) { rewind(cfg->f); } // back to begining of input file


#ifdef VERBOSE
    PRINT("%s:%d Frames stored at address %p.\n",__FUNCTION__,__LINE__,cfg->data);
#endif
  }

  /* Go back at the beginning of the file if we dont want a moving picture */
  if (0==cfg->moving_picture)
  {
    out->status = VP_API_STATUS_PROCESSING;
    if (cfg->f) { rewind(cfg->f); } // back to begining of input file
  }

  if (out->status == VP_API_STATUS_PROCESSING)
  {
    if (cfg->preload>0)
    {
      /* Get a picture from the data preloaded by open()
       * This allows a minimum delay to get a new frame.
       */
      out->buffers = cfg->buffers;
      out->indexBuffer = cfg->current_buffer;
      cfg->current_buffer = ( cfg->current_buffer + 1 ) % ( (cfg->nb_buffers<=8)?cfg->nb_buffers:8) ;
    }
    else
    {
      /* Read a frame from the disk file */

#ifdef VERBOSE
      PRINT("%s:%d  Requesting %d bytes ...\n",__FUNCTION__,__LINE__,cfg->buffer_size);
#endif

      out->buffers=cfg->buffers;
      out->indexBuffer = 0;
      out->size = fread(out->buffers[0], cfg->buffer_size, 1 , cfg->f);

#ifdef VERBOSE
      PRINT("%s:%d  Read %d bytes ...\n",__FUNCTION__,__LINE__,out->size);
#endif

      /* Go back at the beginning of the file if we reached the end of the video */
      if ((out->size < 1 ) || ((cfg->f)&&feof(cfg->f)))
      {
        if (cfg->loop == TRUE)
        {
          out->status = VP_API_STATUS_PROCESSING;
          if (cfg->f) { rewind(cfg->f); } // back to begining of input file
        }
        else
        {
          out->status = VP_API_STATUS_ENDED;
        }
      }

      /* Check for error */
      if ((cfg->f)&&ferror(cfg->f))
      {
        PRINT("ferror\n");
        out->status = VP_API_STATUS_ERROR;
      }
    }

    if (out->status == VP_API_STATUS_PROCESSING)
    {
      out->size = cfg->buffer_size;

      /* Copy the output buffer information in the vp_api_picture structure */
      cfg->vp_api_picture.y_pad = cfg->vp_api_picture.c_pad = cfg->vp_api_picture.framerate = 0;
      cfg->vp_api_picture.blockline = 0;
      cfg->vp_api_picture.complete = 1;

      /* The buffer address may change at every transform() call */
      switch (cfg->vp_api_picture.format)
      {
        case PIX_FMT_YUV420P:
          cfg->vp_api_picture.cb_buf = (uint8_t *) out->buffers[out->indexBuffer] + (cfg->width*cfg->height);
          cfg->vp_api_picture.cr_buf = (uint8_t *) out->buffers[out->indexBuffer] + (cfg->width*cfg->height) + (cfg->width*cfg->height)/4;
          break;
        default :
          /* Leave Cr and Cb to NULL for interleaved formats */
          cfg->vp_api_picture.cb_buf = NULL;
          cfg->vp_api_picture.cr_buf = NULL;
      }
      cfg->vp_api_picture.y_buf = (uint8_t *) out->buffers[out->indexBuffer];
      cfg->vp_api_picture.raw = (uint8_t *) out->buffers[out->indexBuffer];
    }
    else
    {
      out->size = 0;
    }
  }

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_input_file_stage_close(vp_stages_input_file_config_t *cfg)
{
  if (cfg->f) { fclose(cfg->f); }
  cfg->f=NULL;
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_file_stage_open(vp_stages_output_file_config_t *cfg)
{
  VP_OS_ASSERT(cfg->flush_every_nb >= 0);
  cfg->f = fopen(cfg->name, "wb");
  if(NULL == cfg->f)
  {
    PRINT("%s:%d - Error opening file %s\n",__FUNCTION__,__LINE__,cfg->name);perror("");
    return VP_FAILURE;
  }
  return (VP_SUCCESS);
}


#define RATIO 1

C_RESULT
vp_stages_output_file_stage_transform(vp_stages_output_file_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  static int total_size = 0;

  vp_os_mutex_lock(&out->lock);

  if ((cfg->f) && in->status == VP_API_STATUS_PROCESSING && in->size > 0)
  {
    fwrite(in->buffers[in->indexBuffer], sizeof(int8_t), in->size*sizeof(int8_t), cfg->f);
    total_size += in->size;
  }

  //fflush(cfg->f);
  out->status = in->status;

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_file_stage_close(vp_stages_output_file_config_t *cfg)
{
  if (cfg->f) { fclose(cfg->f); cfg->f=NULL; }
  return (VP_SUCCESS);
}

