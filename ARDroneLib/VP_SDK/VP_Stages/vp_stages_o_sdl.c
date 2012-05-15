/**
 *  \brief    VP Stages. Output SDL stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

#if !defined(__NDS__)

///////////////////////////////////////////////
// INCLUDES

#include <VP_Stages/vp_stages_o_sdl.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Api/vp_api_thread_helper.h>


static int pipeline_opened = 0;
static vp_os_mutex_t xlib_mutex;

#if defined(_CK4215_) && defined(WIN32)
static void * main_windows;
static void * child_windows;
static BOOL exit_pipeline;

#ifdef _TEST_CK4215_
static BOOL show_hide;
#endif

void
vp_stages_init_display(void * handle)
{
	main_windows = handle;
}

void *
vp_stages_get_child_window( void )
{
	return child_windows;
}

void manage_events(void)
{
  SDL_Event event;
  static uint16_t x =0, y = 0;
  RECT rect;

      if(SDL_PollEvent(&event))
	{
	  switch(event.type)
	    {
#if defined(_CK4215_) && defined(WIN32)

#ifdef _TEST_CK4215_
	  case SDL_MOUSEMOTION:
		  if(show_hide)
			  ShowWindow((HWND)child_windows,SW_HIDE);
		  else
			  ShowWindow((HWND)child_windows,SW_SHOW);
			break;

#endif

	  case SDL_QUIT:
			//exit(1);
		  //ExitThread(0);
		  if(!exit_pipeline)
		  {
			exit_pipeline = TRUE;
		  }
		break;
#endif
	    case SDL_KEYDOWN:
	    case SDL_KEYUP:
	      {
		SDL_KeyboardEvent *kb_event = (SDL_KeyboardEvent *)&event;
		switch(kb_event->keysym.sym)
		  {
		  case SDLK_ESCAPE:
		    exit(1);
		    break;
		  default:
		    break;
		  }
	      }
	    default:
	      break;
	    }
	}

}

#endif

PROTO_THREAD_ROUTINE(escaper,nomParams)
{
  SDL_Event event;

  while(!pipeline_opened)
    {
      vp_os_delay(100);
    }

  while(pipeline_opened)
    {
      vp_os_mutex_lock(&xlib_mutex);
      if(SDL_PollEvent(&event))
	{
	  vp_os_mutex_unlock(&xlib_mutex);
	  switch(event.type)
	    {
	    case SDL_KEYDOWN:
	    case SDL_KEYUP:
	      {
		SDL_KeyboardEvent *kb_event = (SDL_KeyboardEvent *)&event;
		switch(kb_event->keysym.sym)
		  {
		  case SDLK_ESCAPE:
		    exit(1);
		    break;
		  default:
		    break;
		  }
	      }
	    default:
	      break;
	    }
	}
      else
	{
	  vp_os_mutex_unlock(&xlib_mutex);
	}
    }
  return (THREAD_RET)0;
}


static void
vp_stages_buffer_to_overlay(SDL_Overlay *overlay, vp_stages_output_sdl_config_t *cfg, vp_api_picture_t *picture)
{
  uint8_t *dst0, *dst1, *dst2, *dst3;
  uint8_t *src0, *src1, *src2, *src3;
  int s0, s1, s2;
  int d0, d1, d2;
  int i;

  SDL_LockYUVOverlay(overlay);

  dst0 = overlay->pixels[0];
  src0 = picture->y_buf;
  dst1 = overlay->pixels[1];
  src1 = picture->cr_buf;
  dst2 = overlay->pixels[2];
  src2 = picture->cb_buf;

  d0 = overlay->pitches[0];
  d1 = overlay->pitches[1];
  d2 = overlay->pitches[2];

  s0 = picture->y_line_size;
  s1 = picture->cb_line_size;
  s2 = picture->cr_line_size;

  dst3 = dst0 + d0 * cfg->pic_height/2;
  src3 = src0 + s0 * picture->height/2;

  if(!cfg->c_size)
  {
    vp_os_memset(dst1, 0x80, ((cfg->pic_width/2+d1)*cfg->pic_height/2)/2);
    vp_os_memset(dst2, 0x80, ((cfg->pic_width/2+d2)*cfg->pic_height/2)/2);
  }

  for(i = 0 ; i < (int32_t)cfg->pic_height/2 ; i++)
  {
    memcpy(dst0, src0, cfg->pic_width*sizeof(char));
    dst0 += d0;
    src0 += s0;

    if(cfg->c_size)
    {
      memcpy(dst1, src1, cfg->pic_width/2*sizeof(char));
      dst1 += d1;
      src1 += s1;

      memcpy(dst2, src2, cfg->pic_width/2*sizeof(char));
      dst2 += d2;
      src2 += s2;
    }

    memcpy(dst3, src3, cfg->pic_width*sizeof(char));
    dst3 += d0;
    src3 += s0;
  }

  SDL_UnlockYUVOverlay(overlay);
}


static int
vp_stages_display_frame(vp_stages_output_sdl_config_t *cfg, vp_api_picture_t *picture)
{
  SDL_Rect dstrect;

  if( picture )
  {
    dstrect.x = (cfg->window_width-cfg->width)/2;
    dstrect.y = (cfg->window_height-cfg->height)/2;
    dstrect.w = cfg->width;
    dstrect.h = cfg->height;

    vp_os_mutex_lock(&xlib_mutex);

    vp_stages_buffer_to_overlay(cfg->overlay, cfg, picture);
    SDL_DisplayYUVOverlay(cfg->overlay, &dstrect);

    vp_os_mutex_unlock(&xlib_mutex);
  }

  return 0;
}


C_RESULT
vp_stages_output_sdl_stage_open(vp_stages_output_sdl_config_t *cfg)
{
  vp_os_mutex_init(&xlib_mutex);

  if(SDL_Init(SDL_INIT_TIMER|SDL_INIT_VIDEO))
    {
      PRINT("Error initializing SDL\n");
      return (VP_FAILURE);
    }
#if defined(_CK4215_) && defined(WIN32)
	child_windows = NULL;
	exit_pipeline = FALSE;

#ifdef _TEST_CK4215_
	show_hide = TRUE;//show
#endif

#endif
  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_sdl_stage_transform(vp_stages_output_sdl_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
#if defined(_CK4215_) && defined(WIN32)
	struct SDL_SysWMinfo wmInfo;
#endif

  vp_os_mutex_lock(&out->lock);

#if defined(_CK4215_) && defined(WIN32)
  if( exit_pipeline == TRUE )
  {
	  /* It let the VPSDK handles the closing of the SDL */
	out->status = VP_API_STATUS_ENDED;
	return (VP_FAILURE);
  }
#endif

  if(out->status == VP_API_STATUS_INIT)
  {
      out->numBuffers = 1;
      out->size = cfg->y_size + 2*cfg->c_size;
      out->buffers = (int8_t**)vp_os_malloc(sizeof(uint8_t*)+out->size*sizeof(uint8_t));
      out->buffers[0] = (int8_t*)(out->buffers+1);
      out->indexBuffer = 0;

#if defined(_CK4215_) && defined(WIN32)

	if( main_windows )
	  cfg->surface = SDL_SetVideoMode(cfg->width, cfg->height, cfg->bpp, SDL_NOFRAME);
	else
	  cfg->surface = SDL_SetVideoMode(cfg->width, cfg->height, cfg->bpp, SDL_HWSURFACE);

#else
	  cfg->surface = SDL_SetVideoMode(cfg->width, cfg->height, cfg->bpp, SDL_HWSURFACE);
#endif
    SDL_ShowCursor(SDL_DISABLE);
    cfg->overlay = SDL_CreateYUVOverlay(cfg->pic_width, cfg->pic_height, SDL_YV12_OVERLAY, cfg->surface);

#if defined(_CK4215_) && defined(WIN32)
	SDL_VERSION(&wmInfo.version);

	if(-1 == SDL_GetWMInfo(&wmInfo))
	{
		OutputDebugString(SDL_GetError());
		return -1;
	}

	child_windows = (void *) wmInfo.window;

	//Attach to parent windows
	if( main_windows )
		SetParent((HWND)child_windows,(HWND) main_windows);

	if( main_windows )
	{
		//Put the child windows at the good position
		//MoveWindow((HWND)child_windows, 0,0,/*g_x, g_y,*/ cfg->width, cfg->height, FALSE);
		MoveWindow((HWND)child_windows, cfg->window_pos_x, cfg->window_pos_y, cfg->width, cfg->height, FALSE);
	}
	else
	{
		SetWindowPos(
				(HWND)child_windows,
				HWND_TOPMOST,
				cfg->window_pos_x, //0,	// X
				cfg->window_pos_y,//0,	// Y
				cfg->width,		// cx
				cfg->height,	// cy
				SWP_SHOWWINDOW//SWP_NOREPOSITION//SWP_NOREDRAW	//uFlags
			);
	}

#endif
  }

  out->status = (in->status == VP_API_STATUS_STILL_RUNNING ? VP_API_STATUS_PROCESSING : in->status);

  pipeline_opened = 1;

  if((out->status == VP_API_STATUS_PROCESSING || out->status == VP_API_STATUS_STILL_RUNNING) && in->size > 0)
  {
    vp_stages_display_frame(cfg, (vp_api_picture_t*)in->buffers);
#if defined(_CK4215_) && defined(WIN32)
    manage_events();
#endif
    vp_os_memcpy(out->buffers[0], ((vp_api_picture_t*)in->buffers)->y_buf, cfg->y_size);
    if(cfg->c_size)
      {
	vp_os_memcpy(out->buffers[0]+cfg->y_size, ((vp_api_picture_t*)in->buffers)->cb_buf, cfg->c_size);
	vp_os_memcpy(out->buffers[0]+cfg->y_size+cfg->c_size, ((vp_api_picture_t*)in->buffers)->cr_buf, cfg->c_size);
      }
  }

  // not managed
  if(in->status == VP_API_STATUS_ENDED)
    {
      pipeline_opened = 0;
      vp_os_free(out->buffers);
    }

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_sdl_stage_close(vp_stages_output_sdl_config_t *cfg)
{
  vp_os_mutex_lock(&xlib_mutex);

  SDL_ShowCursor(SDL_ENABLE);
  SDL_FreeYUVOverlay(cfg->overlay);
  SDL_FreeSurface(cfg->surface);

  SDL_Quit();

  vp_os_mutex_unlock(&xlib_mutex);

  return (VP_SUCCESS);
}


#endif // ! __NDS__
