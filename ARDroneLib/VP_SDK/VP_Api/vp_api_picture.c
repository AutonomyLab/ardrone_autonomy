/*
 * vp_api_picture.c
 *
 *  Created on: Apr 5, 2011
 *      Author: s_piskorsko
 */
#ifdef FFMPEG_SUPPORT
#undef FFMPEG_SUPPORT
#endif

#ifdef USE_ELINUX
	/* Allow the MP4 Visual and H264 encoders to read a bit more data than actually stored */
	#define ROUNDUP 16
#else
	#define ROUNDUP 1
#endif

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Os/vp_os_malloc.h>
#include <stdio.h>

static inline int roundUp(int x,int m)
{
    int r = x%m;
    int q = x/m;
    return ( (r==0) ? x : (q+1)*m );
}


typedef struct{
	int width,height;
	char requester[128];
	void * address;
	int size;
}vp_api_picture_allocation_log_t;

static vp_api_picture_allocation_log_t picturelog[128];
static int log_size = 0;

C_RESULT vp_api_picture_print_allocator_log()
{
	int i;
	for (i=0;i<log_size;i++){
		printf("Allocated vp_api_picture #%d : %dx%d address %p size %d \n",
				i,
				picturelog[i].width,
				picturelog[i].height,
				picturelog[i].address,
				picturelog[i].size);
	}
	
	return C_OK;
}

C_RESULT vp_api_picture_alloc(vp_api_picture_t * pic,const int width,const int height,enum PixelFormat format)
{
	int picture_size;

	/* Params check */
		if (NULL==pic) { return C_OK; }

		vp_os_memset(pic,0,sizeof(*pic));

	/* Fill deprecated useless stuff */
		pic->blockline = 0;
		pic->complete = 1;
		pic->y_pad = 0;
		pic->c_pad = 0;

	/* Basic information */
		pic->format = format;
		pic->width = width;
		pic->height = height;

	/* Buffers informations and allocation */
		switch(format)
		{
		case PIX_FMT_YUV422:
		case PIX_FMT_UYVY422:
		case PIX_FMT_VYUY422:
			pic->y_line_size = 2*width;
			pic->cb_line_size = 0;
			pic->cr_line_size = 0;
			picture_size = pic->y_line_size*(roundUp(pic->height,ROUNDUP));

			#ifdef USE_ELINUX
				pic->raw = vp_os_aligned_malloc(picture_size,256);
			#else
				pic->raw  = vp_os_malloc(picture_size);
			#endif
		break;

		case PIX_FMT_YUV420P:
			pic->y_line_size = width;
			pic->cb_line_size = width/2;
			pic->cr_line_size = width/2;
			picture_size = (pic->y_line_size  * (roundUp(pic->height,ROUNDUP))) +
						   (pic->cb_line_size * (roundUp(pic->height,ROUNDUP))/2) +
						   (pic->cr_line_size * (roundUp(pic->height,ROUNDUP))/2);

			#ifdef USE_ELINUX
				pic->raw = vp_os_aligned_malloc(picture_size,256);
			#else
				pic->raw = vp_os_malloc(picture_size);
			#endif
		break;

		default:
			printf("%s:%d - Unsupported picture format.\n",__FUNCTION__,__LINE__);
			return C_FAIL;
		}

		vp_api_picture_format_to_buf_address(pic);

		/* Keep track of allocated pictures for debugging */
		picturelog[log_size].width  = width;
		picturelog[log_size].height = height;
		picturelog[log_size].address = pic->raw;
		picturelog[log_size].size = picture_size;
		log_size++;

#if 0
		vp_api_picture_print_allocator_log();
#endif

	return C_OK;
}

int vp_api_picture_get_buffer_size(const vp_api_picture_t * pic)
{
	if(NULL==pic) { return 0; }
	switch(pic->format)
	{
		case PIX_FMT_YUV422 :
		case PIX_FMT_UYVY422:
		case PIX_FMT_VYUY422:
		    return (pic->width * pic->height * 2);
		case PIX_FMT_YUV420P: 
			return (pic->width * pic->height * 3)/2;
		case PIX_FMT_GRAY8:
            return (pic->width * pic->height);
		default:
			return 0;
	}
}

const char * vp_api_picture_get_filename_extension(const vp_api_picture_t * pic)
{
	const char * ext;
	if (NULL==pic){
		return NULL;
	}
	switch(pic->format)
	{
		case PIX_FMT_YUV420P: ext="yuv"; break;
		case PIX_FMT_YUV422:  ext="yuyv"; break;
		case PIX_FMT_UYVY422: ext="yuyv"; break;
		case PIX_FMT_VYUY422: ext="vyuy"; break;
		default:
		ext="raw";
	}
	return ext;
}

const char * vp_api_picture_get_pixelformat_name(enum PixelFormat fmt)
{
    switch(fmt)
    {
        case PIX_FMT_NONE   : return "NONE";
        case PIX_FMT_YUV420P: return "YUV420p";
        case PIX_FMT_YUV422 : return "YUYV";
        case PIX_FMT_UYVY422: return "UYVY";
        case PIX_FMT_UYVY411: return "VYUY11";
        case PIX_FMT_VYUY422: return "VYUV";

        default : return "unsupported";
    }
}

int vp_api_picture_format_to_buf_address(vp_api_picture_t * pic)
{
    int ret = 1;
    switch(pic->format)
    {
        case PIX_FMT_YUV422:
          pic->y_buf  = (uint8_t *) pic->raw;
          pic->cb_buf = (uint8_t *) pic->raw + 1;
          pic->cr_buf = (uint8_t *) pic->raw + 2;
          break;
        case PIX_FMT_UYVY422:
          pic->y_buf  = (uint8_t *) pic->raw + 1;
          pic->cb_buf = (uint8_t *) pic->raw;
          pic->cr_buf = (uint8_t *) pic->raw + 2;
          break;
        case PIX_FMT_VYUY422:
          pic->y_buf  = (uint8_t *) pic->raw + 2;
          pic->cb_buf = (uint8_t *) pic->raw;
          pic->cr_buf = (uint8_t *) pic->raw + 1;
          break;
        case PIX_FMT_YUV420P:
          pic->y_buf  = (uint8_t *) pic->raw;
          pic->cb_buf = (uint8_t *) pic->raw + (pic->y_line_size * pic->height);
          pic->cr_buf = (uint8_t *) pic->raw + (pic->y_line_size * pic->height) + (pic->cr_line_size * pic->height/2);
          break;

        default:
          ret = 0;
          break;
    }
    return ret;
 }

int vp_api_picture_point_to_buf_address(vp_api_picture_t * pic,void*addr)
{
	if (pic){
		pic->raw = addr;
		vp_api_picture_format_to_buf_address(pic);
	}
	return 0;
}
