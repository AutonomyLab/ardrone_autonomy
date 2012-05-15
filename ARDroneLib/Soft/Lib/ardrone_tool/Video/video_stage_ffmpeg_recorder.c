/* CODE ADAPATED FOR ARDRONE NAVIGATION FROM :
 *
 * Libavformat API example: Output a media file in any supported
 * libavformat format. The default codecs are used.
 *
 * Copyright (c) 2003 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <generated_custom.h>

/*#ifndef USE_FFMPEG_RECORDER
#define USE_FFMPEG_RECORDER
#endif*/

#ifdef USE_FFMPEG_RECORDER

/* From FFMPEG example */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavfilter/avfilter.h>
#include <libavutil/avutil.h>
#include <libavcodec/avcodec.h>
#include <libavcodec/opt.h>


#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



/* Should be in avutil.h but Ubuntu 10.04 package is too old
 * http://cekirdek.pardus.org.tr/~ismail/ffmpeg-docs/libavutil_2avutil_8h-source.html
 * */
#if LIBAVUTIL_VERSION_MAJOR < 50
#define AV_PKT_FLAG_KEY   0x0001
enum AVMediaType {
     AVMEDIA_TYPE_UNKNOWN = -1,
     AVMEDIA_TYPE_VIDEO,
     AVMEDIA_TYPE_AUDIO,
     AVMEDIA_TYPE_DATA,
     AVMEDIA_TYPE_SUBTITLE,
     AVMEDIA_TYPE_ATTACHMENT,
     AVMEDIA_TYPE_NB
 };
#endif


#include <time.h>
#ifndef _WIN32
	#include <sys/time.h>
#else
  #include <sys/timeb.h>
 #include <Winsock2.h>  // for timeval structure

 int gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#endif

#include <VP_Os/vp_os_malloc.h>
#include <VP_Api/vp_api_picture.h>

#include <config.h>
#include <ardrone_tool/Video/video_stage_ffmpeg_recorder.h>

/*#ifdef USE_VIDEO_YUV
#define VIDEO_FILE_EXTENSION "yuv"
#else
#define VIDEO_FILE_EXTENSION "y"
#endif
#define  VIDEO_TIMESTAMPS_FILE_EXTENSION "time.txt"

#ifndef VIDEO_FILE_DEFAULT_PATH
#ifdef USE_ELINUX
#define VIDEO_FILE_DEFAULT_PATH "/data/video"
#else
#define VIDEO_FILE_DEFAULT_PATH "."
#endif
#endif
*/
#define VIDEO_FILE_DEFAULT_PATH "."

 /*- LibAVFormat variables */
	const char *filename;
	AVOutputFormat *fmt;
	AVFormatContext *oc;
	AVStream *video_st;
	double video_pts;
	int i;

#define STREAM_FRAME_RATE (60)
#define STREAM_BIT_RATE_KBITS 1600
#define STREAM_PIX_FMT PIX_FMT_YUV420P /* default pix_fmt */

static int sws_flags = SWS_BICUBIC;

AVFrame *picture_to_encode=NULL, *tmp_picture=NULL;
uint8_t *video_outbuf=NULL;
int frame_count=0, video_outbuf_size=0;


const vp_api_stage_funcs_t video_ffmpeg_recorder_funcs = {
  (vp_api_stage_handle_msg_t) video_stage_ffmpeg_recorder_handle,
  (vp_api_stage_open_t) video_stage_ffmpeg_recorder_open,
  (vp_api_stage_transform_t) video_stage_ffmpeg_recorder_transform,
  (vp_api_stage_close_t) video_stage_ffmpeg_recorder_close
};

char video_filename_ffmpeg[VIDEO_FILENAME_LENGTH];

struct
{
	int width,height;
	char* buffer;
	unsigned long long int timestamp_us;
	int frame_number;
}previous_frame;


/******************************************************************************************************************************************/

void create_video_file(const char*filename,int width,int height)
{
/* auto detect the output format from the name. default is
       mpeg. */
    //fmt = av_guess_format(NULL, filename, NULL);

#if (LIBAVFORMAT_VERSION_INT>=AV_VERSION_INT(52,81,0))
	#define libavformat_guess_format av_guess_format
#else
	#define libavformat_guess_format guess_format
#endif

	fmt = libavformat_guess_format(NULL, filename, NULL);

	if (!fmt) {
        printf("Could not deduce output format from file extension: using MPEG.\n");
        //fmt = av_guess_format("mpeg", NULL, NULL);
        fmt = libavformat_guess_format("mpeg", NULL, NULL);
    }
    if (!fmt) {
        fprintf(stderr, "Could not find suitable output format\n");
        exit(1);
    }

    /* allocate the output media context */
    oc = avformat_alloc_context();
    if (!oc) {
        fprintf(stderr, "Memory error\n");
        exit(1);
    }
    oc->oformat = fmt;
    snprintf(oc->filename, sizeof(oc->filename), "%s", filename);

    /* add the audio and video streams using the default format codecs
       and initialize the codecs */
    video_st = NULL;

    if (fmt->video_codec != CODEC_ID_NONE) {
        video_st = add_video_stream(oc, fmt->video_codec,width,height);
    }

    /* set the output parameters (must be done even if no
       parameters). */
    if (av_set_parameters(oc, NULL) < 0) {
        fprintf(stderr, "Invalid output format parameters\n");
        exit(1);
    }

    dump_format(oc, 0, filename, 1);

    /* now that all the parameters are set, we can open the audio and
       video codecs and allocate the necessary encode buffers */
    if (video_st)
        open_video(oc, video_st);

    /* open the output file, if needed */
    if (!(fmt->flags & AVFMT_NOFILE)) {
        if (url_fopen(&oc->pb, filename, URL_WRONLY) < 0) {
            fprintf(stderr, "Could not open '%s'\n", filename);
            exit(1);
        }
    }

    /* write the stream header, if any */
    av_write_header(oc);
 }


void close_video_file()
{
/* write the trailer, if any.  the trailer must be written
     * before you close the CodecContexts open when you wrote the
     * header; otherwise write_trailer may try to use memory that
     * was freed on av_codec_close() */
    av_write_trailer(oc);

    /* close each codec */
    if (video_st)
        close_video(oc, video_st);

    /* free the streams */
    for(i = 0; i < oc->nb_streams; i++) {
        av_freep(&oc->streams[i]->codec);
        av_freep(&oc->streams[i]);
    }

    if (!(fmt->flags & AVFMT_NOFILE)) {
        /* close the output file */
        url_fclose(oc->pb);
    }

    /* free the stream */
    av_free(oc);
}




/**************************************************************/
/* video output */



/* add a video output stream */
 AVStream *add_video_stream(AVFormatContext *oc, enum CodecID codec_id, int width, int height)
{
    AVCodecContext *c;
    AVStream *st;

    st = av_new_stream(oc, 0);
    if (!st) {
        fprintf(stderr, "Could not alloc stream\n");
        exit(1);
    }

    c = st->codec;
    c->codec_id = codec_id;
    c->codec_type = AVMEDIA_TYPE_VIDEO;

    /* put sample parameters */
    c->bit_rate = (STREAM_BIT_RATE_KBITS)*1000;
    /* resolution must be a multiple of two */
    c->width = width;
    c->height = height;
    /* time base: this is the fundamental unit of time (in seconds) in terms
       of which frame timestamps are represented. for fixed-fps content,
       timebase should be 1/framerate and timestamp increments should be
       identically 1. */
    c->time_base.den = STREAM_FRAME_RATE;
    c->time_base.num = 1;
    c->gop_size = 12; /* emit one intra frame every twelve frames at most */
    c->pix_fmt = STREAM_PIX_FMT;
    if (c->codec_id == CODEC_ID_MPEG2VIDEO) {
        /* just for testing, we also add B frames */
        c->max_b_frames = 2;
    }
    if (c->codec_id == CODEC_ID_MPEG1VIDEO){
        /* Needed to avoid using macroblocks in which some coeffs overflow.
           This does not happen with normal video, it just happens here as
           the motion of the chroma plane does not match the luma plane. */
        c->mb_decision=2;
    }
    // some formats want stream headers to be separate
    if(oc->oformat->flags & AVFMT_GLOBALHEADER)
        c->flags |= CODEC_FLAG_GLOBAL_HEADER;

    return st;
}

 AVFrame *alloc_picture(enum PixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    uint8_t *picture_buf;
    int size;

    picture = avcodec_alloc_frame();
    if (!picture)
        return NULL;
    size = avpicture_get_size(pix_fmt, width, height);
    picture_buf = av_malloc(size);
    if (!picture_buf) {
        av_free(picture);
        return NULL;
    }
    avpicture_fill((AVPicture *)picture, picture_buf,
                   pix_fmt, width, height);
    return picture;
}

 void open_video(AVFormatContext *oc, AVStream *st)
{
    AVCodec *codec;
    AVCodecContext *c;

    c = st->codec;

    /* find the video encoder */
    codec = avcodec_find_encoder(c->codec_id);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    /* open the codec */
    if (avcodec_open(c, codec) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }

    video_outbuf = NULL;
    if (!(oc->oformat->flags & AVFMT_RAWPICTURE)) {
        /* allocate output buffer */
        /* XXX: API change will be done */
        /* buffers passed into lav* can be allocated any way you prefer,
           as long as they're aligned enough for the architecture, and
           they're freed appropriately (such as using av_free for buffers
           allocated with av_malloc) */
        video_outbuf_size = 200000;
        video_outbuf = av_malloc(video_outbuf_size);
    }

    /* allocate the encoded raw picture */
    picture_to_encode =  avcodec_alloc_frame();
    //alloc_picture(c->pix_fmt, c->width, c->height);
    /*if (!picture) {
        fprintf(stderr, "Could not allocate picture\n");
        exit(1);
    }*/

    /* if the output format is not YUV420P, then a temporary YUV420P
       picture is needed too. It is then converted to the required
       output format */
    tmp_picture = NULL;
    if (c->pix_fmt != PIX_FMT_YUV420P) {
        tmp_picture = alloc_picture(PIX_FMT_YUV420P, c->width, c->height);
        if (!tmp_picture) {
            fprintf(stderr, "Could not allocate temporary picture\n");
            exit(1);
        }
    }
}


 void write_video_frame(AVFormatContext *oc, AVStream *st)
{
    int out_size, ret;
    AVCodecContext *c;
    static struct SwsContext *img_convert_ctx;

    //printf("Here0 \n");

    c = st->codec;

        if (c->pix_fmt != PIX_FMT_YUV420P) {
            /* as we only generate a YUV420P picture, we must convert it
               to the codec pixel format if needed */
            if (img_convert_ctx == NULL) {

		#if (LIBSWSCALE_VERSION_INT<AV_VERSION_INT(0,12,0))
            	img_convert_ctx = sws_getContext(c->width, c->height,
                                                 PIX_FMT_YUV420P,
                                                 c->width, c->height,
                                                 c->pix_fmt,
                                                 sws_flags, NULL, NULL, NULL);
		#else
            	img_convert_ctx = sws_alloc_context();

                if (img_convert_ctx == NULL) {
                    fprintf(stderr, "Cannot initialize the conversion context\n");
                    exit(1);
                }

                /* see http://permalink.gmane.org/gmane.comp.video.ffmpeg.devel/118362 */
                /* see http://ffmpeg-users.933282.n4.nabble.com/Documentation-for-sws-init-context-td2956723.html */

                av_set_int(img_convert_ctx, "srcw", c->width);
                av_set_int(img_convert_ctx, "srch", c->height);

                av_set_int(img_convert_ctx, "dstw", c->width);
                av_set_int(img_convert_ctx, "dsth", c->height);

                av_set_int(img_convert_ctx, "src_format", PIX_FMT_YUV420P);
                av_set_int(img_convert_ctx, "dst_format", c->pix_fmt);

                av_set_int(img_convert_ctx, "param0", 0);
                av_set_int(img_convert_ctx, "param1", 0);

                av_set_int(img_convert_ctx, "flags", sws_flags);

                sws_init_context(img_convert_ctx,NULL,NULL);
		#endif

            }
            sws_scale(img_convert_ctx, (const uint8_t* const *)tmp_picture->data,
            		  tmp_picture->linesize,
                      0, c->height, picture_to_encode->data, picture_to_encode->linesize);
        } else {

        }


    if (oc->oformat->flags & AVFMT_RAWPICTURE) {
        /* raw video case. The API will change slightly in the near
           futur for that */
        AVPacket pkt;
        av_init_packet(&pkt);

        pkt.flags |= AV_PKT_FLAG_KEY;
        pkt.stream_index= st->index;
        pkt.data= (uint8_t *)picture_to_encode;
        pkt.size= sizeof(AVPicture);

        ret = av_interleaved_write_frame(oc, &pkt);
    } else {
        /* encode the image */
    	//printf("Here1 \n");
        out_size = avcodec_encode_video(c, video_outbuf, video_outbuf_size, picture_to_encode);
        /* if zero size, it means the image was buffered */
        if (out_size > 0) {
            AVPacket pkt;
            av_init_packet(&pkt);

            if (c->coded_frame->pts != AV_NOPTS_VALUE)
                pkt.pts= av_rescale_q(c->coded_frame->pts, c->time_base, st->time_base);
            if(c->coded_frame->key_frame)
                pkt.flags |= AV_PKT_FLAG_KEY;
            pkt.stream_index= st->index;
            pkt.data= video_outbuf;
            pkt.size= out_size;

            /* write the compressed frame in the media file */
            ret = av_interleaved_write_frame(oc, &pkt);
        } else {
            ret = 0;
        }
    }
    if (ret != 0) {
        fprintf(stderr, "Error while writing video frame\n");
        exit(1);
    }
    frame_count++;
}

 void close_video(AVFormatContext *oc, AVStream *st)
{
    avcodec_close(st->codec);
    //av_free(picture->data[0]);
    av_free(picture_to_encode);
    picture_to_encode = NULL;
    if (tmp_picture) {
        av_free(tmp_picture->data[0]);
        av_free(tmp_picture);
        tmp_picture=NULL;
    }
    av_free(video_outbuf);
}


/******************************************************************************************************************************************/
C_RESULT
video_stage_ffmpeg_recorder_handle (video_stage_ffmpeg_recorder_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param)
{
	//return (VP_SUCCESS);

	printf("FFMPEG recorder message handler.\n");
	switch (msg_id)
	{
		case PIPELINE_MSG_START:
			{

				if(cfg->startRec==VIDEO_RECORD_STOP)
					cfg->startRec=VIDEO_RECORD_HOLD;
				else
					cfg->startRec=VIDEO_RECORD_STOP;
			}
			break;
		default:
			break;
	}
	return (VP_SUCCESS);
}


/******************************************************************************************************************************************/
C_RESULT video_stage_ffmpeg_recorder_open(video_stage_ffmpeg_recorder_config_t *cfg)
{
	//return C_OK;

	previous_frame.width = previous_frame.height = 0;
	previous_frame.buffer = NULL;
	previous_frame.timestamp_us = 0;
	previous_frame.frame_number =0;

	cfg->startRec=VIDEO_RECORD_STOP;

	/* initialize libavcodec, and register all codecs and formats */
	    av_register_all();

  return C_OK;
}


/******************************************************************************************************************************************/
C_RESULT video_stage_ffmpeg_recorder_transform(video_stage_ffmpeg_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	 time_t temptime;
	 struct timeval tv;
	 struct tm *atm;
	 long long int current_timestamp_us;
	 static long long int first_frame_timestamp_us=0;
	 static int frame_counter=0;
	 int i;
	 int frame_size;
	 static int flag_video_file_open=0;

	 vp_os_mutex_lock( &out->lock );
	 vp_api_picture_t* picture = (vp_api_picture_t *) in->buffers;

	gettimeofday(&tv,NULL);

	 temptime = (time_t)tv.tv_sec;
	 atm = localtime(&temptime);  //atm = localtime(&tv.tv_sec);

	 current_timestamp_us = tv.tv_sec *1000000 + tv.tv_usec;


  if( out->status == VP_API_STATUS_INIT )
  {
    out->numBuffers   = 1;
    out->indexBuffer  = 0;
    out->lineSize     = NULL;
    //out->buffers      = (int8_t **) vp_os_malloc( sizeof(int8_t *) );
  }

  out->size     = in->size;
  out->status   = in->status;
  out->buffers  = in->buffers;

  if( in->status == VP_API_STATUS_ENDED ) {
    out->status = in->status;
  }
  else if(in->status == VP_API_STATUS_STILL_RUNNING) {
    out->status = VP_API_STATUS_PROCESSING;
  }
  else {
    out->status = in->status;
  }



	if(cfg->startRec==VIDEO_RECORD_HOLD)
	{
		/* Create a new video file */

		sprintf(video_filename_ffmpeg, "%s/video_%04d%02d%02d_%02d%02d%02d_w%i_h%i.mp4",
				VIDEO_FILE_DEFAULT_PATH,
				atm->tm_year+1900, atm->tm_mon+1, atm->tm_mday,
				atm->tm_hour, atm->tm_min, atm->tm_sec,
				picture->width,
				picture->height);

		create_video_file(video_filename_ffmpeg, picture->width,picture->height);
		flag_video_file_open=1;

		cfg->startRec=VIDEO_RECORD_START;

		first_frame_timestamp_us = current_timestamp_us;
		frame_counter=1;
	}

  if( out->size > 0 && out->status == VP_API_STATUS_PROCESSING && cfg->startRec==VIDEO_RECORD_START)
  {
	  frame_size = ( previous_frame.width * previous_frame.height )*3/2;

	  /* Send the previous frame to FFMPEG */
	  if (previous_frame.buffer!=NULL)
		{
		  /* Compute the number of frames to store to achieve 60 FPS
		   * This should be computed using the timestamp of the first frame
		   * to avoid error accumulation.
		   */
			int current_frame_number = (current_timestamp_us - first_frame_timestamp_us) / 16666;
			int nb_frames_to_write = current_frame_number - previous_frame.frame_number;

			if (picture_to_encode!=NULL){
				picture_to_encode->data[0] = picture_to_encode->base[0] = picture->y_buf;
				picture_to_encode->data[1] = picture_to_encode->base[1] = picture->cb_buf;
				picture_to_encode->data[2] = picture_to_encode->base[2] = picture->cr_buf;

				picture_to_encode->linesize[0] = picture->width;
				picture_to_encode->linesize[1] = picture->width/2;
				picture_to_encode->linesize[2] = picture->width/2;
			}

			for (i=0;i<nb_frames_to_write;i++)
			{
				//printf("Storing %i frames\n",nb_frames_to_write);
				write_video_frame(oc, video_st);
			}

			/* Pass infos to next iteration */
			previous_frame.frame_number = current_frame_number;
		}

	  /* Create a buffer to hold the current frame */
		//if (0)
		{
	  if (previous_frame.buffer!=NULL && (previous_frame.width!=picture->width || previous_frame.height!=picture->height))
		{
			vp_os_free(previous_frame.buffer);
			previous_frame.buffer=NULL;
		}
		if (previous_frame.buffer==NULL)
		{
			previous_frame.width = picture->width;
			previous_frame.height = picture->height;
			frame_size = ( previous_frame.width * previous_frame.height )*3/2;
			printf("Allocating previous frame.\n");
			previous_frame.buffer=vp_os_malloc( frame_size );
		}

	/* Copy the current frame in a buffer so it can be encoded at next stage call */
		if (previous_frame.buffer!=NULL)
		{
			char * dest = previous_frame.buffer;
			int size = picture->width*picture->height;
			vp_os_memcpy(dest,picture->y_buf,size);

			dest+=size;
			size /= 4;
			vp_os_memcpy(dest,picture->cb_buf,size);

			dest+=size;
			vp_os_memcpy(dest,picture->cr_buf,size);
		}
		}
  }


  else
	{
		if(cfg->startRec==VIDEO_RECORD_STOP && flag_video_file_open)
		{
			close_video_file();
			flag_video_file_open=0;
		}
	}

  vp_os_mutex_unlock( &out->lock );

  return C_OK;
}



/******************************************************************************************************************************************/


C_RESULT video_stage_ffmpeg_recorder_close(video_stage_ffmpeg_recorder_config_t *cfg)
{
  if( cfg->fp != NULL )
    fclose( cfg->fp );

  return C_OK;
}



#endif
