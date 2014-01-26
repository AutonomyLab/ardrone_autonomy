/*
 *  video_stage_decoder.c
 *  ARDroneLib
 *
 *  Created by n.brulez on 02/08/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#if defined (FFMPEG_SUPPORT) || defined (ITTIAM_SUPPORT)

#include <ardrone_tool/Video/video_stage_decoder.h>
#include <video_encapsulation.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <stdio.h>

#ifndef USE_ANDROID
# ifdef ITTIAM_SUPPORT
#  define mp4h264_open ittiam_stage_decoding_open
#  define mp4h264_transform ittiam_stage_decoding_transform
#  define mp4h264_close ittiam_stage_decoding_close
static const int resetDecoderOnStreamChange = 1;
# else // FFMPEG
#  define mp4h264_open ffmpeg_stage_decoding_open
#  define mp4h264_transform ffmpeg_stage_decoding_transform
#  define mp4h264_close ffmpeg_stage_decoding_close
static const int resetDecoderOnStreamChange = 0;
# endif
#else // ANDROID
typedef enum {
    NEON_SUPPORT_UNKNOWN = 0,
    NEON_SUPPORT_OK,
    NEON_SUPPORT_FAIL,
} neon_status_t;

static neon_status_t neonStatus = NEON_SUPPORT_FAIL; // <<< FORCE FFMPEG <<<
static int resetDecoderOnStreamChange = 0; // NON CONST as it will be set at runtime

C_RESULT mp4h264_open (mp4h264_config_t *cfg);
C_RESULT mp4h264_transform (mp4h264_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT mp4h264_close (mp4h264_config_t *cfg);
#endif

/**
 * DEBUG ZONE
 */
#if defined(TARGET_OS_IPHONE) || defined (TARGET_OS_IPHONE_SIMULATOR)
#include <mach/mach_time.h>
#endif
float DEBUG_decodingTimeUsec = 0.0;

#define ENABLE_VIDEO_STAGE_DECODER_DEBUG (0)

const vp_api_stage_funcs_t video_decoding_funcs = {
    (vp_api_stage_handle_msg_t) NULL,
    (vp_api_stage_open_t) video_stage_decoder_open,
    (vp_api_stage_transform_t) video_stage_decoder_transform,
    (vp_api_stage_close_t) video_stage_decoder_close
};

#if ENABLE_VIDEO_STAGE_DECODER_DEBUG || defined (DEBUG)
#define PDBG(...)                                                       \
    do                                                                  \
    {                                                                   \
        printf ("VIDEO_STAGE_DECODER (%s@%d) : ", __FUNCTION__, __LINE__); \
        printf (__VA_ARGS__);                                           \
        printf ("\n");                                                  \
    } while (0)
#else
#define PDBG(...)
#endif


#define ALLOC_CHECK(POINTER, SIZE)                      \
    do                                                  \
    {                                                   \
        POINTER = vp_os_calloc (SIZE, 1);               \
        if (NULL == POINTER)                            \
        {                                               \
            printf ("Unable to alloc %s\n", #POINTER);  \
            return C_FAIL;                              \
        }                                               \
    } while (0)

#ifdef USE_LINUX
#include <unistd.h>
int video_stage_decoder_fakeLatency = 0;   /* Simulate a slow decoder inside AR.Drone Navigation */
#endif


parrot_video_encapsulation_codecs_t video_stage_decoder_lastDetectedCodec = 0;

C_RESULT video_stage_decoder_open (video_decoder_config_t *cfg)
{
    // Allocate internal datas
    ALLOC_CHECK (cfg->vlibConf, sizeof (vlib_stage_decoding_config_t));
    ALLOC_CHECK (cfg->vlibOut, sizeof (vp_api_io_data_t));
    ALLOC_CHECK (cfg->mp4h264Conf, sizeof (mp4h264_config_t));
    ALLOC_CHECK (cfg->mp4h264Out, sizeof (vp_api_io_data_t));

    // Fill alloc'd structs with data from cfg
    // --> MPEG4 / H264
    cfg->mp4h264Conf->dst_picture.format = cfg->dst_picture->format;
    // --> VLIB
    cfg->vlibConf->width = cfg->dst_picture->width;
    cfg->vlibConf->height = cfg->dst_picture->height;
    cfg->vlibConf->picture = cfg->dst_picture;
    cfg->vlibConf->luma_only = FALSE;
    cfg->vlibConf->block_mode_enable = TRUE;

    switch (cfg->dst_picture->format)
    {
    case PIX_FMT_RGB565:
        cfg->bpp=2;
        break;
    case PIX_FMT_RGB24:
        cfg->bpp=3;
        break;
    default:
        cfg->bpp=0;
        break;
    }


    vlib_stage_decoding_open (cfg->vlibConf);
    mp4h264_open (cfg->mp4h264Conf);

    return C_OK;
}

static inline bool_t havePaVE (uint8_t *buffer)
{
    if (buffer [0] == 'P' &&
        buffer [1] == 'a' &&
        buffer [2] == 'V' &&
        buffer [3] == 'E')
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

C_RESULT video_stage_decoder_transform (video_decoder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    C_RESULT retVal  = C_OK;
    bool_t useVlib   = FALSE;
    bool_t useMP4H264 = FALSE;
    vp_api_io_data_t *outToCopy = NULL;
    uint8_t *buffer = in->buffers[in->indexBuffer];
    static int lastDecodedStreamID = -1;

#if defined(TARGET_OS_IPHONE) || defined (TARGET_OS_IPHONE_SIMULATOR)
    uint64_t startTime = 0;
    uint64_t stopTime = 0;
    uint64_t elapsedNano = 0;
    static mach_timebase_info_data_t sTimebaseInfo;
    if (0 == sTimebaseInfo.denom)
    {
        mach_timebase_info (&sTimebaseInfo);
    }
    startTime = mach_absolute_time ();
#endif



    // Check for PaVE
    if (havePaVE (buffer))
    {
        parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)buffer;

        video_stage_decoder_lastDetectedCodec = PaVE->video_codec;

        if (lastDecodedStreamID!=-1 && lastDecodedStreamID!=PaVE->stream_id && 1 == resetDecoderOnStreamChange)
        {
            PRINT("Resetting the video decoder.\n");
            video_stage_decoder_close(cfg);
            video_stage_decoder_open(cfg);
        }
        lastDecodedStreamID=PaVE->stream_id;


        if (CODEC_VLIB == PaVE->video_codec ||
            CODEC_P264 == PaVE->video_codec)
        {
            useVlib = TRUE;
        }
        else if (CODEC_MPEG4_VISUAL == PaVE->video_codec ||
                 CODEC_MPEG4_AVC == PaVE->video_codec)
        {
            useMP4H264 = TRUE;
        }
        else
        {
            // Unknown codec
            return C_FAIL;
        }
    }
    else if ( ((*(uint32_t*)buffer) & 0xFFFE7C00 )== 0 )  /* true if a UVLC or P264 header is present */
    {
        /* Test bits 15 and 16 of the header to differenciate UVLC and P264 */
        video_stage_decoder_lastDetectedCodec = ( ((*(uint32_t*)buffer) & 0x8000 )== 0x8000 ) ? CODEC_VLIB : CODEC_P264;
        // No PaVE -> give to VLIB
        useVlib = TRUE;
    }
    else
    {
        // Unknown codec
        printf(" -- Critical error : unrecognized codec  (first bytes : %x %x %x %x %x) --\n",
               buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
        return C_FAIL;
    }

    if (useVlib)
    {
        cfg->vlibConf->num_picture_decoded = cfg->num_picture_decoded;

        retVal = vlib_stage_decoding_transform (cfg->vlibConf, in, cfg->vlibOut);
        if (C_FAIL == retVal)
        {
            return retVal;
        }
        cfg->num_picture_decoded = cfg->vlibConf->num_picture_decoded;
        cfg->num_frames = cfg->vlibConf->controller.num_frames;

        cfg->src_picture->height = cfg->vlibConf->controller.height;
        cfg->src_picture->width = cfg->vlibConf->controller.width;
        cfg->rowstride = cfg->dst_picture->width * cfg->bpp; // Size of the buffer we got for VLIB
        cfg->vlibOut->size = cfg->rowstride * cfg->dst_picture->height;
        outToCopy = cfg->vlibOut;
    }

    if (useMP4H264)
    {
        parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)buffer;

        cfg->mp4h264Conf->num_picture_decoded = cfg->num_picture_decoded;

        retVal = mp4h264_transform (cfg->mp4h264Conf, in, cfg->mp4h264Out);
        if (C_FAIL == retVal)
        {
            return retVal;
        }
        cfg->num_picture_decoded = cfg->mp4h264Conf->num_picture_decoded;
        cfg->num_frames = PaVE->frame_number;

        cfg->dst_picture->height = cfg->mp4h264Conf->dst_picture.height;
        cfg->dst_picture->width = cfg->mp4h264Conf->dst_picture.width;
        cfg->src_picture->height = cfg->mp4h264Conf->src_picture.height;
        cfg->src_picture->width = cfg->mp4h264Conf->src_picture.width;
        cfg->rowstride = cfg->dst_picture->width * cfg->bpp; // Size of the actual picture, alloc'd by MPEG4 / H264 stage
        outToCopy = cfg->mp4h264Out;
    }

    if (NULL == outToCopy)
    {
        retVal = C_FAIL;
    }
    else
    {
        out->numBuffers = outToCopy->numBuffers;
        out->buffers = outToCopy->buffers;
        out->indexBuffer = outToCopy->indexBuffer;
        out->size = outToCopy->size;
        out->lineSize = outToCopy->lineSize;
        out->status = outToCopy->status;

        retVal = C_OK;
    }

#if defined(TARGET_OS_IPHONE) || defined (TARGET_OS_IPHONE_SIMULATOR)
    stopTime = mach_absolute_time ();
    elapsedNano = (stopTime - startTime) * sTimebaseInfo.numer / sTimebaseInfo.denom;
    if (0.0 == DEBUG_decodingTimeUsec)
    {
        DEBUG_decodingTimeUsec = elapsedNano / 1000.0;
    }
    else
    {
        DEBUG_decodingTimeUsec = 0.9 * DEBUG_decodingTimeUsec + (0.1 * elapsedNano / 1000.0);
    }
#endif


#ifdef USE_LINUX
    usleep(video_stage_decoder_fakeLatency * 1000);
#endif


    return retVal;
}


C_RESULT video_stage_decoder_close (video_decoder_config_t *cfg)
{
    C_RESULT res, resVlib, resMp4h264;
    resVlib = vlib_stage_decoding_close (cfg->vlibConf);
    resMp4h264 = mp4h264_close (cfg->mp4h264Conf);
    res = (C_OK == resVlib && C_OK == resMp4h264 ) ? C_OK : C_FAIL;
    vp_os_free (cfg->vlibConf);
    cfg->vlibConf = NULL;
    vp_os_free (cfg->vlibOut);
    cfg->vlibOut = NULL;
    vp_os_free (cfg->mp4h264Conf);
    cfg->mp4h264Conf = NULL;
    vp_os_free (cfg->mp4h264Out);
    cfg->mp4h264Out = NULL;
    return res;
}

#ifdef USE_ANDROID
#define NEONCHECK_BUFFER_STRING_SIZE (512)
void checkNeonSupport ()
{
    neon_status_t loc_neonStat = NEON_SUPPORT_FAIL;
    resetDecoderOnStreamChange = 0;
    FILE *cpuInfo = fopen ("/proc/cpuinfo", "r");
    if (NULL == cpuInfo)
    {
        return;
    }

    char *neonCheckStrBuf = vp_os_malloc (NEONCHECK_BUFFER_STRING_SIZE * sizeof (char));
    if (NULL == neonCheckStrBuf)
    {
        fclose (cpuInfo);
        return;
    }

    while (NULL != fgets (neonCheckStrBuf, NEONCHECK_BUFFER_STRING_SIZE, cpuInfo))
    {
        char *supportTest = strstr (neonCheckStrBuf, "neon");
        if (NULL != supportTest)
        {
            loc_neonStat = NEON_SUPPORT_OK;
            resetDecoderOnStreamChange = 1;
            break;
        }
    }

    vp_os_free (neonCheckStrBuf);
    neonCheckStrBuf = NULL;
    fclose (cpuInfo);

    neonStatus = loc_neonStat;
}

C_RESULT mp4h264_open (mp4h264_config_t *cfg)
{
#if ITTIAM_SUPPORT
    if (NEON_SUPPORT_UNKNOWN == neonStatus)
        checkNeonSupport ();
    if (NEON_SUPPORT_OK == neonStatus)
        return ittiam_stage_decoding_open ((ittiam_stage_decoding_config_t *)cfg);
    else
#endif
        return ffmpeg_stage_decoding_open (cfg);
}

C_RESULT mp4h264_transform (mp4h264_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
#if ITTIAM_SUPPORT
    if (NEON_SUPPORT_UNKNOWN == neonStatus)
        checkNeonSupport ();
    if (NEON_SUPPORT_OK == neonStatus)
        return ittiam_stage_decoding_transform ((ittiam_stage_decoding_config_t *)cfg, in, out);
    else
#endif
        return ffmpeg_stage_decoding_transform (cfg, in, out);
}

C_RESULT mp4h264_close (mp4h264_config_t *cfg)
{
#if ITTIAM_SUPPORT
    if (NEON_SUPPORT_UNKNOWN == neonStatus)
        checkNeonSupport ();
    if (NEON_SUPPORT_OK == neonStatus)
        return ittiam_stage_decoding_close ((ittiam_stage_decoding_config_t *)cfg);
    else
#endif
        return ffmpeg_stage_decoding_close (cfg);
}
#endif

#endif //FFMPEG_SUPPORT || ITTIAM_SUPPORT
