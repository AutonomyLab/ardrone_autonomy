/*
 * ardrone_video_encapsuler.c
 * ARDroneLib
 *
 * Created by n.brulez on 18/08/11
 * Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#include "ardrone_video_encapsuler.h"
#include "ardrone_video_atoms.h"
#include "ardrone_tool/ardrone_version.h"

#include <unistd.h>
#include <sys/types.h>
#include <time.h>

#ifdef USE_ELINUX
#include <sys/stat.h>
#include <fcntl.h>
#endif
#include <stdlib.h>
#include <string.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#ifdef _WIN32
#include <Winsock2.h>
#else
#include <arpa/inet.h>
#ifndef USE_ANDROID
#include <ftw.h> // For file cleanup in subdirs
#else
#include <utils/AR_Ftw.h>
#endif
#endif

#define ENCAPSULER_SMALL_STRING_SIZE (30)
#define ENCAPSULER_INFODATA_MAX_SIZE (256)

/* The structure is initialised to an invalid value
   so we won't set any position in videos unless we got a
   valid ardrone_video_set_gps_infos() call */
static struct videoGpsInfos_s {
    double latitude;
    double longitude;
    double altitude;
} videoGpsInfos = {500.0, 500.0, 500.0};

#define ENCAPSULER_DEBUG_ENABLE (0)
#define ENCAPSULER_FLUSH_ON_EACH_WRITE (0)

// File extension for temporary files
#define TEMPFILE_EXT ".tmpvid"

// File extension for informations files (frame sizes / types)
#define INFOFILE_EXT ".infovid"

#define ENCAPSULER_ERROR(...)                                           \
    do                                                                  \
    {                                                                   \
        fprintf (stderr, "AR.Drone video encapsuler error (%s @ %d) : ", __FUNCTION__, __LINE__); \
        fprintf (stderr, __VA_ARGS__);                                  \
        fprintf (stderr, "\n");                                         \
    } while (0)

#if defined (DEBUG) || ENCAPSULER_DEBUG_ENABLE
#define ENCAPSULER_DEBUG(...)                                           \
    do                                                                  \
    {                                                                   \
        fprintf (stdout, "AR.Drone video encapsuler debug (%s @ %d) : ", __FUNCTION__, __LINE__); \
        fprintf (stdout, __VA_ARGS__);                                  \
        fprintf (stdout, "\n");                                         \
    } while (0)
#else
#define ENCAPSULER_DEBUG(...)
#endif

#if ENCAPSULER_FLUSH_ON_EACH_WRITE || defined USE_ELINUX
#define ENCAPSULER_FFLUSH fflush
#else
#define ENCAPSULER_FFLUSH(...)
#endif

#define EMPTY_ATOM(NAME) movie_atom_t * NAME##Atom = atomFromData (0, #NAME, NULL)

#define ENCAPSULER_CLEANUP(FUNC,PTR)            \
    do                                          \
    {                                           \
        if (NULL != PTR)                        \
        {                                       \
            FUNC(PTR);                          \
        }                                       \
    } while (0)



ardrone_video_t *ardrone_video_start (const char *videoPath, int fps, ardrone_video_type_t vType, ardrone_video_error_t *error)
{
#ifdef USE_ELINUX
    int fd;
#endif

    if (NULL == error)
    {
        ENCAPSULER_ERROR ("error pointer must not be null");
        return NULL;
    }
    if (NULL == videoPath)
    {
        ENCAPSULER_ERROR ("videoPath pointer must not be null");
        *error = ARDRONE_VIDEO_BAD_ARGS;
        return NULL;
    }

    ardrone_video_t *retVideo = vp_os_malloc (sizeof (ardrone_video_t));
    if (NULL == retVideo)
    {
        ENCAPSULER_ERROR ("Unable to allocate retVideo");
        *error = ARDRONE_VIDEO_GENERIC_ERROR;
        return NULL;
    }

    vp_os_mutex_init(&retVideo->mutex);
    vp_os_mutex_lock(&retVideo->mutex);
    retVideo->version = ARDRONE_VIDEO_VERSION_NUMBER;
    retVideo->fps = (uint32_t)fps;
    retVideo->lastFrameNumber = UINT32_MAX;
    retVideo->currentFrameSize = 0;
    retVideo->lastFrameType = FRAME_TYPE_UNKNNOWN;
    retVideo->videoType = vType;
    retVideo->videoCodec = 0;
    retVideo->width = 0;
    retVideo->height = 0;
    retVideo->sps = NULL;
    retVideo->spsSize = 0;
    retVideo->pps = NULL;
    retVideo->ppsSize = 0;
    snprintf (retVideo->infoFilePath, ARDRONE_VIDEO_PATH_MAX_SIZE, "%s%s", videoPath, INFOFILE_EXT);
    snprintf (retVideo->tempFilePath, ARDRONE_VIDEO_PATH_MAX_SIZE, "%s%s", videoPath, TEMPFILE_EXT);
    snprintf (retVideo->outFilePath,  ARDRONE_VIDEO_PATH_MAX_SIZE, "%s", videoPath);
    retVideo->infoFile = fopen (retVideo->infoFilePath, "w+b");
    if (NULL == retVideo->infoFile)
    {
        ENCAPSULER_ERROR ("Unable to open file %s for writing", retVideo->infoFilePath);
        *error = ARDRONE_VIDEO_GENERIC_ERROR;
        vp_os_mutex_unlock(&retVideo->mutex);
        vp_os_free (retVideo);
        retVideo = NULL;
        return NULL;
    }

#ifdef USE_ELINUX
    /* Video recording on the AR.Drone : minimize caching ? */
    fd = open(retVideo->tempFilePath,/*O_DIRECT |*/ O_CREAT | O_RDWR | O_LARGEFILE);
    /* Make an associated stream */
    retVideo->outFile = fdopen (fd, "w+b");
#else
    retVideo->outFile = fopen (retVideo->tempFilePath, "w+b");
#endif

    if (NULL == retVideo->outFile)
    {
        ENCAPSULER_ERROR ("Unable to open file %s for writing", videoPath);
        *error = ARDRONE_VIDEO_GENERIC_ERROR;
        fclose (retVideo->infoFile);
        vp_os_mutex_unlock(&retVideo->mutex);
        vp_os_free (retVideo);
        retVideo = NULL;
        return NULL;
    }
    retVideo->framesCount = 0;
    retVideo->mdatAtomOffset = 0;
    retVideo->framesDataOffset = 0;

    retVideo->creationTime = time (NULL);
    retVideo->droneVersion = ARDRONE_VERSION ();

    *error = ARDRONE_VIDEO_NO_ERROR;
    vp_os_mutex_unlock(&retVideo->mutex);

    return retVideo;
}

ardrone_video_error_t ardrone_video_addFrame (ardrone_video_t *video, uint8_t *frame)
{
    if (NULL == video)
    {
        ENCAPSULER_ERROR ("video pointer must not be null");
        return ARDRONE_VIDEO_BAD_ARGS;
    }
    if (NULL == frame)
    {
        ENCAPSULER_ERROR ("frame pointer must not be null");
        return ARDRONE_VIDEO_BAD_ARGS;
    }

    vp_os_mutex_lock(&video->mutex);
    parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)frame;
    uint8_t *data = &frame[PaVE->header_size];
    if ('P' != PaVE->signature [0] ||
        'a' != PaVE->signature [1] ||
        'V' != PaVE->signature [2] ||
        'E' != PaVE->signature [3])
    {
        ENCAPSULER_ERROR ("frame does not contain PaVE informations");
        vp_os_mutex_unlock(&video->mutex);

        return ARDRONE_VIDEO_BAD_ARGS;
    }

    if (0 == PaVE->payload_size)
    {
        // Do nothing
        ENCAPSULER_DEBUG ("Empty frame\n");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_NO_ERROR;
    }

    if (0 == video->width)
    {

        if (FRAME_TYPE_IDR_FRAME != PaVE->frame_type)
        {
            // Wait until we get an IFrame for starting the actual recording
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_WAITING_FOR_IFRAME;
        }

        // Ensure that the codec is h.264 (mp4 not supported)
        if (CODEC_MPEG4_AVC != PaVE->video_codec)
        {
            ENCAPSULER_ERROR ("Only h.264 codec is supported");
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_BAD_CODEC;
        }

        // Init video structure
        video->width = PaVE->display_width;
        video->height = PaVE->display_height;
        video->videoCodec = PaVE->video_codec;

        // If codec is H.264, save SPS/PPS
        if (CODEC_MPEG4_AVC == video->videoCodec)
        {
            if (0 == PaVE->header1_size)
            {
                ENCAPSULER_ERROR ("IFrame don't have SPS/PPS infos\n");
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_GENERIC_ERROR;
            }
            if (0 == PaVE->header2_size)
            {
                // Header 1 size contains the SPS + PPS size : we'll need to search the "00 00 00 01" pattern to find each header size
                // Search start at index 4 to avoid finding the SPS "00 00 00 01" tag
                uint32_t searchIndex = 4;
                for (searchIndex = 4; searchIndex <= PaVE->header1_size - 4; searchIndex ++)
                {
                    if (0 == data[searchIndex  ] &&
                        0 == data[searchIndex+1] &&
                        0 == data[searchIndex+2] &&
                        1 == data[searchIndex+3])
                    {
                        break;
                    }
                }
                video->spsSize = searchIndex - 4;
                video->ppsSize = PaVE->header1_size - searchIndex - 4;
            }
            else
            {
                video->spsSize = PaVE->header1_size - 4;
                video->ppsSize = PaVE->header2_size - 4;
            }
            video->sps = vp_os_malloc (video->spsSize);
            video->pps = vp_os_malloc (video->ppsSize);
            if (NULL == video->sps || NULL == video->pps)
            {
                ENCAPSULER_ERROR ("Unable to allocate SPS/PPS buffers");
                if (NULL != video->sps)
                {
                    vp_os_free (video->sps);
                    video->sps = NULL;
                }
                if (NULL != video->pps)
                {
                    vp_os_free (video->pps);
                    video->pps = NULL;
                }
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_GENERIC_ERROR;
            }
            vp_os_memcpy (video->sps, &data[4], video->spsSize);
            vp_os_memcpy (video->pps, &data[8+video->spsSize], video->ppsSize);
        }

        // Start to write file
        rewind (video->outFile);
        movie_atom_t *freeAtomIfNeeded = NULL;
        movie_atom_t *ftypAtom = ftypAtomForFormatAndCodecWithOffset (video->videoType, video->videoCodec, &(video->framesDataOffset), &freeAtomIfNeeded);
        if (NULL == ftypAtom)
        {
            ENCAPSULER_ERROR ("Unable to create ftyp atom");
            vp_os_mutex_unlock(&video->mutex);
            if (NULL != freeAtomIfNeeded)
            {
                vp_os_free (freeAtomIfNeeded);
                freeAtomIfNeeded = NULL;
            }
            return ARDRONE_VIDEO_GENERIC_ERROR;
        }
        if (-1 == writeAtomToFile (&ftypAtom, video->outFile))
        {
            ENCAPSULER_ERROR ("Unable to write ftyp atom");
            vp_os_mutex_unlock(&video->mutex);
            if (NULL != freeAtomIfNeeded)
            {
                vp_os_free (freeAtomIfNeeded);
                freeAtomIfNeeded = NULL;
            }
            return ARDRONE_VIDEO_FILE_ERROR;
        }

        if (NULL != freeAtomIfNeeded)
        {
            /* We need to write a free atom to ensure that the offset of the mdat atom remains the same
               This is useful for mp4<->mov conversions */
            if (-1 == writeAtomToFile (&freeAtomIfNeeded, video->outFile))
            {
                ENCAPSULER_ERROR ("Unable to write free atom");
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_FILE_ERROR;
            }
        }

        if (-1 == fseek (video->outFile, video->framesDataOffset, SEEK_SET))
        {
            ENCAPSULER_ERROR ("Unable to set file write pointer to %d", video->framesDataOffset);
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        video->mdatAtomOffset = video->framesDataOffset - 16;

        // Write video infos to info file header
        uint32_t descriptorSize = sizeof (ardrone_video_t) + video->spsSize + video->ppsSize;
        // Write total length
        if (1 != fwrite (&descriptorSize, sizeof (uint32_t), 1, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write size of video descriptor");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        // Write video_t info
        if (1 != fwrite (video, sizeof (ardrone_video_t), 1, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write video descriptor");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        // Write SPS
        if (video->spsSize != fwrite (video->sps, sizeof (uint8_t), video->spsSize, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write sps header");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        // Write PPS
        if (video->ppsSize != fwrite (video->pps, sizeof (uint8_t), video->ppsSize, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write pps header");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
    }

    // Normal operation : file pointer is at end of file
    if (video->videoCodec != PaVE->video_codec ||
        video->width != PaVE->display_width ||
        video->height != PaVE->display_height)
    {
        ENCAPSULER_ERROR ("New frame don't match the video size/codec");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_BAD_CODEC;
    }

    video->lastFrameNumber = PaVE->frame_number;
    video->currentFrameSize = 0;
    video->lastFrameType = FRAME_TYPE_UNKNNOWN;
    video->framesCount++;
    if (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT <= video->framesCount)
    {
        ENCAPSULER_ERROR ("Video contains already %d frames, which is the maximum", ARDRONE_VIDEO_FRAMES_COUNT_LIMIT);
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_GENERIC_ERROR;
    }

    uint32_t frameSize = PaVE->payload_size;
    uint8_t *myData = vp_os_malloc (frameSize);
    if (NULL == myData)
    {
        ENCAPSULER_ERROR ("Unable to allocate local data");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_GENERIC_ERROR;
    }
    vp_os_memcpy (myData, data, frameSize);
    // Modify frames before writing
    if (FRAME_TYPE_I_FRAME != PaVE->frame_type &&
        FRAME_TYPE_IDR_FRAME != PaVE->frame_type)
    {
        // P_Frame : only first 4 octets
        // 00 00 00 01 to frameSize -4
        uint32_t f_size = htonl (frameSize - 4);
        memcpy (myData, &f_size, sizeof (uint32_t));
    }
    else
    {
        // I_Frame : modify all 00 00 00 01 patterns
        // to the size of the following SPS/PPS/Frame
        // SPS   00 00 00 01 -> sps_size-4
        // PPS   00 00 00 01 -> pps_size-4
        // Frame 00 00 00 01 -> frameSize - (sps_size + pps_size + 4)
        uint8_t sps_size = PaVE->header1_size;
        uint8_t pps_size = PaVE->header2_size;
        uint32_t sps_size_NE = htonl (sps_size - 4);
        uint32_t pps_size_NE = htonl (pps_size - 4);
        uint32_t f_size = htonl (frameSize - (sps_size + pps_size + 4));
        uint32_t pps_offset = sps_size;
        uint32_t frame_offset = pps_offset + pps_size;

        memcpy ( myData,               &sps_size_NE, sizeof (uint32_t));
        memcpy (&myData[pps_offset],   &pps_size_NE, sizeof (uint32_t));
        memcpy (&myData[frame_offset], &f_size,      sizeof (uint32_t));
    }

    if (frameSize != fwrite (myData, 1, frameSize, video->outFile))
    {
        vp_os_free (myData);
        myData = NULL;
        ENCAPSULER_ERROR ("Unable to write frame into data file");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_FILE_ERROR;
    }
    else
    {
        ENCAPSULER_FFLUSH (video->outFile);
    }
    vp_os_free (myData);
    myData = NULL;

    char infoData [ENCAPSULER_INFODATA_MAX_SIZE] = {0};
    char fTypeChar = 'p';
    if (FRAME_TYPE_I_FRAME == PaVE->frame_type ||
        FRAME_TYPE_IDR_FRAME == PaVE->frame_type)
    {
        fTypeChar = 'i';
    }
    snprintf (infoData, ENCAPSULER_INFODATA_MAX_SIZE, ARDRONE_VIDEO_INFO_PATTERN, frameSize, fTypeChar);
    uint32_t infoLen = strlen (infoData);
    if (infoLen != fwrite (infoData, 1, infoLen, video->infoFile))
    {
        ENCAPSULER_ERROR ("Unable to write frameInfo into info file");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_FILE_ERROR;
    }
    else
    {
        ENCAPSULER_FFLUSH (video->infoFile);
    }

    vp_os_mutex_unlock(&video->mutex);
    return ARDRONE_VIDEO_NO_ERROR;
}


ardrone_video_error_t ardrone_video_addSlice (ardrone_video_t *video, uint8_t *slice)
{
    static uint8_t *myData = NULL;
    static int myDataSize = 0;

    if (NULL == video)
    {
        ENCAPSULER_ERROR ("video pointer must not be null");
        return ARDRONE_VIDEO_BAD_ARGS;
    }
    if (NULL == slice)
    {
        ENCAPSULER_ERROR ("slice pointer must not be null");
        return ARDRONE_VIDEO_BAD_ARGS;
    }

    vp_os_mutex_lock(&video->mutex);
    parrot_video_encapsulation_t *PaVE = (parrot_video_encapsulation_t *)slice;
    uint8_t *data = &slice[PaVE->header_size];
    if ('P' != PaVE->signature [0] ||
        'a' != PaVE->signature [1] ||
        'V' != PaVE->signature [2] ||
        'E' != PaVE->signature [3])
    {
        ENCAPSULER_ERROR ("slice does not contain PaVE informations");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_BAD_ARGS;
    }

    if (0 == PaVE->payload_size)
    {
        // Do nothing
        ENCAPSULER_DEBUG ("Empty slice\n");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_NO_ERROR;
    }

    // First slice
    if (0 == video->width)
    {

        if (FRAME_TYPE_IDR_FRAME != PaVE->frame_type ||
            0 != PaVE->slice_index)
        {
            // Wait until we get an IFrame-slice 1 to start the actual recording
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_WAITING_FOR_IFRAME;
        }

        // Ensure that the codec is h.264 (mp4 not supported)
        if (CODEC_MPEG4_AVC != PaVE->video_codec)
        {
            ENCAPSULER_ERROR ("Only h.264 codec is supported");
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_BAD_CODEC;
        }

        // Init video structure
        video->width = PaVE->display_width;
        video->height = PaVE->display_height;
        video->videoCodec = PaVE->video_codec;

        // If codec is H.264, save SPS/PPS
        if (CODEC_MPEG4_AVC == video->videoCodec)
        {
            if (0 == PaVE->header1_size)
            {
                ENCAPSULER_ERROR ("IFrame don't have SPS/PPS infos\n");
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_GENERIC_ERROR;
            }
            if (0 == PaVE->header2_size)
            {
                // Header 1 size contains the SPS + PPS size : we'll need to search the "00 00 00 01" pattern to find each header size
                // Search start at index 4 to avoid finding the SPS "00 00 00 01" tag
                uint32_t searchIndex = 4;
                for (searchIndex = 4; searchIndex <= PaVE->header1_size - 4; searchIndex ++)
                {
                    if (0 == data[searchIndex  ] &&
                        0 == data[searchIndex+1] &&
                        0 == data[searchIndex+2] &&
                        1 == data[searchIndex+3])
                    {
                        break;
                    }
                }
                video->spsSize = searchIndex - 4;
                video->ppsSize = PaVE->header1_size - searchIndex - 4;
            }
            else
            {
                video->spsSize = PaVE->header1_size - 4;
                video->ppsSize = PaVE->header2_size - 4;
            }
            video->sps = vp_os_malloc (video->spsSize);
            video->pps = vp_os_malloc (video->ppsSize);
            if (NULL == video->sps || NULL == video->pps)
            {
                ENCAPSULER_ERROR ("Unable to allocate SPS/PPS buffers");
                if (NULL != video->sps)
                {
                    vp_os_free (video->sps);
                    video->sps = NULL;
                }

                if (NULL != video->pps)
                {
                    vp_os_free (video->pps);
                    video->pps = NULL;
                }
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_GENERIC_ERROR;
            }
            vp_os_memcpy (video->sps, &data[4], video->spsSize);
            vp_os_memcpy (video->pps, &data[8+video->spsSize], video->ppsSize);
        }

        // Start to write file
        rewind (video->outFile);
        movie_atom_t *freeAtomIfNeeded = NULL;
        movie_atom_t *ftypAtom = ftypAtomForFormatAndCodecWithOffset (video->videoType, video->videoCodec, &(video->framesDataOffset), &freeAtomIfNeeded);
        if (NULL == ftypAtom)
        {
            ENCAPSULER_ERROR ("Unable to create ftyp atom");
            vp_os_mutex_unlock(&video->mutex);
            if (NULL != freeAtomIfNeeded)
            {
                vp_os_free (freeAtomIfNeeded);
                freeAtomIfNeeded = NULL;
            }
            return ARDRONE_VIDEO_GENERIC_ERROR;
        }
        if (-1 == writeAtomToFile (&ftypAtom, video->outFile))
        {
            ENCAPSULER_ERROR ("Unable to write ftyp atom");
            vp_os_mutex_unlock(&video->mutex);
            if (NULL != freeAtomIfNeeded)
            {
                vp_os_free (freeAtomIfNeeded);
                freeAtomIfNeeded = NULL;
            }
            return ARDRONE_VIDEO_FILE_ERROR;
        }

        if (NULL != freeAtomIfNeeded)
        {
            /* We need to write a free atom to ensure that the offset of the mdat atom remains the same
               This is useful for mp4<->mov conversions */
            if (-1 == writeAtomToFile (&freeAtomIfNeeded, video->outFile))
            {
                ENCAPSULER_ERROR ("Unable to write free atom");
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_FILE_ERROR;
            }
        }

        if (-1 == fseek (video->outFile, video->framesDataOffset, SEEK_SET))
        {
            ENCAPSULER_ERROR ("Unable to set file write pointer to %d", video->framesDataOffset);
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        video->mdatAtomOffset = video->framesDataOffset - 16;

        // Write video infos to info file header
        uint32_t descriptorSize = sizeof (ardrone_video_t) + video->spsSize + video->ppsSize;
        // Write total length
        if (1 != fwrite (&descriptorSize, sizeof (uint32_t), 1, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write size of video descriptor");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        // Write video_t info
        if (1 != fwrite (video, sizeof (ardrone_video_t), 1, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write video descriptor");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        // Write SPS
        if (video->spsSize != fwrite (video->sps, sizeof (uint8_t), video->spsSize, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write sps header");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }
        // Write PPS
        if (video->ppsSize != fwrite (video->pps, sizeof (uint8_t), video->ppsSize, video->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to write pps header");
            vp_os_mutex_unlock (&video->mutex);
            return ARDRONE_VIDEO_FILE_ERROR;
        }

    }

    // Normal operation : file pointer is at end of file
    if (video->videoCodec != PaVE->video_codec ||
        video->width != PaVE->display_width ||
        video->height != PaVE->display_height)
    {
        ENCAPSULER_ERROR ("New slice don't match the video size/codec");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_BAD_CODEC;
    }

    // New frame, write all infos about last one
    // Use lastFrameType to check that we don't write infos about a null frame
    if (video->lastFrameNumber != PaVE->frame_number)
    {
        if (FRAME_TYPE_UNKNNOWN != video->lastFrameType)
        {
            char infoData [ENCAPSULER_INFODATA_MAX_SIZE] = {0};
            char fTypeChar = 'p';
            if (FRAME_TYPE_I_FRAME == video->lastFrameType ||
                FRAME_TYPE_IDR_FRAME == video->lastFrameType)
            {
                fTypeChar = 'i';
            }
            snprintf (infoData, ENCAPSULER_INFODATA_MAX_SIZE, ARDRONE_VIDEO_INFO_PATTERN, video->currentFrameSize, fTypeChar);
            uint32_t infoLen = strlen (infoData);
            if (infoLen != fwrite (infoData, 1, infoLen, video->infoFile))
            {
                ENCAPSULER_ERROR ("Unable to write frameInfo into info file");
                vp_os_mutex_unlock(&video->mutex);
                return ARDRONE_VIDEO_FILE_ERROR;
            }
            else
            {
                ENCAPSULER_FFLUSH (video->infoFile);
            }
        }
        video->currentFrameSize = 0;
        video->lastFrameType = PaVE->frame_type;
        video->lastFrameNumber = PaVE->frame_number;
        video->framesCount++;
        if (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT <= video->framesCount)
        {
            ENCAPSULER_ERROR ("Video contains already %d frames, which is the maximum", ARDRONE_VIDEO_FRAMES_COUNT_LIMIT);
            vp_os_mutex_unlock(&video->mutex);
            return ARDRONE_VIDEO_GENERIC_ERROR;
        }
    }

    uint32_t sliceSize = PaVE->payload_size;
    if (sliceSize>0 && sliceSize>myDataSize) {  myDataSize=sliceSize; myData = vp_os_realloc(myData,sliceSize); }

    if (NULL == myData)
    {
        ENCAPSULER_ERROR ("Unable to allocate local data (%d->%d bytes) ",myDataSize,sliceSize);
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_GENERIC_ERROR;
    }
    vp_os_memcpy (myData, data, sliceSize);

    // Modify slices before writing
    if (0 != PaVE->slice_index ||
        (FRAME_TYPE_I_FRAME != PaVE->frame_type &&
         FRAME_TYPE_IDR_FRAME != PaVE->frame_type))
    {
        // P_Frame : only first 4 octets
        // I_Frame (not first slice) : only first 4 octets
        // 00 00 00 01 to sliceSize -4
        uint32_t f_size = htonl (sliceSize - 4);
        memcpy (myData, &f_size, sizeof (uint32_t));
    }
    else
    {
        // I_Frame first slice : modify all 00 00 00 01 patterns
        // to the size of the following SPS/PPS/Frame
        // SPS   00 00 00 01 -> sps_size-4
        // PPS   00 00 00 01 -> pps_size-4
        // Frame 00 00 00 01 -> sliceSize - (sps_size + pps_size + 4)
        uint8_t sps_size = PaVE->header1_size;
        uint8_t pps_size = PaVE->header2_size;
        uint32_t sps_size_NE = htonl (sps_size - 4);
        uint32_t pps_size_NE = htonl (pps_size - 4);
        uint32_t f_size = htonl (sliceSize - (sps_size + pps_size + 4));
        uint32_t pps_offset = sps_size;
        uint32_t slice_offset = pps_offset + pps_size;

        memcpy ( myData,               &sps_size_NE, sizeof (uint32_t));
        memcpy (&myData[pps_offset],   &pps_size_NE, sizeof (uint32_t));
        memcpy (&myData[slice_offset], &f_size,      sizeof (uint32_t));
    }

    if (sliceSize != fwrite (myData, 1, sliceSize, video->outFile))
    {
        /*vp_os_sfree (&myData);*/
        ENCAPSULER_ERROR ("Unable to write slice into data file");
        vp_os_mutex_unlock(&video->mutex);
        return ARDRONE_VIDEO_FILE_ERROR;
    }
    else
    {
        ENCAPSULER_FFLUSH (video->outFile);
    }

    video->currentFrameSize += sliceSize;
    /*vp_os_sfree (&myData);*/
    vp_os_mutex_unlock(&video->mutex);

    return ARDRONE_VIDEO_NO_ERROR;
}

/* Return 1 if the string was correctly generated, 0 otherwise */
int generateGpsString (char *gpsBuffer, int bufferSize)
{
    /* Buffer must be at least 27 character long
       - "+90.0000+180.0000+9999.000" is the longest possible gps string
       -> 26 chars + the null terminating one */
    if ((   27   > bufferSize)              ||
        (   90.0 < videoGpsInfos.latitude)  ||
        (  -90.0 > videoGpsInfos.latitude)  ||
        (  180.0 < videoGpsInfos.longitude) ||
        ( -180.0 > videoGpsInfos.longitude) ||
        ( 9999.0 < videoGpsInfos.altitude)  ||
        (-9999.0 > videoGpsInfos.altitude))
    {
        return 0;
    }
    snprintf (gpsBuffer, bufferSize -1, "%+.4f%+.4f%+.3f", videoGpsInfos.latitude, videoGpsInfos.longitude, videoGpsInfos.altitude);
    return 1;
}


ardrone_video_error_t ardrone_video_finish (ardrone_video_t **video)
{
    ardrone_video_error_t localError = ARDRONE_VIDEO_NO_ERROR;
    if (NULL == (*video))
    {
        ENCAPSULER_ERROR ("video pointer must not be null");
        localError = ARDRONE_VIDEO_BAD_ARGS;
    }

    ardrone_video_t *myVideo = NULL;

    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {
        vp_os_mutex_lock(&((*video)->mutex));
        myVideo = (*video); // ease of reading

        if (0 == myVideo->width)
        {
            // Video was not initialized
            ENCAPSULER_ERROR ("video was not initialized");
            localError =  ARDRONE_VIDEO_BAD_ARGS;
        } // No else
    }

    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {
        // For slice, we'll write infos about the last frame here
        if (0 != myVideo->currentFrameSize)
        {
            if (FRAME_TYPE_UNKNNOWN != myVideo->lastFrameType)
            {
                char infoData [ENCAPSULER_INFODATA_MAX_SIZE] = {0};
                char fTypeChar = 'p';
                if (FRAME_TYPE_I_FRAME == myVideo->lastFrameType ||
                    FRAME_TYPE_IDR_FRAME == myVideo->lastFrameType)
                {
                    fTypeChar = 'i';
                }
                snprintf (infoData, ENCAPSULER_INFODATA_MAX_SIZE, ARDRONE_VIDEO_INFO_PATTERN, myVideo->currentFrameSize, fTypeChar);

                uint32_t infoLen = strlen (infoData);
                if (infoLen != fwrite (infoData, 1, infoLen, myVideo->infoFile))
                {
                    ENCAPSULER_ERROR ("Unable to write frameInfo into info file");
                    vp_os_mutex_unlock(&myVideo->mutex);
                    localError = ARDRONE_VIDEO_FILE_ERROR;
                }
                else
                {
                    ENCAPSULER_FFLUSH (myVideo->infoFile);
                }
            }
        }
    }

    // Create internal buffers
    uint32_t *frameSizeBuffer   = NULL;
    uint32_t *frameSizeBufferNE = NULL;
    uint32_t *frameOffsetBuffer = NULL;
    uint32_t *iFrameIndexBuffer = NULL;
    uint8_t *frameIsIFrame      = NULL;

    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {
        // Init internal buffers
        frameSizeBuffer   = vp_os_calloc (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT, sizeof (uint32_t));
        frameSizeBufferNE = vp_os_calloc (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT, sizeof (uint32_t));
        frameOffsetBuffer = vp_os_calloc (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT, sizeof (uint32_t));
        iFrameIndexBuffer = vp_os_calloc (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT, sizeof (uint32_t));
        frameIsIFrame     = vp_os_calloc (ARDRONE_VIDEO_FRAMES_COUNT_LIMIT, sizeof (uint8_t) );

        if (NULL == frameSizeBuffer   ||
            NULL == frameSizeBufferNE ||
            NULL == frameOffsetBuffer ||
            NULL == iFrameIndexBuffer ||
            NULL == frameIsIFrame)
        {
            ENCAPSULER_ERROR ("Unable to allocate buffers for video finish");
            vp_os_free (frameSizeBuffer);
            frameSizeBuffer = NULL;
            vp_os_free (frameSizeBufferNE);
            frameSizeBufferNE = NULL;
            vp_os_free (frameOffsetBuffer);
            frameOffsetBuffer = NULL;
            vp_os_free (iFrameIndexBuffer);
            iFrameIndexBuffer = NULL;
            vp_os_free (frameIsIFrame);
            frameIsIFrame = NULL;
            vp_os_mutex_unlock(&myVideo->mutex);
            localError = ARDRONE_VIDEO_GENERIC_ERROR;
        }
    }

    uint64_t dataTotalSize = 0;
    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {
        // Init internal counters
        uint32_t nbFrames = 0;
        uint32_t nbIFrames = 0;
        uint32_t frameOffset = myVideo->framesDataOffset;

        // Read info file
        rewind (myVideo->infoFile);
        // Skip video descriptor
        uint32_t descriptorSize = 0;
        if (1 != fread(&descriptorSize, sizeof (uint32_t), 1, myVideo->infoFile))
        {
            ENCAPSULER_ERROR ("Unable to read descriptor size (probably an old .infovid file)");
            rewind (myVideo->infoFile);
        }
        else
        {
            fseek (myVideo->infoFile, descriptorSize, SEEK_CUR);
        }
        while (! feof (myVideo->infoFile) && nbFrames < myVideo->framesCount)
        {
            uint32_t fSize = -1;
            char fType = 'a';
            if (ARDRONE_VIDEO_NUM_MATCH_PATTERN ==
                fscanf (myVideo->infoFile, ARDRONE_VIDEO_INFO_PATTERN, &fSize, &fType))
            {
                frameSizeBuffer   [nbFrames] = fSize;
                frameSizeBufferNE [nbFrames] = htonl (fSize);
                frameOffsetBuffer [nbFrames] = htonl (frameOffset);
                frameOffset += fSize;
                if ('i' == fType)
                {
                    frameIsIFrame [nbFrames] = 1;
                    iFrameIndexBuffer [nbIFrames++] = htonl (nbFrames+1);
                }
                nbFrames ++;
                dataTotalSize += (uint64_t) fSize;
            }
        }

        // create atoms
        // Generating Atoms
        tzset ();
        struct tm *nowTm = localtime (&myVideo->creationTime);
        EMPTY_ATOM(moov);
        movie_atom_t *mvhdAtom = mvhdAtomFromFpsNumFramesAndDate (myVideo->fps, nbFrames, myVideo->creationTime - timezone + (3600 * nowTm->tm_isdst));
        EMPTY_ATOM(trak);
        movie_atom_t *tkhdAtom = tkhdAtomWithResolutionNumFramesFpsAndDate (myVideo->width, myVideo->height, nbFrames, myVideo->fps, myVideo->creationTime - timezone + (3600 * nowTm->tm_isdst));
        EMPTY_ATOM(mdia);
        movie_atom_t *mdhdAtom = mdhdAtomFromFpsNumFramesAndDate (myVideo->fps, nbFrames, myVideo->creationTime - timezone + (3600 * nowTm->tm_isdst));
        movie_atom_t *hdlrAtom = hdlrAtomForMdia ();
        EMPTY_ATOM(minf);
        movie_atom_t *vmhdAtom = vmhdAtomGen ();
        movie_atom_t *hdlr2Atom = hdlrAtomForMinf ();
        EMPTY_ATOM(dinf);
        movie_atom_t *drefAtom = drefAtomGen ();
        EMPTY_ATOM(stbl);
        movie_atom_t *stsdAtom = stsdAtomWithResolutionCodecSpsAndPps (myVideo->width, myVideo->height, myVideo->videoCodec, myVideo->sps, myVideo->spsSize, myVideo->pps, myVideo->ppsSize);
        movie_atom_t *sttsAtom = sttsAtomWithNumFrames (nbFrames);

        // Generate stss atom from iFramesIndexBuffer and nbIFrames
        uint32_t stssDataLen = (8 + (nbIFrames * sizeof (uint32_t)));
        uint8_t *stssBuffer = vp_os_calloc (stssDataLen, 1);
        uint32_t nenbIFrames = htonl (nbIFrames);
        memcpy (&stssBuffer[4], &nenbIFrames, sizeof (uint32_t));
        memcpy (&stssBuffer[8], iFrameIndexBuffer, nbIFrames * sizeof (uint32_t));
        movie_atom_t *stssAtom = atomFromData (stssDataLen, "stss", stssBuffer);
        vp_os_free (stssBuffer);
        stssBuffer = NULL;

        movie_atom_t *stscAtom = stscAtomGen ();

        // Generate stsz atom from frameSizeBufferLE and nbFrames
        uint32_t stszDataLen = (12 + (nbFrames * sizeof (uint32_t)));
        uint8_t *stszBuffer = vp_os_calloc (stszDataLen, 1);
        uint32_t nenbFrames = htonl (nbFrames);
        memcpy (&stszBuffer[8], &nenbFrames, sizeof (uint32_t));
        memcpy (&stszBuffer[12], frameSizeBufferNE, nbFrames * sizeof (uint32_t));
        movie_atom_t *stszAtom = atomFromData (stszDataLen, "stsz", stszBuffer);
        vp_os_free (stszBuffer);
        stszBuffer = NULL;

        // Generate stco atom from frameOffsetBuffer and nbFrames
        uint32_t stcoDataLen = (8 + (nbFrames * sizeof (uint32_t)));
        uint8_t *stcoBuffer = vp_os_calloc (stcoDataLen, 1);
        memcpy (&stcoBuffer[4], &nenbFrames, sizeof (uint32_t));
        memcpy (&stcoBuffer[8], frameOffsetBuffer, nbFrames * sizeof (uint32_t));
        movie_atom_t *stcoAtom = atomFromData (stcoDataLen, "stco", stcoBuffer);
        vp_os_free (stcoBuffer);
        stcoBuffer = NULL;

        EMPTY_ATOM (udta);
        movie_atom_t *swrAtom = metadataAtomFromTagAndValue ("swr", "AR.Drone 2.0");

        char dateInfoString[ENCAPSULER_SMALL_STRING_SIZE] = {0};
        snprintf (dateInfoString, ENCAPSULER_SMALL_STRING_SIZE, "%04d-%02d-%02dT%02d:%02d:%02d%+03d%02d",
                  nowTm->tm_year + 1900,
                  nowTm->tm_mon + 1,
                  nowTm->tm_mday,
                  nowTm->tm_hour,
                  nowTm->tm_min,
                  nowTm->tm_sec,
                  (int)(-timezone / 3600) + nowTm->tm_isdst,
                  (int)((-timezone % 3600) / 60));
        movie_atom_t *dayAtom = metadataAtomFromTagAndValue ("day", dateInfoString);

        char gpsInfoString[ENCAPSULER_SMALL_STRING_SIZE] = {0};
        int gpsIsValid = generateGpsString (gpsInfoString, ENCAPSULER_SMALL_STRING_SIZE);
        ENCAPSULER_DEBUG ("Valid : %d, Gps info string : %s\n", gpsIsValid, gpsInfoString);
        movie_atom_t *xyzAtom = NULL;

        /**
         * Android 4.0.3 and later don't support the (c)xyz atom in the videos
         * We won't generate it for any android versions
         */
#ifndef USE_ANDROID
        if (1 == gpsIsValid)
        {
            xyzAtom = metadataAtomFromTagAndValue ("xyz", gpsInfoString);
        }
#endif

        // Create atom tree
        insertAtomIntoAtom (udtaAtom, &swrAtom);
        insertAtomIntoAtom (udtaAtom, &dayAtom);
        if (NULL != xyzAtom)
        {
            insertAtomIntoAtom (udtaAtom, &xyzAtom);
        }

        insertAtomIntoAtom (stblAtom, &stsdAtom);
        insertAtomIntoAtom (stblAtom, &sttsAtom);
        insertAtomIntoAtom (stblAtom, &stssAtom);
        insertAtomIntoAtom (stblAtom, &stscAtom);
        insertAtomIntoAtom (stblAtom, &stszAtom);
        insertAtomIntoAtom (stblAtom, &stcoAtom);

        insertAtomIntoAtom (dinfAtom, &drefAtom);

        insertAtomIntoAtom (minfAtom, &vmhdAtom);
        insertAtomIntoAtom (minfAtom, &hdlr2Atom);
        insertAtomIntoAtom (minfAtom, &dinfAtom);
        insertAtomIntoAtom (minfAtom, &stblAtom);

        insertAtomIntoAtom (mdiaAtom, &mdhdAtom);
        insertAtomIntoAtom (mdiaAtom, &hdlrAtom);
        insertAtomIntoAtom (mdiaAtom, &minfAtom);

        insertAtomIntoAtom (trakAtom, &tkhdAtom);
        insertAtomIntoAtom (trakAtom, &mdiaAtom);

        insertAtomIntoAtom (moovAtom, &mvhdAtom);
        insertAtomIntoAtom (moovAtom, &trakAtom);
        insertAtomIntoAtom (moovAtom, &udtaAtom);

        if (-1 == writeAtomToFile (&moovAtom, myVideo->outFile))
        {
            ENCAPSULER_ERROR ("Error while writing moovAtom\n");
            localError = ARDRONE_VIDEO_FILE_ERROR;
        }
    }

    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {

        // Write atom to identify ardrone video with path (get directory contained video file)
        char *mediaPath = vp_os_calloc (ARDRONE_VIDEO_PATH_MAX_SIZE, sizeof (char));
        if (NULL != mediaPath)
        {
            strncpy(mediaPath, myVideo->outFilePath, ARDRONE_VIDEO_PATH_MAX_SIZE);

            char *lastPathComponent = strrchr(mediaPath, '/');
            char *currentPathComponent = NULL;

            if(lastPathComponent != NULL)
            {
                char *previousPathComponent = NULL;
                currentPathComponent = strchr(mediaPath, '/');

                while((currentPathComponent != NULL) && (currentPathComponent != lastPathComponent))
                {
                    previousPathComponent = currentPathComponent;
                    currentPathComponent = strchr(currentPathComponent + 1, '/');
                }

                currentPathComponent = previousPathComponent + 1;
            }

            movie_atom_t *ardtAtom = ardtAtomFromPathAndDroneVersion(currentPathComponent, myVideo->droneVersion);
            if (-1 == writeAtomToFile (&ardtAtom, myVideo->outFile))
            {
                ENCAPSULER_ERROR ("Unable to write ardt atom");
                localError = ARDRONE_VIDEO_FILE_ERROR;
            }
            vp_os_free (mediaPath);
            mediaPath = NULL;
        }
        else
        {
            localError = ARDRONE_VIDEO_GENERIC_ERROR;
        }
    }

    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {
        movie_atom_t *mdatAtom = mdatAtomForFormatWithVideoSize (myVideo->videoType, dataTotalSize+8);
        // write mdat atom
        fseek (myVideo->outFile, myVideo->mdatAtomOffset, SEEK_SET);
        if (-1 == writeAtomToFile (&mdatAtom, myVideo->outFile))
        {
            ENCAPSULER_ERROR ("Error while writing mdatAtom\n");
            localError = ARDRONE_VIDEO_FILE_ERROR;
        }
    }

    if (ARDRONE_VIDEO_NO_ERROR == localError)
    {
        fclose (myVideo->outFile);
        fclose (myVideo->infoFile);

        remove (myVideo->infoFilePath);
        rename (myVideo->tempFilePath, myVideo->outFilePath);


        if (myVideo->sps)
        {
            vp_os_free (myVideo->sps);
            myVideo->sps = NULL;
        }
        if (myVideo->pps)
        {
            vp_os_free (myVideo->pps);
            myVideo->pps = NULL;
        }

        vp_os_mutex_unlock(&myVideo->mutex);
        vp_os_free (myVideo);
        myVideo = NULL;
        *video = NULL;

        vp_os_free (frameSizeBuffer);
        frameSizeBuffer = NULL;
        vp_os_free (frameSizeBufferNE);
        frameSizeBufferNE = NULL;
        vp_os_free (frameOffsetBuffer);
        frameOffsetBuffer = NULL;
        vp_os_free (iFrameIndexBuffer);
        iFrameIndexBuffer = NULL;
        vp_os_free (frameIsIFrame);
        frameIsIFrame = NULL;
    }
    else
    {
        vp_os_free (frameSizeBuffer);
        frameSizeBuffer = NULL;
        vp_os_free (frameSizeBufferNE);
        frameSizeBufferNE = NULL;
        vp_os_free (frameOffsetBuffer);
        frameOffsetBuffer = NULL;
        vp_os_free (iFrameIndexBuffer);
        iFrameIndexBuffer = NULL;
        vp_os_free (frameIsIFrame);
        frameIsIFrame = NULL;
        vp_os_mutex_unlock(&myVideo->mutex);
        ardrone_video_cleanup (video);
    }
    return localError;
}

ardrone_video_error_t ardrone_video_cleanup (ardrone_video_t **video)
{
    if (NULL == (*video))
    {
        ENCAPSULER_ERROR ("video pointer must not be null");
        return ARDRONE_VIDEO_BAD_ARGS;
    }
    vp_os_mutex_lock(&((*video)->mutex));
    ardrone_video_t *myVideo = (*video); // ease of reading

    fclose (myVideo->outFile);
    fclose (myVideo->infoFile);
    remove (myVideo->infoFilePath);
    remove (myVideo->tempFilePath);
    remove (myVideo->outFilePath);

    if (myVideo->sps)
    {
        vp_os_free (myVideo->sps);
        myVideo->sps = NULL;
    }
    if (myVideo->pps)
    {
        vp_os_free (myVideo->pps);
        myVideo->pps = NULL;
    }

    vp_os_mutex_unlock(&myVideo->mutex);
    vp_os_free (myVideo);
    *video = NULL;

    return ARDRONE_VIDEO_NO_ERROR;
}

#ifndef _WIN32
int ftwCallback (const char *fpath, const struct stat *sb, int typeflag, struct FTW *ftwbuf)
{
    // ENCAPSULER_DEBUG ("Processing file %s\n", fpath);
#if defined(TARGET_OS_IPHONE) || defined (TARGET_OS_IPHONE_SIMULATOR)
    if (FTW_F == typeflag) // Check only plain files
    {
        char *infoFile = strstr (fpath, INFOFILE_EXT);
        if (NULL != infoFile)
        {
            printf ("Will try to fix video at path %s : ", fpath);
            if (TRUE == ardrone_video_try_fix (fpath))
            {
                printf ("[OK]\n");
            }
            else
            {
                printf ("[FAIL]\n");
            }
        }
    }
    return 0;
#else
    int retVal = FTW_CONTINUE;
    if (FTW_D == typeflag) // Check if directory is "media_*" to continue
    {
        // Two first test lines detect the "." directory
        //  -> length of 1 character, starting with '.'
        // Third line detect any media_ folder directly under the "." folder
        if ( ( ( 1  == strlen (fpath) )   &&
               ('.' == fpath [0]      ) ) ||
             (   0  == strncmp (fpath, "./media_", 8) ) )
        {
            // We're processing "." dir or any ./media_xxx dir, continue
            retVal = FTW_CONTINUE;
        }
        else
        {
            // We're processing any other dir : don't recurse into it
            retVal = FTW_SKIP_SUBTREE;
        }
    }
    else if (FTW_F == typeflag)
    {
        char *infoFile = strstr (fpath, INFOFILE_EXT);
        if (NULL != infoFile)
        {
            printf ("Will try to fix video at path %s : ", fpath);
            if (TRUE == ardrone_video_try_fix (fpath))
            {
                printf ("[OK]\n");
            }
            else
            {
                printf ("[FAIL]\n");
            }
        }
    }
    return retVal;
#endif
}
#endif

ardrone_video_error_t ardrone_video_remove_all (const char *rootDir)
{
#ifdef _WIN32
    WIN32_FIND_DATA f;
    char *dirWithStar = vp_os_malloc (strlen (dir) + 3);
    if (NULL == dirWithStar)
    {
        ENCAPSULER_ERROR ("Unable to allocate string");
        return ARDRONE_VIDEO_GENERIC_ERROR;
    }
    snprintf (dirWithStar, strlen (dir) + 3, "%s/*", dir);
    HANDLE h = FindFirstFile(dirWithStar, &f);
    if (h != INVALID_HANDLE_VALUE)
    {
        do
        {
            char *tmpFile = strstr (f.cFileName, TEMPFILE_EXT);
            char *infoFile = strstr (f.cFileName, INFOFILE_EXT);
            if (NULL != tmpFile || NULL != infoFile)
            {
                int string_size = strlen (dir) + strlen (f.cFileName) + 2;
                char *completeFileName = vp_os_malloc (string_size);
                if (NULL == completeFileName)
                {
                    ENCAPSULER_ERROR ("Unable to allocate string");
                    vp_os_free (dirWithStar);
                    return ARDRONE_VIDEO_GENERIC_ERROR;
                }
                snprintf (completeFileName, string_size, "%s/%s", dir, f.cFileName);
                remove (completeFileName);
                vp_os_free (completeFileName);
            }
        } while(FindNextFile(h, &f));
    }
    else
    {
        ENCAPSULER_ERROR ("Unable to open directory");
        vp_os_free (dirWithStar);
        return ARDRONE_VIDEO_FILE_ERROR;
    }
    vp_os_free (dirWithStar);
#else
#define MAX_FD_FOR_NFTW (20)
    ardrone_video_error_t retError = ARDRONE_VIDEO_NO_ERROR;
#if defined (TARGET_OS_IPHONE) || defined (TARGET_OS_IPHONE_SIMULATOR)
    int NFTW_Flags = 0;
#else
    int NFTW_Flags = FTW_ACTIONRETVAL;
#endif
    if (0 != nftw (rootDir, ftwCallback, MAX_FD_FOR_NFTW, NFTW_Flags))
    {
        retError = ARDRONE_VIDEO_GENERIC_ERROR;
    }
    return retError;
#undef MAX_FD_FOR_NFTW
#endif
}

void ardrone_video_error_string (ardrone_video_error_t error, char *buf, uint32_t bufLen)
{
    switch (error)
    {
    case ARDRONE_VIDEO_NO_ERROR:
        snprintf (buf, bufLen, "No error");
        break;
    case ARDRONE_VIDEO_GENERIC_ERROR:
        snprintf (buf, bufLen, "Generic error -e.g. malloc error-");
        break;
    case ARDRONE_VIDEO_BAD_CODEC:
        snprintf (buf, bufLen, "Wrong video codec");
        break;
    case ARDRONE_VIDEO_FILE_ERROR:
        snprintf (buf, bufLen, "I/O error on file");
        break;
    case ARDRONE_VIDEO_WAITING_FOR_IFRAME:
        snprintf (buf, bufLen, "Waiting for IFrame to start recording");
        break;
    case ARDRONE_VIDEO_BAD_ARGS:
        snprintf (buf, bufLen, "Bad arguments");
        break;
    default:
        snprintf (buf, bufLen, "Unknown error");
        break;
    }
}

void ardrone_video_set_gps_infos (double latitude, double longitude, double altitude)
{
    videoGpsInfos.latitude = latitude;
    videoGpsInfos.longitude = longitude;
    videoGpsInfos.altitude = altitude;
}

bool_t ardrone_video_try_fix (const char *infoFilePath)
{
    // Local values
    ardrone_video_t *video = NULL;
    FILE *infoFile = NULL;
    bool_t noError = TRUE;
    bool_t finishNoError = TRUE;
    uint32_t frameNumber = 0;

    // Alloc local video pointer
    if (TRUE == noError)
    {
        video = vp_os_calloc (1, sizeof (ardrone_video_t));
        if (NULL == video)
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Unable to alloc video pointer");
        } // No else
    } // No else

    // Open file for reading
    if (TRUE == noError)
    {
        infoFile = fopen (infoFilePath, "r+b");
        if (NULL == infoFile)
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Unable to open infoFile");
        } // No else
    } // No else

    // Read size from info descriptor
    if (TRUE == noError)
    {
        uint32_t descriptorSize = 0;
        if (1 != fread (&descriptorSize, sizeof (uint32_t), 1, infoFile))
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Unable to read descriptorSize");
        }
        else if (descriptorSize < sizeof (ardrone_video_t))
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Descriptor size (%d) is smaller than ardrone_video_t size (%d)", descriptorSize, sizeof (ardrone_video_t));
        } // No else
    } // No else

    // Read video
    if (TRUE == noError)
    {
        if (1 != fread (video, sizeof (ardrone_video_t), 1, infoFile))
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Unable to read ardrone_video_t from infoFile");
        }
        else
        {
            video->infoFile = infoFile;
            video->sps = NULL;
            video->pps = NULL;
            video->outFile = NULL;
            infoFile = NULL;
        }
    } // No else

    if (TRUE == noError)
    {
        if (ARDRONE_VIDEO_VERSION_NUMBER != video->version)
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Bad ardrone_video_t version number");
        } // No else, version number is OK
    }

    // Allocate / copy SPS/PPS pointers
    if (TRUE == noError && 0 != video->spsSize && 0 != video->ppsSize)
    {
        video->sps = vp_os_malloc (video->spsSize);
        video->pps = vp_os_malloc (video->ppsSize);
        if (NULL == video->sps ||
            NULL == video->pps)
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Unable to allocate video sps/pps");
            vp_os_free (video->sps);
            vp_os_free (video->pps);
            video->sps = NULL;
            video->pps = NULL;
        }
        else
        {
            if (1 != fread (video->sps, video->spsSize, 1, video->infoFile))
            {
                ENCAPSULER_DEBUG ("Unable to read video SPS");
                noError = FALSE;
            }
            else if (1 != fread (video->pps, video->ppsSize, 1, video->infoFile))
            {
                ENCAPSULER_DEBUG ("Unable to read video PPS");
                noError = FALSE;
            } // No else
        }
    }
    else
    {
        ENCAPSULER_DEBUG ("Video SPS/PPS sizes are bad");
        noError = FALSE;
    }

    // Open temp file
    if (TRUE == noError)
    {
        video->outFile = fopen (video->tempFilePath, "r+b");
        if (NULL == video->outFile)
        {
            noError = FALSE;
            ENCAPSULER_DEBUG ("Unable to open outFile : %s", video->tempFilePath);
        } // No else
    } // No else

    // Count frames
    uint32_t dataSize = 0;
    size_t tmpvidSize = 0;
    if (TRUE == noError)
    {
        bool_t endOfSearch = FALSE;
        uint32_t frameSize = 0;
        char fType = 'a';
        fseek (video->outFile, 0, SEEK_END);
        tmpvidSize = ftell (video->outFile);
        size_t prevInfoIndex = 0;
        while (FALSE == endOfSearch)
        {
            prevInfoIndex = ftell (video->infoFile);
            if (! feof (video->infoFile) &&
                ARDRONE_VIDEO_NUM_MATCH_PATTERN ==
                fscanf (video->infoFile, ARDRONE_VIDEO_INFO_PATTERN, &frameSize, &fType))
            {
                if ((dataSize + video->framesDataOffset + frameSize) > tmpvidSize)
                {
                    // We have too many infos : truncate at prevInfoIndex
                    fseek (video->infoFile, 0, SEEK_SET);
                    if (0 != ftruncate (fileno (video->infoFile), prevInfoIndex))
                    {
                        ENCAPSULER_DEBUG ("Unable to truncate infoFile");
                        noError = FALSE;
                    } // No else
                    endOfSearch = TRUE;
                }
                else
                {
                    dataSize += frameSize;
                    frameNumber ++;
                }
            }
            else
            {
                endOfSearch = TRUE;
            }
        }
    } // No else

    if (TRUE == noError)
    {
        // If needed remove unused frames from .tmpvid file
        dataSize += video->framesDataOffset;
        if (tmpvidSize > dataSize)
        {
            if (0 != ftruncate (fileno (video->outFile), dataSize))
            {
                ENCAPSULER_DEBUG ("Unable to truncate outFile");
                noError = FALSE;
            } // No else
        } // No else

        fseek (video->outFile, 0, SEEK_END);
    }

    if (TRUE == noError)
    {
        video->framesCount = frameNumber;
        vp_os_mutex_init (&video->mutex);
        rewind (video->infoFile);
        if (ARDRONE_VIDEO_NO_ERROR != ardrone_video_finish (&video))
        {
            ENCAPSULER_DEBUG ("Unable to finish video");
            /* We don't use the "noError" variable here as the
             * ardrone_video_finish function will do the cleanup
             * even if it fails */
            finishNoError = FALSE;
        }
    }

    if (FALSE == noError)
    {
        if (NULL != video)
        {
            ENCAPSULER_DEBUG ("Freeing local copies");
            ENCAPSULER_CLEANUP (vp_os_free, video->sps);
            video->sps = NULL;
            ENCAPSULER_CLEANUP (vp_os_free, video->pps);
            video->pps = NULL;
            ENCAPSULER_CLEANUP (fclose, video->infoFile);
            ENCAPSULER_CLEANUP (fclose, video->outFile);
            ENCAPSULER_CLEANUP (vp_os_free ,video);
            video = NULL;
        }

        if (NULL != infoFile)
        {
            ENCAPSULER_DEBUG ("Closing local file");
            ENCAPSULER_CLEANUP (fclose, infoFile);
        }

        ENCAPSULER_DEBUG ("Cleaning up files");
        ENCAPSULER_DEBUG ("removing %s", infoFilePath);
        remove (infoFilePath);
        uint32_t pathLen = strlen (infoFilePath);
        char *tmpVidFilePath = vp_os_malloc (pathLen + 1);
        if (NULL != tmpVidFilePath)
        {
            strncpy (tmpVidFilePath, infoFilePath, pathLen);
            tmpVidFilePath[pathLen] = 0;
            strncpy (&tmpVidFilePath[pathLen-7], "tmpvid", 7);
            ENCAPSULER_DEBUG ("removing %s", tmpVidFilePath);
            remove (tmpVidFilePath);
            ENCAPSULER_CLEANUP (vp_os_free, tmpVidFilePath);
        }
        else
        {
            ENCAPSULER_DEBUG ("Unable to allocate filename buffer, cleanup was not complete");
        }
    }

    return noError && finishNoError;
}
