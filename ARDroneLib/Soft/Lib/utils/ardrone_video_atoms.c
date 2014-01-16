/*
 * ardrone_video_atoms.c
 * ARDroneLib
 *
 * Created by n.brulez on 19/08/11
 * Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#include "ardrone_video_atoms.h"
#include <string.h>
#ifdef _WIN32
#include <Winsock2.h>
#else
#include <arpa/inet.h>
#endif

#define USE_VP_OS (1)

#if USE_VP_OS
#include <VP_Os/vp_os_malloc.h>
#define ATOM_MALLOC(SIZE) vp_os_malloc(SIZE)
#define ATOM_REALLOC(PTR, SIZE) vp_os_realloc(PTR, SIZE)
#define ATOM_FREE(PTR) vp_os_free(PTR)
#define ATOM_MEMCOPY(DST, SRC, SIZE) vp_os_memcpy(DST, SRC, SIZE)
#else
#include <stdlib.h>
#define ATOM_MALLOC(SIZE) malloc(SIZE)
#define ATOM_REALLOC(PTR, SIZE) realloc(PTR, SIZE)
#define ATOM_FREE(PTR) free(PTR)
#define ATOM_MEMCOPY(DST, SRC, SIZE) memcpy(DST, SRC, SIZE)
#endif

#define TIMESTAMP_FROM_1970_TO_1904 (0x7c25b080U)

/**
 * To use these macros, function must ensure :
 *  - currentIndex is declared and set to zero BEFORE first call
 *  - data is a valid pointer to the output array of datas we want to write
 * These macros don't include ANY size check, so function have to ensure that the data buffer have enough left space
 */

// Write a 4CC into atom data
//  Usage : ATOM_WRITE_4CC ('c', 'o', 'd', 'e');
#define ATOM_WRITE_4CC(_A, _B, _C, _D)          \
    do                                          \
    {                                           \
        data[currentIndex++] = _A;              \
        data[currentIndex++] = _B;              \
        data[currentIndex++] = _C;              \
        data[currentIndex++] = _D;              \
    } while (0)
// Write a 8 bit word into atom data
#define ATOM_WRITE_U8(VAL)                      \
    do                                          \
    {                                           \
        data[currentIndex] = (uint8_t)VAL;      \
        currentIndex++;                         \
    } while (0)
// Write a 16 bit word into atom data
#define ATOM_WRITE_U16(VAL)                                     \
    do                                                          \
    {                                                           \
        uint16_t *DPOINTER = (uint16_t *)&(data[currentIndex]); \
        currentIndex += 2;                                      \
        *DPOINTER = htons((uint16_t)VAL);                       \
    } while (0)
// Write a 32 bit word into atom data
#define ATOM_WRITE_U32(VAL)                                     \
    do                                                          \
    {                                                           \
        uint32_t *DPOINTER = (uint32_t *)&(data[currentIndex]); \
        currentIndex += 4;                                      \
        *DPOINTER = htonl((uint32_t)VAL);                       \
    } while (0)
// Write an defined number of bytes into atom data
//  Usage : ATOM_WRITE_BYTES (pointerToData, sizeToCopy)
#define ATOM_WRITE_BYTES(POINTER, SIZE)                     \
    do                                                      \
    {                                                       \
        ATOM_MEMCOPY (&data[currentIndex], POINTER, SIZE);  \
        currentIndex += SIZE;                               \
    } while (0)


movie_atom_t *atomFromData (uint32_t data_size, const char *_tag, const uint8_t *_data)
{
  movie_atom_t *retAtom = ATOM_MALLOC (sizeof (movie_atom_t));
  if (NULL == retAtom)
    {
      return retAtom;
    }
  retAtom->size = data_size + 8;
  strncpy (retAtom->tag, _tag, 4);

  retAtom->data = NULL;
  if (NULL != _data && data_size > 0)
    {
      retAtom->data = ATOM_MALLOC (data_size);
      if (NULL == retAtom->data)
        {
          ATOM_FREE (retAtom);
          retAtom = NULL;
          return NULL;
        }
      ATOM_MEMCOPY (retAtom->data, _data, data_size);
    }
  retAtom->wide = 0;
  return retAtom;
}

void insertAtomIntoAtom (movie_atom_t *container, movie_atom_t **leaf)
{
  uint32_t new_container_size = container->size - 8 + (*leaf)->size;
  container->data = ATOM_REALLOC (container->data, new_container_size);
  if (NULL == container->data)
    {
      fprintf (stderr, "Alloc error for atom insertion\n");
      return;
    }
  uint32_t leafSizeNetworkEndian = htonl ((*leaf)->size);
  ATOM_MEMCOPY (&container->data[container->size - 8], &leafSizeNetworkEndian, sizeof (uint32_t));
  ATOM_MEMCOPY (&container->data[container->size - 4], (*leaf)->tag, 4);
  if (NULL != (*leaf)->data)
    {
      ATOM_MEMCOPY (&container->data[container->size], (*leaf)->data, ((*leaf)->size - 8));
      container->size = new_container_size + 8;
    }
  ATOM_FREE ((*leaf)->data);
  (*leaf)->data = NULL;
  ATOM_FREE (*leaf);
  *leaf = NULL;
}

int writeAtomToFile (movie_atom_t **_atom, FILE *file)
{
    if (NULL == *_atom)
    {
        return -1;
    }
  uint32_t networkEndianSize = htonl ((*_atom)->size);
  if (4 != fwrite (&networkEndianSize, 1, 4, file))
    {
      return -1;
    }
  if (4 != fwrite ((*_atom)->tag, 1, 4, file))
    {
      return -1;
    }
  if (NULL != (*_atom)->data)
    {
      uint32_t atom_data_size = 0;
      if (1 == (*_atom)->wide)
        {
          atom_data_size = 8;
        }
      else
        {
          atom_data_size = (*_atom)->size - 8;
        }
      if (atom_data_size != fwrite ((*_atom)->data, 1, atom_data_size, file))
        {
          return -1;
        }
    }

  ATOM_FREE ((*_atom)->data);
  (*_atom)->data = NULL;
  ATOM_FREE (*_atom);
  *_atom = NULL;
 
  return 0;
}

void freeAtom (movie_atom_t **_atom)
{
    if ((NULL != _atom) &&
        (NULL != *_atom))
    {
        if (NULL != (*_atom)->data)
        {
            ATOM_FREE ((*_atom)->data);
        }
        ATOM_FREE (*_atom);
        *_atom = NULL;
    }
}

movie_atom_t *ftypAtomForFormatAndCodecWithOffset (ardrone_video_type_t format, parrot_video_encapsulation_codecs_t codec, uint32_t *offset, movie_atom_t **wideAtom)
{
  if (NULL == offset || NULL == wideAtom)
    {
      return NULL;
    }
  movie_atom_t *retAtom = NULL;
  if (ARDRONE_VIDEO_MP4 == format)
        {
        uint8_t data [24] = {'i', 's', 'o', 'm',
                               0x00, 0x00, 0x02, 0x00,
                             'i', 's', 'o', 'm',
                             'i', 's', 'o', '2',
                             'a', 'v', 'c', '1',
                               'm', 'p', '4', '1'};
          retAtom = atomFromData (24, "ftyp", data);
          *offset = 48;
        }
  else if (ARDRONE_VIDEO_MOV == format)
    {
        uint8_t data [12] = {'q', 't', ' ', ' ',
                           0x00, 0x00, 0x02, 0x00,
                           'q', 't', ' ', ' '};
      retAtom = atomFromData (12, "ftyp", data);
      uint8_t wideData [4] = {0};
      *wideAtom = atomFromData (4, "wide", wideData);
      *offset = 48;
    }
  return retAtom;
}

movie_atom_t *mdatAtomForFormatWithVideoSize (ardrone_video_type_t format, uint64_t videoSize)
{
  movie_atom_t *retAtom = NULL;
  if (videoSize <= INT32_MAX)
    {
      // Free/wide atom + mdat
      uint8_t data [8] = {0x00, 0x00, 0x00, 0x00,
                          'm', 'd', 'a', 't'};
      uint32_t sizeNe = htonl ((uint32_t) videoSize);
      ATOM_MEMCOPY (data, &sizeNe, sizeof (uint32_t));
      if (ARDRONE_VIDEO_MP4 == format)
        {
          retAtom = atomFromData (8, "free", data);
          retAtom->size = 8;
          retAtom->wide = 1;
        }
      else
        {
          retAtom = atomFromData (8, "wide", data);
          retAtom->size = 8;
          retAtom->wide = 1;
        }
    }
  else
    {
      // 64bit wide mdat atom
      uint8_t data [8] = {0};
      uint32_t highSize = (videoSize >> 32);
        uint32_t lowSize = (videoSize & 0xffffffff);
      uint32_t highSizeNe = htonl (highSize);
      uint32_t lowSizeNe = htonl (lowSize);
      ATOM_MEMCOPY (data, &highSizeNe, sizeof (uint32_t));
      ATOM_MEMCOPY (&data[4], &lowSizeNe, sizeof (uint32_t));
      retAtom = atomFromData (8, "mdat", data);
      retAtom->size = 0;
      retAtom->wide = 1;
    }
  return retAtom;
}

movie_atom_t *mvhdAtomFromFpsNumFramesAndDate (uint32_t fps, uint32_t nbFrames, time_t date)
{
    uint32_t dataSize = 100;
    uint8_t *data = ATOM_MALLOC (dataSize);
    if (NULL == data)
    {
        return NULL;
    }
    uint32_t currentIndex = 0;
    ATOM_WRITE_U32 (0); /* Version (8) + Flags (24) */
    ATOM_WRITE_U32 ((uint32_t)date + TIMESTAMP_FROM_1970_TO_1904); /* Creation time */
    ATOM_WRITE_U32 ((uint32_t)date + TIMESTAMP_FROM_1970_TO_1904); /* Modification time */
    ATOM_WRITE_U32 (1000); /* Timescale */
    ATOM_WRITE_U32 (1000u * nbFrames / fps); /* Duration (in timescale units) */

    ATOM_WRITE_U32 (0x00010000); /* Reserved */
    ATOM_WRITE_U16 (0x0100); /* Reserved */
    ATOM_WRITE_U16 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */

    ATOM_WRITE_U32 (0x00010000); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0x00010000); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0x40000000); /* Reserved */

    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (2); /* Next track id */

    movie_atom_t *retAtom = atomFromData (100, "mvhd", data);
    ATOM_FREE (data);
    data = NULL;
    return retAtom;
}

movie_atom_t *tkhdAtomWithResolutionNumFramesFpsAndDate (uint32_t w, uint32_t h, uint32_t nbFrames, uint32_t fps, time_t date)
{
    uint32_t dataSize = 84;
    uint8_t *data = ATOM_MALLOC (dataSize);
    if (NULL == data)
    {
        return NULL;
    }
    uint32_t currentIndex = 0;

    ATOM_WRITE_U32 (0x0000000f); /* Version (8) + Flags (24) */
    ATOM_WRITE_U32 ((uint32_t)date + TIMESTAMP_FROM_1970_TO_1904); /* Creation time */
    ATOM_WRITE_U32 ((uint32_t)date + TIMESTAMP_FROM_1970_TO_1904); /* Modification time */
    ATOM_WRITE_U32 (1); /* Track ID */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (1000u * nbFrames / fps); /* Duration, in timescale unit */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */

    ATOM_WRITE_U32 (0x00010000); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0x00010000); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0); /* Reserved */
    ATOM_WRITE_U32 (0x40000000); /* Reserved */

    ATOM_WRITE_U16 (w); /* Width */
    ATOM_WRITE_U16 (0); /* Reserved */
    ATOM_WRITE_U16 (h); /* Height*/
    ATOM_WRITE_U16 (0); /* Reserved */

    movie_atom_t *retAtom = atomFromData (84, "tkhd", data);
    ATOM_FREE (data);
    data = NULL;
    return retAtom;
}

movie_atom_t *mdhdAtomFromFpsNumFramesAndDate (uint32_t fps, uint32_t nbFrames, time_t date)
{
    uint32_t dataSize = 24;
    uint8_t *data = ATOM_MALLOC (dataSize);
    if (NULL == data)
    {
        return NULL;
    }
    uint32_t currentIndex = 0;

    ATOM_WRITE_U32 (0); /* Version (8) + Flags (24) */
    ATOM_WRITE_U32 ((uint32_t)date + TIMESTAMP_FROM_1970_TO_1904); /* Creation time */
    ATOM_WRITE_U32 ((uint32_t)date + TIMESTAMP_FROM_1970_TO_1904); /* Modification time */
    ATOM_WRITE_U32 (fps); /* Frames per second */
    ATOM_WRITE_U32 (nbFrames); /* Number of Frames */
    ATOM_WRITE_U32 (0); /* Reserved */

    movie_atom_t *retAtom = atomFromData (24, "mdhd", data);
    ATOM_FREE (data);
    data = NULL;
    return retAtom;
}


movie_atom_t *hdlrAtomForMdia ()
{
  uint8_t data [37] =  {0x00, 0x00, 0x00, 0x00,
                        'm', 'h', 'l', 'r',
                        'v', 'i', 'd', 'e',
                        0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00,
                        0x0c, 'V', 'i', 'd',
                        'e', 'o', 'H', 'a',
                        'n', 'd', 'l', 'e',
                        'r'};
  return atomFromData (37, "hdlr", data);
}

movie_atom_t *vmhdAtomGen ()
{
  uint8_t data [12] = {0x00, 0x00, 0x00, 0x01,
                       0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00};
  return atomFromData (12, "vmhd", data);
}

movie_atom_t *hdlrAtomForMinf ()
{
  uint8_t data [36] =  {0x00, 0x00, 0x00, 0x00,
                        'd', 'h', 'l', 'r',
                        'u', 'r', 'l', ' ',
                        0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00,
                        0x0b, 'D', 'a', 't',
                        'a', 'H', 'a', 'n',
                        'd', 'l', 'e', 'r'};
  return atomFromData (36, "hdlr", data);
}

movie_atom_t *drefAtomGen ()
{
  uint8_t data [20] = {0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x01,
                       0x00, 0x00, 0x00, 0x0C,
                       0x75, 0x72, 0x6c, 0x20,
                       0x00, 0x00, 0x00, 0x01};
  return atomFromData (20, "dref", data);
}

movie_atom_t *stsdAtomWithResolutionAndCodec (uint32_t w, uint32_t h, parrot_video_encapsulation_codecs_t codec)
{
  if (CODEC_MPEG4_AVC == codec)
    {
      uint8_t data [128] = {0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x78,
                            'a', 'v', 'c', '1', // vse type
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            'A', 'R', '.', 'D', // Muxer string
                            0x00, 0x00, 0x02, 0x00,
                            0x00, 0x00, 0x02, 0x00,
                            0x00, 0x00, 0x00, 0x00,// 40 -> width | 42 -> height
                            0x00, 0x48, 0x00, 0x00,
                            0x00, 0x48, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x01, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x18,
                            0xff, 0xff, 0x00, 0x00,
                            0x00, 0x22, 0x61, 0x76,
                            0x63, 0x43, 0x01, 0x42,
                            0x80, 0x1e, 0xff, 0xe1,
                            0x00, 0x09, 0x67, 0x42,
                            0x80, 0x1e, 0x8b, 0x68,
                            0x0a, 0x02, 0xf1, 0x01,
                            0x00, 0x06, 0x68, 0xce,
                            0x01, 0xa8, 0x77, 0x20};
      uint16_t neW = htons ((uint16_t) w);
      uint16_t neH = htons ((uint16_t) h);
      ATOM_MEMCOPY(&data[40], &neW, sizeof (uint16_t));
      ATOM_MEMCOPY(&data[42], &neH, sizeof (uint16_t));
      return atomFromData (128, "stsd", data);
    }
  else if (CODEC_MPEG4_VISUAL == codec)
    {
      uint8_t data [187] = {0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0xb3,
                            'm', 'p', '4', 'v', // vse type
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            'A', 'R', '.', 'D', // Muxer string
                            0x00, 0x00, 0x02, 0x00,
                            0x00, 0x00, 0x02, 0x00,
                            0x00, 0x00, 0x00, 0x00,// 40 -> width | 42 -> height
                            0x00, 0x48, 0x00, 0x00,
                            0x00, 0x48, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x01, 0x05, 'm',
                            'p', 'e', 'g', '4',
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x18,
                            0xff, 0xff, 0x00, 0x00,
                            0x00, 0x5d, 0x65, 0x73,
                            0x64, 0x73, 0x00, 0x00,
                            0x00, 0x00, 0x03, 0x80,
                            0x80, 0x80, 0x4c, 0x00,
                            0x01, 0x00, 0x04, 0x80,
                            0x80, 0x80, 0x3e, 0x20,
                            0x11, 0x00, 0x00, 0x00,
                            0x00, 0x07, 0xac, 0xda,
                            0x00, 0x07, 0xac, 0xda,
                            0x05, 0x80, 0x80, 0x80,
                            0x2c, 0x00, 0x00, 0x01,
                            0xb0, 0x01, 0x00, 0x00,
                            0x01, 0xb5, 0x89, 0x13,
                            0x00, 0x00, 0x01, 0x00,
                            0x00, 0x00, 0x01, 0x20,
                            0x00, 0xc4, 0x8d, 0x88,
                            0x00, 0xf5, 0x14, 0x04,
                            0x2e, 0x14, 0x63, 0x00,
                            0x00, 0x01, 0xb2, 'A',
                            'R', '.', 'D', 'r',
                            'o', 'n', 'e', '_',
                            '2', 0x06, 0x80, 0x80,
                            0x80, 0x01, 0x02};
      uint16_t neW = htons ((uint16_t) w);
      uint16_t neH = htons ((uint16_t) h);
      ATOM_MEMCOPY(&data[40], &neW, sizeof (uint16_t));
      ATOM_MEMCOPY(&data[42], &neH, sizeof (uint16_t));
      return atomFromData (187, "stsd", data);
    }
  else
    {
      return atomFromData (0, "stsd", NULL);
    }
}

movie_atom_t *stsdAtomWithResolutionCodecSpsAndPps (uint32_t w, uint32_t h, parrot_video_encapsulation_codecs_t codec, uint8_t *sps, uint32_t spsSize, uint8_t *pps, uint32_t ppsSize)
{
  if (NULL == sps || NULL == pps || CODEC_MPEG4_AVC != codec)
    {
      return stsdAtomWithResolutionAndCodec (w, h, codec);
    }
  uint32_t avcCSize = 19 + spsSize + ppsSize;
  uint32_t vse_size = avcCSize + 86;
  uint32_t dataSize = vse_size + 8;
  uint8_t *data = ATOM_MALLOC (dataSize);
  if (NULL == data)
    {
      return NULL;
    }
  uint32_t currentIndex = 0;
  ATOM_WRITE_U32 (0); /* version / flags */
  ATOM_WRITE_U32 (1); /* entry count */
  ATOM_WRITE_U32 (vse_size); /* video sample entry size */
  ATOM_WRITE_4CC ('a', 'v', 'c', '1'); /* VSE type */
  ATOM_WRITE_U32 (0); /* Reserved */
  ATOM_WRITE_U16 (0); /* Reserved */
  ATOM_WRITE_U16 (1); /* Data reference index */
  ATOM_WRITE_U16 (0); /* Codec stream version */
  ATOM_WRITE_U16 (0); /* Codec stream revision */
  ATOM_WRITE_4CC ('A', 'R', '.', 'D'); /* Muxer name */
  ATOM_WRITE_U32 (0x200); /* Temporal quality */
  ATOM_WRITE_U32 (0x200); /* Spatial quality */
  ATOM_WRITE_U16 (w); /* Width */
  ATOM_WRITE_U16 (h); /* Height */
  ATOM_WRITE_U32 (0x00480000); /* Horiz DPI : 72dpi */
  ATOM_WRITE_U32 (0x00480000); /* Vert DPI : 72dpi */
  ATOM_WRITE_U32 (0); /* Data size */
  ATOM_WRITE_U16 (1); /* Frame count */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U32 (0); /* Compressor name ... 32 octets of zeros */
  ATOM_WRITE_U16 (0x18); /* Reserved */
  ATOM_WRITE_U16 (0xffff); /* Reserved */
  /* avcC tag */
  ATOM_WRITE_U32 (avcCSize); /* Size */
  ATOM_WRITE_4CC ('a', 'v', 'c', 'C'); /* avcC header */
  ATOM_WRITE_U8 (1); /* version */
  ATOM_WRITE_U8 (sps[1]); /* profile */
  ATOM_WRITE_U8 (sps[2]); /* profile compat */
  ATOM_WRITE_U8 (sps[3]); /* level */
  ATOM_WRITE_U8 (0xff); /* Reserved (6bits) -- NAL size length - 1 (2bits) */
  ATOM_WRITE_U8 (0xe1); /* Reserved (3 bits) -- Number of SPS (5 bits) */
  ATOM_WRITE_U16 (spsSize); /* Size of SPS */
  ATOM_WRITE_BYTES (sps, spsSize); /* SPS header */
  ATOM_WRITE_U8 (1); /* Number of PPS */
  ATOM_WRITE_U16 (ppsSize); /* Size of PPS */
  ATOM_WRITE_BYTES (pps, ppsSize); /* PPS Header */

  movie_atom_t *retAtom = atomFromData (dataSize, "stsd", data);
  ATOM_FREE (data);
  data = NULL;
  return retAtom;
}

movie_atom_t *sttsAtomWithNumFrames (uint32_t nbFrames)
{
  uint8_t data [16] = {0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x01,
                       0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x01};
  uint32_t networkEndianFrames = htonl (nbFrames);
  ATOM_MEMCOPY( &data[8], &networkEndianFrames, sizeof (uint32_t));
  return atomFromData (16, "stts", data);
}

movie_atom_t *stscAtomGen ()
{
  uint8_t data [20] = {0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x01,
                       0x00, 0x00, 0x00, 0x01,
                       0x00, 0x00, 0x00, 0x01,
                       0x00, 0x00, 0x00, 0x01};
  return atomFromData (20, "stsc", data);
}

movie_atom_t *metadataAtomFromTagAndValue (const char *tag, const char *value)
{
  movie_atom_t *retAtom = NULL;
  char locTag [4] = {0};
  /* If tag have a length of 3 chars, the (c) sign is added by this function */
  if (3 == strlen (tag))
    {
      locTag[0] = '\251'; // (c) sign
      strncpy (&locTag[1], tag, 3);
    }
  /* Custom tag */
  else if (4 == strlen (tag))
    {
      strncpy (locTag, tag, 4);
    }
  
  /* Continue only if we got a valid tag */
  if (0 != locTag [0])
    {
      uint16_t valLen = (uint16_t) strlen (value);
      uint16_t langCode = 0x55c4; /* 5-bit ascii for "und", undefined */
      uint32_t currentIndex = 0;
      uint32_t dataSize = valLen + 4;
      uint8_t *data = ATOM_MALLOC (dataSize);
      if (NULL != data)
        {
          ATOM_WRITE_U16 (valLen); /* Length of the value field */
          ATOM_WRITE_U16 (langCode); /* Language code, set to "und" (undefined) */
          ATOM_WRITE_BYTES (value, valLen); /* Actual value */
          retAtom = atomFromData (dataSize, locTag, data);
          ATOM_FREE (data);
          data = NULL;
        }
    }
  return retAtom;
}

movie_atom_t *ardtAtomFromPathAndDroneVersion(const char *path, uint8_t droneVersion)
{
    movie_atom_t *retAtom = NULL;
    uint32_t pathLength = 0;
    uint32_t versionStringLen = 4; /* "xxx|" format, padded with zeros if version is too small */

    if(path != NULL)
    {
        pathLength = strlen(path);
    }
    

    uint32_t dataSize = pathLength + versionStringLen + 4; // 2 for version + 2 for string size without '\0'
    uint8_t *data = ATOM_MALLOC(dataSize);
    if (NULL != data)
    {
        uint32_t currentIndex = 0;
        
        ATOM_WRITE_U16 (0); /* Version */
        ATOM_WRITE_U16 (pathLength + versionStringLen); /* length of string */
        ATOM_WRITE_U8 ('0' + ((droneVersion / 100) % 10)); /* Drone version number */
        ATOM_WRITE_U8 ('0' + ((droneVersion /  10) % 10)); /* Drone version number */
        ATOM_WRITE_U8 ('0' + ((droneVersion      ) % 10)); /* Drone version number */
        ATOM_WRITE_U8 ('|'); /* separator between drone version and path */
        if(path != NULL)
        {
            ATOM_WRITE_BYTES(path, pathLength);
        }
        retAtom = atomFromData (dataSize, "ardt", data);
        ATOM_FREE (data);
        data = NULL;
    }
    // NO ELSE - malloc failed
    
    return retAtom;
}
