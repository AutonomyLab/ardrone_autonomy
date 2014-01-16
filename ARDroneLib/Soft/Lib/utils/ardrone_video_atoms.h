/*
 * ardrone_video_atoms.h
 * ARDroneLib
 *
 * Created by n.brulez on 19/08/11
 * Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#ifndef _ARDRONE_VIDEO_ATOMS_H_
#define _ARDRONE_VIDEO_ATOMS_H_

#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include "ardrone_video_encapsuler.h"

typedef struct _movie_atom_t {
  uint32_t size;
  char tag [4];
  uint8_t *data;
  uint8_t wide;
} movie_atom_t;

/* Atoms :
LEGEND :
-> specific = specific function to generate this atom
-> empty = use atomFromData (0, "name", NULL);
-> from data = use atomFromData (dataSize, "name", dataPointer);

ftyp -> specific
mdat -> specific (include freeAtom if needed)
moov -> empty
 |- mvhd -> specific
 |- trak -> empty
 |   |- tkhd -> specific
 |   \- mdia -> empty
 |       |- mdhd -> specific
 |       |- hdlr -> specific (for mdia)
 |       \- minf -> empty
 |           |- vmhd -> specific
 |           |- hdlr -> specific (for minf)
 |           |- dinf -> empty
 |           |   \- dref -> specific
 |           \- stbl -> empty
 |               |- stsd -> specific
 |               |- stts -> specific
 |               |- stss -> from data (i frame positions as uint32_t network endian)
 |               |- stsc -> specific
 |               |- stsz -> from data (frames sizes as uint32_t network endian)
 |               \- stco -> from data (frames offset as uint32_t network endian)
 \- udta -> empty
     |- meta1 -> all meta specific (metadataAtomFromTagAndValue)
   [...]
     \- metaN -> all meta specific (metadataAtomFromTagAndValue)
ardt -> specific
*/

/* REMINDER : NEVER INCLUDE A MDAT ATOM INTO ANY OTHER ATOM */

/* GENERIC */
movie_atom_t *atomFromData (uint32_t data_size, const char *_tag, const uint8_t *_data);
void insertAtomIntoAtom (movie_atom_t *container, movie_atom_t **leaf); // will free leaf
int writeAtomToFile (movie_atom_t **_atom, FILE *file); // Will free _atom
void freeAtom (movie_atom_t **_atom);

/* SPECIFIC */
movie_atom_t *ftypAtomForFormatAndCodecWithOffset (ardrone_video_type_t format, parrot_video_encapsulation_codecs_t codec, uint32_t *offset, movie_atom_t **freeAtom);
movie_atom_t *mdatAtomForFormatWithVideoSize (ardrone_video_type_t format, uint64_t videoSize);
movie_atom_t *mvhdAtomFromFpsNumFramesAndDate (uint32_t fps, uint32_t nbFrames, time_t date);
movie_atom_t *tkhdAtomWithResolutionNumFramesFpsAndDate (uint32_t w, uint32_t h, uint32_t nbFrames, uint32_t fps, time_t date);
movie_atom_t *mdhdAtomFromFpsNumFramesAndDate (uint32_t fps, uint32_t nbFrames, time_t date);
movie_atom_t *hdlrAtomForMdia ();
movie_atom_t *vmhdAtomGen ();
movie_atom_t *hdlrAtomForMinf ();
movie_atom_t *drefAtomGen ();
movie_atom_t *stsdAtomWithResolutionAndCodec (uint32_t w, uint32_t h, parrot_video_encapsulation_codecs_t codec);
movie_atom_t *stsdAtomWithResolutionCodecSpsAndPps (uint32_t w, uint32_t h, parrot_video_encapsulation_codecs_t codec, uint8_t *sps, uint32_t spsSize, uint8_t *pps, uint32_t ppsSize);
movie_atom_t *sttsAtomWithNumFrames (uint32_t nbFrames);
movie_atom_t *stscAtomGen ();
movie_atom_t *metadataAtomFromTagAndValue (const char *tag, const char *value);
movie_atom_t *ardtAtomFromPathAndDroneVersion(const char *path, uint8_t droneVersion);

#endif // _ARDRONE_VIDEO_ATOMS_H_
