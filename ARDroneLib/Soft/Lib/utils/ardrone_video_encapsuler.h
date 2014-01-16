/*
 * ardrone_video_encapsuler.h
 * ARDroneLib
 *
 * Created by n.brulez on 18/08/11
 * Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#ifndef _ARDRONE_VIDEO_ENCAPSULER_H_
#define _ARDRONE_VIDEO_ENCAPSULER_H_

#include <VP_Os/vp_os_signal.h>
#include <video_encapsulation.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#else // Linux
#include <dirent.h>
#endif

#define ARDRONE_VIDEO_PATH_MAX_SIZE (256)
#define ARDRONE_VIDEO_FRAMES_COUNT_LIMIT (131072)

#define COUNT_WAITING_FOR_IFRAME_AS_AN_ERROR (0)

#define ARDRONE_VIDEO_VERSION_NUMBER (1)
#define ARDRONE_VIDEO_INFO_PATTERN "%u:%c|"
#define ARDRONE_VIDEO_NUM_MATCH_PATTERN (2)

typedef enum {
  ARDRONE_VIDEO_NO_ERROR = 0,
  ARDRONE_VIDEO_GENERIC_ERROR,
  ARDRONE_VIDEO_BAD_CODEC,
  ARDRONE_VIDEO_FILE_ERROR,
  ARDRONE_VIDEO_WAITING_FOR_IFRAME,
  ARDRONE_VIDEO_BAD_ARGS,
} ardrone_video_error_t;

#if COUNT_WAITING_FOR_IFRAME_AS_AN_ERROR
#define ARDRONE_VIDEO_FAILED(errCode) ((errCode) != ARDRONE_VIDEO_NO_ERROR)
#define ARDRONE_VIDEO_SUCCEEDED(errCode) ((errCode) == ARDRONE_VIDEO_NO_ERROR)
#else
static inline int ARDRONE_VIDEO_FAILED (ardrone_video_error_t error)
{
  if (ARDRONE_VIDEO_NO_ERROR == error ||
      ARDRONE_VIDEO_WAITING_FOR_IFRAME == error)
    {
      return 0;
    }
  return 1;
}
static inline int ARDRONE_VIDEO_SUCCEEDED (ardrone_video_error_t error)
{
  if (ARDRONE_VIDEO_NO_ERROR == error ||
      ARDRONE_VIDEO_WAITING_FOR_IFRAME == error)
    {
      return 1;
    }
  return 0;
}
#endif // COUNT_WAITING_FOR_IFRAME_AS_AN_ERROR


typedef enum {
  ARDRONE_VIDEO_MOV = 0,
  ARDRONE_VIDEO_QUICKTIME = ARDRONE_VIDEO_MOV, // Alias for mov file
  ARDRONE_VIDEO_MP4,
} ardrone_video_type_t;

typedef struct _ardrone_video_t {
    uint32_t version;
  // Provided to constructor
  uint32_t fps;
  ardrone_video_type_t videoType;
  // Read from frames PaVE
  parrot_video_encapsulation_codecs_t videoCodec;
  uint16_t width;
  uint16_t height;
  // Private datas
  char infoFilePath [ARDRONE_VIDEO_PATH_MAX_SIZE]; // Keep this so we can delete the file
  char outFilePath  [ARDRONE_VIDEO_PATH_MAX_SIZE]; // Keep this so we can rename the output file
  char tempFilePath [ARDRONE_VIDEO_PATH_MAX_SIZE]; // Keep this so we can rename the output file
  FILE *infoFile;
  FILE *outFile;
  uint32_t framesCount; // Number of frames
  uint32_t mdatAtomOffset;
  uint32_t framesDataOffset;

  /* H.264 only values */
  uint8_t *sps;
    uint8_t *pps;
  uint16_t spsSize;
  uint16_t ppsSize;

  /* Slices recording values */
  uint32_t lastFrameNumber;
  parrot_video_encapsulation_frametypes_t lastFrameType;
  uint32_t currentFrameSize;
  
  vp_os_mutex_t mutex;
    time_t creationTime;
    uint32_t droneVersion;
} ardrone_video_t;

/**
 * @brief Start a new video with quicktime or mpeg4 format
 * @param videoPath Path where the video should be saved (must contain the .mov/.mp4 extension)
 * @param fps Frames per second of the video
 * @param vType Type of the output video (container format)
 * @param error Pointer to a ardrone_video_error_t that will contain the return code (can't be null)
 * @return A pointer to the video structure allocated during the call, of NULL if anything failed (in this case, see the value of *error)
 */
ardrone_video_t *ardrone_video_start (const char *videoPath, int fps, ardrone_video_type_t vType, ardrone_video_error_t *error);

/**
 * Add a frame to an encapsulated video
 * The actual writing of the video will start on the first given I-Frame. (after that, each frame will be written)
 * The first I-Frame will be used to get the video codec, height and width of the video
 * @brief Add a new frame to a video
 * @param video The video on which you want to add the frame to
 * @param frame Pointer to frame data (WITH PaVE INCLUDED !)
 * @return Possible return values are :
 * @return  - ARDRONE_VIDEO_NO_ERROR if the frame was correctly added
 * @return  - ARDRONE_VIDEO_BAD_ARGS if the video or frame pointer is not correct
 * @return  - ARDRONE_VIDEO_FILE_ERROR if an error occured on file write
 * @return  - ARDRONE_VIDEO_BAD_CODEC if the video codec is not supported OR if the video codec changed during the recording
 * @return  - ARDRONE_VIDEO_WAIT_FOR_IFRAME when the recorder is waiting for an IFrame to start the actual recording
 * @return  - ARDRONE_VIDEO_GENERIC_ERROR in any other error case
 */
ardrone_video_error_t ardrone_video_addFrame (ardrone_video_t *video, uint8_t *frame);

/**
 * Add a slice to an encapsulated video
 * The actual writing of the video will start on the first given I-Frame slice. (after that, each frame will be written)
 * The first I-Frame slice will be used to get the video codec, height and width of the video
 * @brief Add a new slice to a video
 * @param video The video on which you want to add the slice to
 * @param slice Pointer to slice data (WITH PaVE INCLUDED !)
 * @return Possible return values are :
 * @return  - ARDRONE_VIDEO_NO_ERROR if the slice was correctly added
 * @return  - ARDRONE_VIDEO_BAD_ARGS if the video or slice pointer is not correct
 * @return  - ARDRONE_VIDEO_FILE_ERROR if an error occured on file write
 * @return  - ARDRONE_VIDEO_BAD_CODEC if the video codec is not supported OR if the video codec changed during the recording
 * @return  - ARDRONE_VIDEO_WAIT_FOR_IFRAME when the recorder is waiting for an IFrame to start the actual recording
 * @return  - ARDRONE_VIDEO_GENERIC_ERROR in any other error case
 */
ardrone_video_error_t ardrone_video_addSlice (ardrone_video_t *video, uint8_t *slice);

/**
 * Compute, write and close the video
 * This can take some time to complete
 * @param video pointer to your ardrone_video pointer (will be set to NULL by call)
 */
ardrone_video_error_t ardrone_video_finish (ardrone_video_t **video);

/**
 * Abort a video recording
 * This will delete all created files
 * @param video pointer to your ardrone_video pointer (will be set to NULL by call)
 */
ardrone_video_error_t ardrone_video_cleanup (ardrone_video_t **video);

/**
 * Cleanup directory for all previous files
 * @param dir directory to clean
 */
ardrone_video_error_t ardrone_video_remove_all (const char *dir);

/**
 * Get the string associated with an error code
 * @param error the error code
 * @param buf pointer for the return string
 * @param bufLen size of the return string pointer
 */
void ardrone_video_error_string (ardrone_video_error_t error, char *buf, uint32_t bufLen);

/**
 * Set the current GPS position for further recordings
 * @param latitude current latitude
 * @param longitude current longitude
 * @param altitude current altitude
 */
void ardrone_video_set_gps_infos (double latitude, double longitude, double altitude);

/**
 * Try fo fix an MP4 infovid file.
 * @param infoFilePath Full path to the .infovid file.
 * @return TRUE on success, FALSE on failure
 */
bool_t ardrone_video_try_fix (const char *infoFilePath);

#endif // _ARDRONE_VIDEO_ENCAPSULER_H_
