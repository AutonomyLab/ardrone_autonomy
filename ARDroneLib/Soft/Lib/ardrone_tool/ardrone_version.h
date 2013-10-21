#ifndef _ARDRONE_VERSION_H_
#define _ARDRONE_VERSION_H_

#include <VP_Os/vp_os_types.h>

typedef struct _ardrone_version_t
{
  uint32_t majorVersion;
  uint32_t minorVersion;
  uint32_t revision;
} ardrone_version_t;


#ifndef USE_ELINUX
extern ardrone_version_t ardroneVersion;

/**
 * @brief Get the AR.Drone model from its version
 * @param version pointer to the ardrone_version_t structure
 * @return the model number of the AR.Drone
 */
static inline uint32_t ardroneModelFromVersion (ardrone_version_t *version)
{
  if (NULL == version)
    {
      return 0;
    }
  return version->majorVersion;
}

/*
 * Macros for easier use of ardroneModelFromVersion 
 */
#define ARDRONE_VERSION() ardroneModelFromVersion(&ardroneVersion)

/**
 * @brief Compare two ardrone_versions
 * @param v1 first version to compare
 * @param v2 second version to compare
 * @return negative if v1<v2, zero if v1=v2, positive if v1>v2
 * @return (return zero if any version pointer is NULL)
 */
int compareVersions (ardrone_version_t *v1, ardrone_version_t *v2);

/**
 * @brief Get current drone version
 * @param tempPath a valid path to store a temporary file (typically : root_dir)
 * @param droneIp IP address of the AR.Drone as a string (e.g. "192.168.1.1")
 * @param version pointer to the return version
 * @return 0 when no error occured, -1 otherwise. version is NOT changed on error
 */
int getDroneVersion (const char *tempPath, const char *droneIp, ardrone_version_t *version);

#else

# if CARD_VERSION >= 0x20
#  define ARDRONE_VERSION() 2
# else
#  define ARDRONE_VERSION() 1
# endif

#define getDroneVersion(...) ARDRONE_VERSION()

#endif

#define IS_ARDRONE1 (1 == ARDRONE_VERSION()) // Drone 1
#define IS_ARDRONE2 (2 == ARDRONE_VERSION()) // Drone 2
#define IS_LEAST_ARDRONE2 (2 <= ARDRONE_VERSION()) // Drone 2 and laters

#endif //_ARDRONE_VERSION_H_
