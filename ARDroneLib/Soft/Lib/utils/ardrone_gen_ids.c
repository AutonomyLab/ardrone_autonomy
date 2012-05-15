#include <utils/ardrone_gen_ids.h>
#include <utils/ardrone_crc_32.h>
#include <string.h>
#include <stdio.h>

#include <stdlib.h>
#include <time.h>

void
ardrone_gen_appid (const char *appName, const char *sdkVersion, char appId [9], char *appDesc, int descLen)
{
#define _BUFFER_SIZE 512
  char appNamePlusSdk [_BUFFER_SIZE] = {0};
  snprintf (appNamePlusSdk, _BUFFER_SIZE, "%s:%s", appName, sdkVersion);
#undef _BUFFER_SIZE
  uint32_t binaryId = ardrone_crc_32 ((uint8_t *)appNamePlusSdk, strlen (appNamePlusSdk));
  snprintf (appId, 9, "%08x", binaryId);
  appId [8] = '\0';
  strncpy (appDesc, appName, descLen);
}

void
ardrone_gen_usrid (const char *usrName, char usrId [9], char *usrDesc, int descLen)
{
  uint32_t binaryId = ardrone_crc_32 ((uint8_t *)usrName, strlen (usrName));
  snprintf (usrId, 9, "%08x", binaryId);
  usrId [8] = '\0';
  strncpy (usrDesc, usrName, descLen);
}

void
ardrone_gen_sessionid (char sessId [9], char *sessDesc, int descLen)
{
  static int runOnce = 1;
  if (1 == runOnce)
    {
      srand (time (NULL));
      runOnce = 0;
    }
  uint32_t binaryId = (uint32_t)rand ();
  binaryId = (0 != binaryId) ? binaryId : 1u;
  snprintf (sessId, 9, "%08x", binaryId);
  sessId [8] = '\0';
  snprintf (sessDesc, descLen, "Session %s", sessId);
}
