#include <VP_Os/vp_os_malloc.h>
#include <ardrone_tool/ardrone_version.h>
#include <utils/ardrone_ftp.h>
#include <config.h>

#include <stdio.h>


#ifndef USE_ELINUX

int
compareVersions (ardrone_version_t *v1, ardrone_version_t *v2)
{
  if (NULL == v1 || NULL == v2)
    {
      return 0;
    }
  int retVal =  v1->majorVersion - v2->majorVersion;
  if (0 == retVal)
    {
      retVal = v1->minorVersion - v2->minorVersion;
      if (0 == retVal)
        {
          retVal = v1->revision - v2->revision;
        }
    }
  return retVal;
}

int
getDroneVersion (const char *tempPath, const char *droneIp, ardrone_version_t *version)
{
  if (NULL == tempPath || NULL == droneIp || NULL == version)
    {
      return -1;
    }
  _ftp_status status;
  _ftp_t *ftp = ftpConnect (droneIp, FTP_PORT, "anonymous", "", &status);
  if (FTP_FAILED (status) || NULL == ftp)
    {
      ftpClose (&ftp);
      return -1;
    }

  size_t lNameSize = strlen (tempPath) + strlen ("/__version.txt") + 1;
  char *localName = vp_os_calloc (lNameSize, 1);
  if (NULL == localName)
    {
      ftpClose (&ftp);
      return -1;
    }

  snprintf (localName, lNameSize, "%s/__version.txt", tempPath);
  status = ftpGet (ftp, "version.txt", localName, 0, NULL);
  if (FTP_FAILED (status))
    {
      vp_os_free (localName);
      localName = NULL;
      ftpClose (&ftp);
      return -1;
    }
  
  ftpClose (&ftp);
  
  FILE *versionFile = fopen (localName, "r");
  if (NULL == versionFile)
    {
      remove (localName);
      vp_os_free (localName);
      localName = NULL;
      return -1;
    }

  uint32_t maj, min, rev;
  if (3 != fscanf (versionFile, "%u.%u.%u", &maj, &min, &rev))
    {
      fclose (versionFile);
      remove (localName);
      vp_os_free (localName);
      localName = NULL;
      return -1;
    }
  
  fclose (versionFile);
  remove (localName);
  vp_os_free (localName);
  localName = NULL;
  
  version->majorVersion = maj;
  version->minorVersion = min;
  version->revision = rev;

  return 0;
}

#endif
