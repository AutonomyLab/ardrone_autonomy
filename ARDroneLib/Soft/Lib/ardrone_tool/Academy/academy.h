/**
 *  academy.h
 *
 *  Created by Frederic D'HAEYER on 8/04/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#ifndef _ACADEMY_H_
#define _ACADEMY_H_

#include <ardrone_api.h>
#include <academy_common.h>
#include <ardrone_tool/ardrone_tool.h>

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <utils/ardrone_ftp.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Os/vp_os_thread.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_signal.h>
#include <stdio.h>
#include <dirent.h>
#ifndef USE_ANDROID
#include <ftw.h>
#else
#include <utils/AR_Ftw.h>
#endif

#include <time.h>

#ifndef _WIN32
	#include <sys/time.h>
	#include <unistd.h>
#else

 	#include <sys/timeb.h>
	#include <Winsock2.h>  // for timeval structure
#endif

#ifdef DEBUG
#define PILOTACADEMY_DEBUG
#endif

#define ACADEMY_MAX_LIST		1024
#define ACADEMY_MAX_LINE		128
#define ACADEMY_MAX_FILENAME	256
#define ACADEMY_MAX_FD_FOR_FTW  20

#define PA_PREFIX "PA : "

#ifdef PILOTACADEMY_DEBUG
#define PA_DEBUG(...)			\
  do							\
    {							\
      PRINT (PA_PREFIX);		\
      PRINT (__VA_ARGS__);		\
    } while (0)
#define PA_WARNING(...)			\
do							\
{							\
PRINT (PA_PREFIX);		\
PRINT (__VA_ARGS__);		\
} while (0)
#else
#define PA_DEBUG(...)
#define PA_WARNING(...)		\
do							\
{							\
PRINT (PA_PREFIX);		\
PRINT (__VA_ARGS__);		\
} while (0)
#endif

#define ACADEMY_SERVERNAME			"parrot01.nyx.emencia.net"
#define ACADEMY_PORT				21
#define BYTE_TO_MBYTE(a) (a / 1048576)
#define MBYTE_TO_BYTE(a) (a * 1048576)

void academy_init(const char *flight_dir, int max_storing_size);
void academy_shutdown(void);
C_RESULT academy_connect(const char *username, const char *password, academy_callback callback);
C_RESULT academy_disconnect(void);

extern char flight_dir[];

#endif // _ACADEMY_H_
