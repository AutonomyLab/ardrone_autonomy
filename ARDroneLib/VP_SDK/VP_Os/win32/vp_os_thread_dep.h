/**
 * @file vp_os_thread_dep.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _THREAD_INCLUDE_OS_DEP_
#define _THREAD_INCLUDE_OS_DEP_


#include <windows.h>

typedef HANDLE                  THREAD_HANDLE;
typedef LPTHREAD_START_ROUTINE  THREAD_ROUTINE;
typedef LPVOID                  THREAD_PARAMS;
typedef DWORD                   THREAD_RET;

#define THREAD_RETURN(value) return (value)

#endif // ! _THREAD_INCLUDE_OS_DEP_

