/**
 * @file vp_os_thread_dep.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _THREAD_INCLUDE_OS_DEP_
#define _THREAD_INCLUDE_OS_DEP_


#include "parrotOS_thread.h"

#include <VP_Os/vp_os_types.h>

#define THREAD_HANDLE   SUP_THREAD_HANDLE
#define THREAD_ROUTINE  SUP_THREAD_ENTRY

typedef void THREAD_RET;
typedef void *THREAD_PARAMS;

#define THREAD_RETURN(value)

#endif // ! _THREAD_INCLUDE_OS_DEP_

