/**
 * @file vp_os_thread_dep.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _THREAD_INCLUDE_OS_DEP_
#define _THREAD_INCLUDE_OS_DEP_


#include <pthread.h>


typedef pthread_t THREAD_HANDLE;
typedef void *THREAD_PARAMS;
typedef void *THREAD_RET;
typedef THREAD_RET (*THREAD_ROUTINE) (THREAD_PARAMS);

#define THREAD_RETURN(value) return ((void*)value)

#endif // ! _THREAD_INCLUDE_OS_DEP_

