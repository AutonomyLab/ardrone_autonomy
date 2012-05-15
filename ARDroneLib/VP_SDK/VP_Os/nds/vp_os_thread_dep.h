/**
 * @file vp_os_thread_dep.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _THREAD_INCLUDE_OS_DEP_
#define _THREAD_INCLUDE_OS_DEP_


typedef void *THREAD_HANDLE;
typedef void *THREAD_PARAMS;
typedef void THREAD_RET;
typedef THREAD_RET (*THREAD_ROUTINE) (THREAD_PARAMS);


#endif // ! _THREAD_INCLUDE_OS_DEP_

