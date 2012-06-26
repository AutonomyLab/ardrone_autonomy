/**
 *  \brief    VP OS. Types declaration.
 *  \brief    These types are used to provide clean types declaration for portability between OSes
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 19/12/2006
 *  \date     modification  26/03/2007
 */

#ifndef _VP_SDK_TYPES_H_
#define _VP_SDK_TYPES_H_

#include <VP_Os/vp_os.h>
#include <stdint.h>

#if defined(USE_LINUX) || defined(__ELINUX__)
# include "vp_os_serial.h"
#endif // USE_LINUX

#define C_RESULT        int
#define C_OK            0
#define C_FAIL          -1

#define VP_SUCCESS         (0)
#define VP_FAILURE            (!VP_SUCCESS)

#define VP_SUCCEEDED(a)	(((a) & 0xffff) == VP_SUCCESS)
#define VP_FAILED(a)	(((a) & 0xffff) != VP_SUCCESS)

#ifndef _WIN32
	/* 
	Those macros have been used in the SDK but are already defined in Windows.h
	and thus used with a wrong value when compiling under Windows, leading to a crash.
	VP_xxx macros should be used to avoid confusion. 
	*/
	#define SUCCESS VP_SUCCESS
	#define FAIL VP_FAILURE

	#define SUCCEED	VP_SUCCEEDED
	#define FAILED VP_FAILED	
#endif

#ifdef USE_MINGW32
#include <windows.h>
#include <windef.h>
//typedef unsigned __int64 off64_t;
#endif

#if defined(_WIN32) || defined(USE_MINGW32)

// Definition des types entiers
typedef signed    __int8    int8_t;
typedef unsigned  __int8    uint8_t;
typedef signed    __int16   int16_t;
typedef unsigned  __int16   uint16_t;
typedef signed    __int32   int32_t;
typedef unsigned  __int32   uint32_t;
typedef signed    __int64   int64_t;
typedef unsigned  __int64   uint64_t;

#endif // < _WIN32

#if defined(__NDS__) || defined(TARGET_OS_IPHONE) || defined(TARGET_IPHONE_SIMULATOR)

#ifdef __NDS__
#include <nds.h>
#endif // ! __NDS__

#ifndef NULL
#define NULL (void*)0
#endif

#endif // < __NDS__ || TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR

typedef float               float32_t;
typedef double              float64_t;

#if defined(__linux__) && !defined(USE_MINGW32)
#include <stdint.h>
#include <stddef.h>

#define CBOOL int

#endif // < __linux__

#if defined(USE_ANDROID)

#include <time.h>
#undef __FD_ZERO
#define __FD_ZERO(fdsetp)   (vp_os_memset (fdsetp, 0, sizeof (*(fd_set *)(fdsetp))))
#undef FD_ZERO
#define FD_ZERO(fdsetp) __FD_ZERO(fdsetp)

#endif

#define bool_t  int32_t

#if defined(USE_PARROTOS_CORE)
#include <generic/parrotOS_types.h>
#define TYPEDEF_BOOL
#endif

#if !defined(USE_MINGW32)

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#endif // < USE_MINGW32

#if !defined(USE_ANDROID)
typedef volatile uint8_t    vuint8;
typedef volatile uint16_t   vuint16;
typedef volatile uint32_t   vuint32;
typedef volatile uint64_t   vuint64;

typedef volatile int8_t     vint8;
typedef volatile int16_t    vint16;
typedef volatile int32_t    vint32;
typedef volatile int64_t    vint64;

#ifndef INT_MAX
#define INT_MAX 2147483647
#endif // ! INT_MAX

#endif // !USE_ANDROID

#ifdef __linux__
#    include <VP_Os/linux/intrin.h>
#endif

#define BDADDR_SIZE   6

#if !defined(__BLUETOOTH_H)
typedef struct _bdaddr_t
{
  uint8_t b[BDADDR_SIZE];
} bdaddr_t;
#endif // !defined(__BLUETOOTH_H)

typedef C_RESULT (*Read)  (void* s, uint8_t* buffer, int32_t* size);
typedef C_RESULT (*Write) (void* s, const uint8_t* buffer, int32_t* size);

#endif // _TYPES_H_
