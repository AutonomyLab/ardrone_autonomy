// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck@parrot.fr>
//  Date   : 03/01/2007
//
//  Parrot Video SDK : Os API
//
// ----------------------------------------------

#ifndef _OS_PRINT_H_
#define _OS_PRINT_H_

#include <stdio.h>

#ifdef __MACOSX__
#ifdef DEBUG_MODE
#define PRINT printf
#else
#define PRINT(...)
#endif // DEBUG_MODE
#else
#define PRINT printf
#endif // __MACOSX__

#ifdef USE_ANDROID
  #include <android/log.h>
  #undef PRINT
  #ifdef DEBUG_MODE
    #define PRINT(_fmt_, args...)                                        \
    __android_log_print(ANDROID_LOG_INFO, "ARDrone", _fmt_, ##args)
  #else
    #define PRINT(...)
  #endif //DEBUG_MODE
#endif // USE_ANDROID

#ifndef PRINT
	#define PRINT printf
#endif

#ifdef DEBUG_MODE
  #define DEBUG_PRINT_SDK(...) PRINT(__VA_ARGS__)
#else
  #define DEBUG_PRINT_SDK(...)
#endif

#ifdef DEBUG_MODE
  #define DEBUG_PRINT(...) PRINT(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
#endif

#endif // _OS_PRINT_H_
