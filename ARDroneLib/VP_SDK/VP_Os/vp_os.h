/**
***************************************************************************
*
* Copyright (C) 2007 Parrot S.A.
*
* \date First Release 03/01/2007
* \date Last Modified 15/06/2009
***************************************************************************
*/

#ifndef _OS_H_
#define _OS_H_

#include <VP_Os/vp_os_rtmon.h>


/*
 * Platforms support
 */

#ifdef _WIN32
#include <windows.h>
#include <windef.h>
#endif // _WIN32

#ifdef __MACOSX__
#include "TargetConditionals.h"
#endif //


#undef INLINE

/*
 * Compilers support
 */
#ifdef _MSC_VER // Microsoft visual C++

//#undef  FAILED
#define inline  __inline
#define INLINE  __forceinline

#define WEAK

#endif // _MSC_VER

#ifdef __GNUC__ // The Gnu Compiler Collection

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#ifndef USE_MINGW32
#define WINAPI
#else // USE_MINGW32
//#undef  FAILED
#endif // USE_MINGW32

#define INLINE __inline__ __attribute__((always_inline))

#define WEAK __attribute__((weak))
#define NO_INSTRUMENT  __attribute__ ((no_instrument_function))

#endif // __GNUC__

#endif // _OS_H_

/*! \mainpage Video Product SDK

The VP_SDK is a small library that aims to ease video application developpment.
However it can be used to write portable code, to ease network initialization and to
stream any kind of data.

<hr>

<center><h2> VP_SDK as an abstraction layer </h2></center>

VP_SDK provides abstraction for

   <ul>
   <li> Delay (ms and us)
   <li> Signal (mutexes and conditionals)
   <li> threads
   </ul>

on the following platforms

   <ul>
   <li> linux
   <li> embedded linux
   <li> Win32
   </ul>

On top of this, we also encapsulated ParrotOs with our syntax.

<center><h2> VP_SDK main concepts </h2></center>

VP_SDK has two main concepts:

   <ul>
   <li> Stages
   <li> Pipeline
   </ul>

\section Stages

A stage is a piece of code with standardized input and output. Usually a stage is created to transform
some sort of data from one representation to another. Stages can be seen as a way to separate concerns, to
maximize reusability and to ease debugging.

\section Pipeline

A pipeline contains a set of stages and the link between them. Pipelines aim to ease data streaming.
The canonical example of pipelines is to have on an embedded system a camera and a wifi adapter and on PC
you have a wifi adapter and a screen. You'll define two pipelines.
The first one on your embedded system will have three stages:

   <ul>
   <li> Camera acquisition (camif or V4L for example)
   <li> Compression (VLIB for example)
   <li> Output Com (wifi for example)
   </ul>

The second one on your PC will have three stages too:

   <ul>
   <li> Input Com (wifi in our example)
   <li> Deompression (VLIB in our example)
   <li> Display (SDL for example)
   </ul>

<center><h2> Intrinsics </h2></center>

VP_SDK provides also defines that map to processor instruction so they can be used in C.

   <ul>
   <li> <tt>clz</tt> counts number of leading zero
   <li> <tt>bswap</tt> transforms an int from little endian to big endian
   </ul>
*/
