/**
 * @file assert.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/27
 */

#ifndef _ASSERT_INCLUDE_OS_
#define _ASSERT_INCLUDE_OS_


#include <assert.h>

#ifdef DEBUG_MODE
# define VP_OS_ASSERT(expr) assert(expr)
#else // ! DEBUG_MODE
# define VP_OS_ASSERT(expr) ((void)0)
#endif // <- DEBUG_MODE


#endif // ! _ASSERT_INCLUDE_OS_

