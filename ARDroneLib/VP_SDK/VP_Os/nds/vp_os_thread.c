/**
 * @file vp_os_thread.c
 * @author aurelien.morelle@parrot.com
 * @date 2007/03/04
 */

#include <stdarg.h>
#include <VP_Os/vp_os_thread.h>
#include <VP_Os/vp_os_types.h>

void
vp_os_thread_create(THREAD_ROUTINE f, THREAD_PARAMS parameters, THREAD_HANDLE *handle, ...)
{
}


void
vp_os_thread_join(THREAD_HANDLE handle)
{
}

void
vp_os_thread_suspend(THREAD_HANDLE handle)
{
}

void
vp_os_thread_resume(THREAD_HANDLE handle)
{
}

void
vp_os_thread_yield(void)
{
}

void
vp_os_thread_priority(THREAD_HANDLE handle, int32_t priority)
{
}

