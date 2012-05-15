/**
 * @file thread.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#include "VP_Os/vp_os_thread.h"
#include "VP_Os/vp_os_assert.h"

void
vp_os_thread_create(THREAD_ROUTINE f, void *parameters, THREAD_HANDLE *handle, ...)
{
  unsigned long id;

  *handle = CreateThread
    (
     NULL,      // security
     0,         // stack size (common)
     f,         // start (common)
     parameters,// parameters given to thread routine (common)
     0,         // creation flags
     &id        // id
    );
}

THREAD_HANDLE
vp_os_thread_self(void)
{
  return GetCurrentThread();
}

void
vp_os_thread_join(THREAD_HANDLE handle)
{
  WaitForSingleObject(handle, INFINITE);
}

void
vp_os_thread_suspend(THREAD_HANDLE handle)
{
  SuspendThread(handle);
}

void
vp_os_thread_resume(THREAD_HANDLE handle)
{
  ResumeThread(handle);
}

void
vp_os_thread_yield(void)
{
 VP_OS_ASSERT(0==1);
}

void
vp_os_thread_priority(THREAD_HANDLE handle, int32_t priority)
{
  SetThreadPriority(handle, priority);
}
