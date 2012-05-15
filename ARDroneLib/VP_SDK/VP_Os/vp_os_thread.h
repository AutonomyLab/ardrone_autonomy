/**
 * @file thread.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _THREAD_INCLUDE_OS_
#define _THREAD_INCLUDE_OS_

#include <VP_Os/vp_os_types.h>
#include "vp_os_thread_dep.h"


/**
 * Prototype definition of a thread routine.
 */
#define DEFINE_THREAD_ROUTINE(NomRoutine,NomParams) THREAD_RET WINAPI thread_##NomRoutine(THREAD_PARAMS NomParams)


/**
 * Creates and starts a thread.
 *
 * @param f          The thread routine
 * @param parameters The parameters passed to the thread routine
 * @param handle     The returned handle of the created thread
 *
 * Variadic parameters : only for eCos
 * @param sched_info Sechduling priority
 * @param name       Thread name
 * @param stack_base Stack address
 * @param stack_size Stack size
 * @param thread     cyg_thread_t object
 */
void
vp_os_thread_create(THREAD_ROUTINE f, THREAD_PARAMS parameters, THREAD_HANDLE *handle, ...);

/**
 * Waits for a thread to terminate.
 *
 * @param handle The handle of the thread to wait for the termination
 */
void
vp_os_thread_join(THREAD_HANDLE handle);

THREAD_HANDLE
vp_os_thread_self(void);

void
vp_os_thread_suspend(THREAD_HANDLE handle);

void
vp_os_thread_resume(THREAD_HANDLE handle);

void
vp_os_thread_yield(void);

void
vp_os_thread_priority(THREAD_HANDLE handle, int32_t priority);

#endif // ! _THREAD_INCLUDE_OS_

