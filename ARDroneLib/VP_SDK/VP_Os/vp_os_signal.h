/**
 * @file signal.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _SIGNAL_INCLUDE_OS_
#define _SIGNAL_INCLUDE_OS_


#include <VP_Os/vp_os_types.h>

#include <vp_os_signal_dep.h>

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * Initializes a mutex.
 *
 * @param mutex The mutex to initialize
 */
void
vp_os_mutex_init(vp_os_mutex_t *mutex);

/**
 * Destroys a mutex.
 *
 * @param mutex The mutex to destroy
 */
void
vp_os_mutex_destroy(vp_os_mutex_t *mutex);

/**
 * Locks a mutex.
 *
 * @param mutex The mutex to lock
 */
void
vp_os_mutex_lock(vp_os_mutex_t *mutex);

/**
 * Tries to lock a mutex.
 *
 * @param mutex The mutex to lock
 */
C_RESULT
vp_os_mutex_trylock(vp_os_mutex_t *mutex);


/**
 * Unlocks a mutex.
 *
 * @param mutex The mutex to unlock
 */
void
vp_os_mutex_unlock(vp_os_mutex_t *mutex);

/**
 * Initializes a condition variable.
 *
 * @param cond  The condition to initialize
 * @param mutex The mutex associated to this condition variable
 */
void
vp_os_cond_init(vp_os_cond_t *cond, vp_os_mutex_t *mutex);

/**
 * Destroys a condition variable.
 *
 * @param cond The condition to destroy
 */
void
vp_os_cond_destroy(vp_os_cond_t *cond);

/**
 * Waits for the signal of a condition variable.
 *
 * @param cond The condition to wait for
 */
void
vp_os_cond_wait(vp_os_cond_t *cond);

/**
 * Waits for the signal of a condition variable.
 *
 * @param cond The condition to wait for
 * @param ms Time to wait in milliseconds
 *
 * @return  VP_SUCCESS or VP_FAILURE (if delay has been reached before or not)
 */
C_RESULT
vp_os_cond_timed_wait(vp_os_cond_t *cond, uint32_t ms);

/**
 * Signals a condition variable.
 *
 * @param cond The condition to signal
 */
void
vp_os_cond_signal(vp_os_cond_t *cond);

/**
 * Broadcasts a condition variable.
 *
 * @param cond The condition to signal
 */
void
vp_os_cond_broadcast(vp_os_cond_t *cond);

#ifdef __cplusplus
}
#endif

#endif // ! _SIGNAL_INCLUDE_OS_

