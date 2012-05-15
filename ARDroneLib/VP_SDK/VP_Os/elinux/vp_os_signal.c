/**
 * @file signal.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#include "VP_Os/vp_os_signal.h"

#ifndef __USE_GNU
#define __USE_GNU
#endif
#include <sys/time.h>
#include <errno.h>

void
vp_os_mutex_init(vp_os_mutex_t *mutex)
{
  pthread_mutex_init((pthread_mutex_t *)mutex, NULL);
}


void
vp_os_mutex_destroy(vp_os_mutex_t *mutex)
{
  pthread_mutex_destroy((pthread_mutex_t *)mutex);
}


void
vp_os_mutex_lock(vp_os_mutex_t *mutex)
{
  pthread_mutex_lock((pthread_mutex_t *)mutex);
}

C_RESULT
vp_os_mutex_trylock(vp_os_mutex_t *mutex)
{
  return pthread_mutex_trylock((pthread_mutex_t *)mutex) ? C_FAIL : C_OK;
}

void
vp_os_mutex_unlock(vp_os_mutex_t *mutex)
{
  pthread_mutex_unlock((pthread_mutex_t *)mutex);
}


void
vp_os_cond_init(vp_os_cond_t *cond, vp_os_mutex_t *mutex)
{
  pthread_cond_init(&cond->cond, NULL);
  cond->mutex = mutex;
}


void
vp_os_cond_destroy(vp_os_cond_t *cond)
{
  pthread_cond_destroy(&cond->cond);
}


void
vp_os_cond_wait(vp_os_cond_t *cond)
{
  pthread_cond_wait(&cond->cond, (pthread_mutex_t *)cond->mutex);
}


C_RESULT
vp_os_cond_timed_wait(vp_os_cond_t *cond, uint32_t ms)
{
  struct timespec ts;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  TIMEVAL_TO_TIMESPEC(&tv, &ts);
/*  ts.tv_sec += ms/1000;
//  ts.tv_nsec += (ms%1000)*1000;
  ts.tv_nsec += (ms%1000)*1000000;*/

  int tmp;
  ts.tv_nsec += ms * 1000000;
  tmp = ts.tv_nsec / (1000 * 1000000);
  ts.tv_sec += tmp;
  ts.tv_nsec -= tmp * (1000 * 1000000);
  return ( pthread_cond_timedwait(&cond->cond, (pthread_mutex_t *)cond->mutex, &ts) == ETIMEDOUT ? FAIL : SUCCESS );
}


void
vp_os_cond_signal(vp_os_cond_t *cond)
{
  pthread_cond_signal(&cond->cond);
}

