/**
 * @file signal.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#include "VP_Os/vp_os_signal.h"

	#if defined USE_WINDOWS_CONDITION_VARIABLES
	#elif defined USE_PTHREAD_FOR_WIN32
	#else
	#endif

void
vp_os_mutex_init(vp_os_mutex_t *mutex)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		InitializeCriticalSection((CRITICAL_SECTION *)mutex);
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_mutex_init((pthread_mutex_t *)mutex, NULL);
	#else
	#endif	
}


void
vp_os_mutex_destroy(vp_os_mutex_t *mutex)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		DeleteCriticalSection((CRITICAL_SECTION *)mutex);
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_mutex_destroy((pthread_mutex_t *)mutex);
	#else
	#endif 
}


void
vp_os_mutex_lock(vp_os_mutex_t *mutex)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		EnterCriticalSection((CRITICAL_SECTION *)mutex);
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_mutex_lock((pthread_mutex_t *)mutex);
	#else
	#endif  
}


void
vp_os_mutex_unlock(vp_os_mutex_t *mutex)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		LeaveCriticalSection((CRITICAL_SECTION *)mutex);
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_mutex_unlock((pthread_mutex_t *)mutex);
	#else
	#endif
	
}


void
vp_os_cond_init(vp_os_cond_t *cond, vp_os_mutex_t *mutex)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		InitializeConditionVariable(&cond->cond);
		cond->mutex = mutex;
	#elif defined USE_PTHREAD_FOR_WIN32
		  pthread_cond_init(&cond->cond, NULL);
		  cond->mutex = mutex;
	#else
		/**/
	#endif
}


void
vp_os_cond_destroy(vp_os_cond_t *cond)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		/**/
	#elif defined USE_PTHREAD_FOR_WIN32
		 pthread_cond_destroy(&cond->cond);
	#else
		/**/
	#endif
	
}


void
vp_os_cond_wait(vp_os_cond_t *cond)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		SleepConditionVariableCS(&cond->cond, (CRITICAL_SECTION *)cond->mutex, INFINITE);
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_cond_wait(&cond->cond, (pthread_mutex_t *)cond->mutex);
	#else
		WaitForSingleObject(cond->LockSemaphore,INFINITE); // TODO: to test
	#endif
}


int gettimeofday(struct timeval *tv, struct timezone *tz); 

C_RESULT
vp_os_cond_timed_wait(vp_os_cond_t *cond, uint32_t ms)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		return  SleepConditionVariableCS(&cond->cond, (CRITICAL_SECTION *)cond->mutex, ms) == WAIT_TIMEOUT ? VP_FAILURE : VP_SUCCESS;
	#elif defined USE_PTHREAD_FOR_WIN32
		  struct timespec ts;
		  struct timeval tv;
		  gettimeofday(&tv, NULL);
		  ts.tv_sec = tv.tv_sec + ms/1000;
		  ts.tv_nsec = (tv.tv_usec + (ms%1000))*1000;
		  return ( pthread_cond_timedwait(&cond->cond, (pthread_mutex_t *)cond->mutex, &ts) == ETIMEDOUT ? VP_FAILURE : VP_SUCCESS );
	#else
		return WaitForSingleObject(cond->LockSemaphore, ms) == WAIT_TIMEOUT ? VP_FAILURE : VP_SUCCESS; // TODO: to test
	#endif
}

void
vp_os_cond_signal(vp_os_cond_t *cond)
{
 	#if defined USE_WINDOWS_CONDITION_VARIABLES
		WakeConditionVariable(&cond->cond);
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_cond_signal(&cond->cond);
	#endif
}


void
vp_os_cond_broadcast(vp_os_cond_t *cond)
{
	#if defined USE_WINDOWS_CONDITION_VARIABLES
		/**/
	#elif defined USE_PTHREAD_FOR_WIN32
		pthread_cond_broadcast(&cond->cond);
	#endif
	
}

