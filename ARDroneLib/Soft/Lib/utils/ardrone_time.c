#include <VP_Os/vp_os_malloc.h>

#include <utils/ardrone_time.h>

#if defined(_WIN32)
 #include <sys/timeb.h>
 
 static int _gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#elif defined(TARGET_OS_IPHONE) || defined(TARGET_OS_SIMULATOR)
#include <mach/mach_time.h>
static int _gettimeofday (struct timeval *tp, void *tz)
{
    mach_timebase_info_data_t sTimebaseInfo;
    uint64_t mtime = mach_absolute_time();
    uint64_t useconds;
    
    mach_timebase_info(&sTimebaseInfo);
    useconds = (mtime * (sTimebaseInfo.numer / sTimebaseInfo.denom) / 1000);

    tp->tv_sec = (long)(useconds / 1000000);
    tp->tv_usec = (long)(useconds % 1000000);
    
    return 0;
}
#else
#include <sys/time.h>
#define _gettimeofday gettimeofday
#endif

C_RESULT ardrone_timer_reset(ardrone_timer_t* timer)
{
  vp_os_memset(timer, 0, sizeof(ardrone_timer_t));

  return C_OK;
}

C_RESULT ardrone_timer_update(ardrone_timer_t* timer)
{
    C_RESULT retVal = C_OK;
  if( timer->init == FALSE )
  {
    _gettimeofday(&timer->tv_init, NULL);
    timer->init = TRUE;
  }

  _gettimeofday(&timer->tv, NULL);
    if (timer->tv.tv_usec >= timer->tv_init.tv_usec)
    {
        timer->tv.tv_usec -= timer->tv_init.tv_usec;
    }
    else
    {
        timer->tv.tv_usec = 1000000 - timer->tv_init.tv_usec + timer->tv.tv_usec;
        timer->tv.tv_sec -= 1;
    }

    if (timer->tv.tv_sec < timer->tv_init.tv_sec)
    {
        /* The tv timer has a time sooner than the tv_init timer */
        retVal = C_FAIL;
        /* We do the calculation anyway because it still can be used. We'll only have a negative time */
    }
    timer->tv.tv_sec -= timer->tv_init.tv_sec;

  return retVal;
}

uint64_t ardrone_timer_elapsed_ms(ardrone_timer_t* timer)
{
    uint64_t time;

  time = -1;

  if( timer->init )
  {
        time = (1000ll * (uint64_t)timer->tv.tv_sec) + ((uint64_t)timer->tv.tv_usec / 1000ll);
  }

    return time;
}

uint64_t ardrone_timer_elapsed_us(ardrone_timer_t* timer)
{
    uint64_t time;

  time = -1;

  if( timer->init )
  {
        time = (1000000ll * (uint64_t)timer->tv.tv_sec) + ((uint64_t)timer->tv.tv_usec);
  }

  return time;
}

uint64_t ardrone_timer_delta_ms(ardrone_timer_t* timer)
{
  ardrone_timer_t timer_current;

  timer_current.init = TRUE;
  timer_current.tv_init = timer->tv_init;

    uint64_t retVal = 0;
    if (C_OK == ardrone_timer_update(&timer_current))
    {
        retVal = ardrone_timer_elapsed_ms(&timer_current) - ardrone_timer_elapsed_ms(timer);
    }

    return retVal;
}

uint64_t ardrone_timer_delta_us(ardrone_timer_t* timer)
{
	ardrone_timer_t timer_current;
	
	timer_current.init = TRUE;
	timer_current.tv_init = timer->tv_init;
	
    uint64_t retVal = 0;
    if (C_OK == ardrone_timer_update(&timer_current))
    {
        retVal = ardrone_timer_elapsed_us(&timer_current) - ardrone_timer_elapsed_us(timer);
}

    return retVal;
}
