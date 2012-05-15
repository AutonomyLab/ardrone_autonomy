#include <VP_Os/vp_os_malloc.h>

#include <ardrone_tool/ardrone_time.h>

#ifndef _WIN32
	#include <sys/time.h>
#else
 
 #include <sys/timeb.h>
 

 static int gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#endif

C_RESULT ardrone_timer_reset(ardrone_timer_t* timer)
{
  vp_os_memset(timer, 0, sizeof(ardrone_timer_t));

  return C_OK;
}

C_RESULT ardrone_timer_update(ardrone_timer_t* timer)
{
  if( timer->init == FALSE )
  {
    gettimeofday(&timer->tv_init, NULL);
    timer->init = TRUE;
  }

  gettimeofday(&timer->tv, NULL);
  timer->tv.tv_sec   -= timer->tv_init.tv_sec;
  timer->tv.tv_usec  -= timer->tv_init.tv_usec;

  return C_OK;
}

int32_t ardrone_timer_elapsed_ms(ardrone_timer_t* timer)
{
  int32_t time;

  time = -1;

  if( timer->init )
  {
    float32_t t, s, us;

    s   = (float32_t)timer->tv.tv_sec;
    us  = (float32_t)timer->tv.tv_usec;

    t   = (s * 1000.0f) + ( us / 1000.0f);

    time = (int32_t)t;
  }

  return (int32_t)time;
}

int32_t ardrone_timer_elapsed_us(ardrone_timer_t* timer)
{
  int32_t time;

  time = -1;

  if( timer->init )
  {
    float32_t t, s, us;

    s   = (float32_t)timer->tv.tv_sec;
    us  = (float32_t)timer->tv.tv_usec;

    t   = (s * 1000.0f * 1000.0f) + us;

    time = (int32_t)t;
  }

  return time;
}

int32_t ardrone_timer_delta_ms(ardrone_timer_t* timer)
{
  ardrone_timer_t timer_current;

  timer_current.init = TRUE;
  timer_current.tv_init = timer->tv_init;

  ardrone_timer_update(&timer_current);

  return ardrone_timer_elapsed_ms(&timer_current) - ardrone_timer_elapsed_ms(timer);
}

int32_t ardrone_timer_delta_us(ardrone_timer_t* timer)
{
	ardrone_timer_t timer_current;
	
	timer_current.init = TRUE;
	timer_current.tv_init = timer->tv_init;
	
	ardrone_timer_update(&timer_current);
	
	return ardrone_timer_elapsed_us(&timer_current) - ardrone_timer_elapsed_us(timer);
}
