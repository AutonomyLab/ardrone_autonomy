#include <Maths/time.h>

#ifndef _WIN32
	#include <sys/time.h>
#else
 
 #include <sys/timeb.h>
 //#include <Winsock2.h>  // dont include; already included and double inclusion is buggy

 int gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#endif

float32_t time_in_ms_f(void)
{
  float32_t time_milli_sec, f_sec, f_usec;

  struct timeval  tv;

  gettimeofday(&tv, NULL);

  f_sec   = (float32_t)tv.tv_sec;
  f_usec  = (float32_t)tv.tv_usec;

  time_milli_sec = (float32_t)(1000.0f*f_sec + f_usec/1000.0f);

  return time_milli_sec;
}
