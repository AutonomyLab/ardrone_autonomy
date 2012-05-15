#ifndef _ARDRONE_TIME_H_
#define _ARDRONE_TIME_H_

#ifndef _WIN32
	#include <sys/time.h>
#endif

#include <VP_Os/vp_os_types.h>

typedef struct _ardrone_timer_t
{
  bool_t  init;
  struct timeval  tv_init;
  struct timeval  tv;
} ardrone_timer_t;

C_RESULT ardrone_timer_reset(ardrone_timer_t* timer);
C_RESULT ardrone_timer_update(ardrone_timer_t* timer);

int32_t ardrone_timer_elapsed_ms(ardrone_timer_t* timer);
int32_t ardrone_timer_elapsed_us(ardrone_timer_t* timer);
int32_t ardrone_timer_delta_ms(ardrone_timer_t* timer);
int32_t ardrone_timer_delta_us(ardrone_timer_t* timer);

#endif // _ARDRONE_TIME_H_
