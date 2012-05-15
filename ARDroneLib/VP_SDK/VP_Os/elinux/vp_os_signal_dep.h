/**
 * @file vp_os_signal_dep.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _SIGNAL_INCLUDE_OS_DEP_
#define _SIGNAL_INCLUDE_OS_DEP_


#include <pthread.h>


typedef pthread_mutex_t vp_os_mutex_t;

typedef struct _vp_os_cond_t_
{
  pthread_cond_t  cond;
  vp_os_mutex_t     *mutex;
}
vp_os_cond_t;


#endif // ! _SIGNAL_INCLUDE_OS_DEP_

