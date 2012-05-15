#include <VP_Os/vp_os_signal.h>

void vp_os_mutex_init(vp_os_mutex_t *mutex)
{
  sup_mutex_init(mutex, FALSE);
}


void vp_os_mutex_destroy(vp_os_mutex_t *mutex)
{
  sup_mutex_destroy(mutex);
}


void vp_os_mutex_lock(vp_os_mutex_t *mutex)
{
  sup_mutex_lock(mutex);
}

C_RESULT vp_os_mutex_trylock(vp_os_mutex_t *mutex)
{
  return sup_mutex_trylock(mutex) ? C_FAIL : C_OK;
}


void vp_os_mutex_unlock(vp_os_mutex_t *mutex)
{
  sup_mutex_unlock(mutex);
}


void vp_os_cond_init(vp_os_cond_t *cond, vp_os_mutex_t *mutex)
{
  sup_cond_init(cond, mutex);
}


void vp_os_cond_destroy(vp_os_cond_t *cond)
{
  sup_cond_destroy(cond);
}


void vp_os_cond_wait(vp_os_cond_t *cond)
{
  sup_cond_wait(cond);
}


C_RESULT vp_os_cond_timed_wait(vp_os_cond_t *cond, uint32_t ms)
{
  return sup_cond_timedwait(cond, sup_time_current()+((ms/10)*WAIT10MS)) == 0 ? C_OK : C_FAIL;
}


void vp_os_cond_signal(vp_os_cond_t *cond)
{
  sup_cond_signal(cond);
}

