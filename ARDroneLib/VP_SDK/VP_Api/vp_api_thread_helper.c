/**
 *  \brief    VP Api. Thread utility.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 19/12/2006
 *  \date     modification  26/03/2007
 */

#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_print.h>
#include <string.h>

int32_t vp_api_get_thread_idx_tab_by_name(thread_table_entry_t* tab, const char* name)
{
  int32_t i = 0;
  while(tab[i].name)
  {
    if(strcmp(name,tab[i].name) == 0)
      return i;
    i++;
  }
  PRINT("Thread %s is not in thread_table_entry_t \n", name);
  return -1;
}

int32_t vp_api_get_thread_idx_tab_by_handle(thread_table_entry_t* tab, THREAD_HANDLE handle, int32_t ret_on_failure)
{
  int32_t i = 0;

  while(tab[i].name && tab[i].handle != handle) i++;

  return tab[i].name ? i : ret_on_failure;
}

C_RESULT vp_api_start_thread_tab(thread_table_entry_t* tab,int32_t idx,THREAD_PARAMS parameters)
{
  if(idx < 0)
    return C_FAIL;

  PRINT("Starting thread %s\n", tab[idx].name);

  if( parameters == THREAD_NO_PARAM )
    parameters = tab[idx].parameters;

  vp_os_thread_create(tab[idx].routine,
                      parameters,
                      &tab[idx].handle,
                      tab[idx].priority,
                      tab[idx].name,
                      (void*)tab[idx].stack,
                      tab[idx].stackSize,
                      &tab[idx].thread );

  return C_OK;
}

C_RESULT vp_api_join_thread_tab(thread_table_entry_t* tab,int32_t idx)
{
  if(idx < 0)
    return C_FAIL;

  if( tab[idx].handle != 0 )
  {
    vp_os_thread_join(tab[idx].handle);
  }

  return C_OK;
}

C_RESULT vp_api_start_all_threads_tab(thread_table_entry_t* tab)
{
  int32_t i = 0;
  while(tab[i].name)
  {
    if(VP_FAILED(vp_api_start_thread_tab(tab,i,0)))
      PRINT("Thread %d refused to start\n",(int)i);
    i++;
  }

  return C_OK;
}

C_RESULT  vp_api_change_thread_prio_tab(thread_table_entry_t* tab, int32_t idx, int32_t priority)
{
  if(idx < 0)
    return C_FAIL;

  if( tab[idx].handle != 0 )
  {
    vp_os_thread_priority( tab[idx].handle, priority );
  }

  return C_OK;
}

C_RESULT  vp_api_resume_thread(thread_table_entry_t* tab, int32_t idx)
{
  if( idx < 0 )
    return C_FAIL;

  if( tab[idx].handle != 0 )
  {
    vp_os_thread_resume( tab[idx].handle );
  }

  return C_OK;
}

C_RESULT  vp_api_suspend_thread(thread_table_entry_t* tab, int32_t idx)
{
  if( idx < 0 )
    return C_FAIL;

  if( tab[idx].handle != 0 )
  {
    vp_os_thread_suspend( tab[idx].handle );
  }

  return C_OK;
}

THREAD_HANDLE vp_api_get_thread_handle(thread_table_entry_t* tab, int32_t idx)
{
  if( idx < 0 )
    return 0;

  return tab[idx].handle;
}

C_RESULT  vp_api_suspend_all_threads_tab(thread_table_entry_t* tab, THREAD_HANDLE handle)
{
  int32_t i = 0;
  while(tab[i].name)
  {
    if( handle != tab[i].handle && tab[i].handle != 0 )
      vp_os_thread_suspend(tab[i].handle);

    i++;
  }

  return C_OK;
}
