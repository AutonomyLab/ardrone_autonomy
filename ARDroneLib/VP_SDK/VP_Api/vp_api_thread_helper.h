/**
 *  \brief    VP Api. Thread utility.
 *  \brief    These macros are there to help static declaration of threads and their management.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 19/12/2006
 *  \date     modification  26/03/2007
 */

#ifndef _THREAD_HELPER_H_
#define _THREAD_HELPER_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_thread.h>


#ifdef __cplusplus
extern "C"
{
#endif


# define DEFINE_CYG_THREAD void* thread;
# define DECLARE_CYG_THREAD 0
# define STACK_ALIGNMENT

#define BEGIN_THREAD_TABLE  \
  thread_table_entry_t threadTable[] = {

#define END_THREAD_TABLE    \
    { 0 }                   \
  };

#define THREAD_TABLE_ENTRY(name, priority) \
  { #name, priority, 0, 0, THREAD_NO_PARAM, thread_##name, 0, DECLARE_CYG_THREAD },

#define THREAD_TABLE_ENTRY_STACK(name, priority) \
  { #name, priority, sizeof stack_##name, stack_##name, THREAD_NO_PARAM, thread_##name, 0, DECLARE_CYG_THREAD },

#define THREAD_TABLE_ENTRY_STACK_PARAM(name, priority, params) \
  { #name, priority, sizeof stack_##name, stack_##name, params, thread_##name, 0, DECLARE_CYG_THREAD },

#define DEFINE_THREAD_ROUTINE_STACK(name,params,stackSize) \
  int8_t stack_##name[stackSize] STACK_ALIGNMENT;          \
  DEFINE_THREAD_ROUTINE(name,params)

#define PROTO_THREAD_ROUTINE(name,params)    \
  DEFINE_THREAD_ROUTINE(name,params)

#define PROTO_THREAD_ROUTINE_STACK(name,params,stackSize)    \
  extern int8_t stack_##name[stackSize];                     \
  DEFINE_THREAD_ROUTINE(name,params)

#define start_all_threads()                 vp_api_start_all_threads_tab(threadTable)
#define get_thread_idx(name)                vp_api_get_thread_idx_tab_by_name(threadTable, name)
#define start_thread(idx, param)            vp_api_start_thread_tab(threadTable, idx, param)
#define join_thread(idx)                    vp_api_join_thread_tab(threadTable, idx)
#define change_thread_prio(idx, priority)   vp_api_change_thread_prio_tab(threadTable, idx, priority)
#define resume_thread(idx)                  vp_api_resume_thread(threadTable, idx)
#define suspend_thread(idx)                 vp_api_suspend_thread(threadTable, idx)
#define suspend_all_threads(h)              vp_api_suspend_all_threads_tab(threadTable, h)
#define get_thread_handle(idx)              vp_api_get_thread_handle(threadTable, idx)

#define START_THREAD(name,param) start_thread(get_thread_idx(#name), param)

#define JOIN_THREAD(name) join_thread(get_thread_idx(#name))

#define CHANGE_THREAD_PRIO(name, priority) change_thread_prio(get_thread_idx(#name), priority)

#define GET_THREAD_HANDLE(name) get_thread_handle(get_thread_idx(#name))

#define RESUME_THREAD(name) resume_thread(get_thread_idx(#name))

#define SUSPEND_THREAD(name) suspend_thread(get_thread_idx(#name))

#define THREAD_NO_PARAM        (0)

typedef struct
{
  char*           name;
  int32_t         priority;
  int32_t         stackSize;
  int8_t*         stack;
  THREAD_PARAMS   parameters;
  THREAD_ROUTINE  routine;
  THREAD_HANDLE   handle;
  DEFINE_CYG_THREAD
} thread_table_entry_t;

extern thread_table_entry_t threadTable[];

C_RESULT  vp_api_start_all_threads_tab(thread_table_entry_t* tab);
int32_t   vp_api_get_thread_idx_tab_by_name(thread_table_entry_t* tab, const char* name);
int32_t   vp_api_get_thread_idx_tab_by_handle(thread_table_entry_t* tab, THREAD_HANDLE handle, int32_t ret_on_failure);
C_RESULT  vp_api_start_thread_tab(thread_table_entry_t* tab, int32_t idx, THREAD_PARAMS parameters);
C_RESULT  vp_api_join_thread_tab(thread_table_entry_t* tab, int32_t idx);
C_RESULT  vp_api_change_thread_prio_tab(thread_table_entry_t* tab, int32_t idx, int32_t priority);
C_RESULT  vp_api_resume_thread(thread_table_entry_t* tab, int32_t idx);
C_RESULT  vp_api_suspend_thread(thread_table_entry_t* tab, int32_t idx);
THREAD_HANDLE vp_api_get_thread_handle(thread_table_entry_t* tab, int32_t idx);

// Suspend all threads but the one passed as second parameters (if it's an application thread)
C_RESULT  vp_api_suspend_all_threads_tab(thread_table_entry_t* tab, THREAD_HANDLE handle);

#ifdef __cplusplus
}
#endif

#endif // _THREAD_HELPER_H_
