#include <stdarg.h>
#include <malloc.h>

#include <VP_Os/vp_os_thread.h>

typedef struct _pthread_data_t
{
  uint32_t        		 free;       // bit array representing thread array allocation state (0 means free)
  SUP_THREAD 	thread[32]; // thread array
  struct _pthread_data_t* next;
} pthread_data_t;

static pthread_data_t* threadTab = NULL;

static INLINE uint32_t bittest(uint32_t word, uint32_t pos)
{
  return (word & (1 << pos));
}

static INLINE uint32_t bitset(uint32_t word, uint32_t pos)
{
  return word |= (1 << pos);
}

static INLINE uint32_t bitreset(uint32_t word, uint32_t pos)
{
  return word &= ~(1 << pos);
}

static int32_t findFreeIndex(uint32_t word)
{
  int32_t bit;
  int32_t index;

  index = *(int32_t*)&word;

  if( index != -1 )
  {
    index = 0;
    bit = 1;

    while( word & bit )
    {
      index ++;
      bit <<= 1;
    }
  }

  return index;
}

static SUP_THREAD* findFreeSlot(void)
{
  int32_t index;
  pthread_data_t *tab, *prev;

  prev = threadTab;
  tab  = threadTab;

  while( tab != NULL )
  {
    index = findFreeIndex( tab->free );
    if( index >= 0 )
    {
      tab->free = bitset( tab->free, index );
      return &tab->thread[index];
    }

    prev = tab;
    tab = tab->next;
  }

  // If we are here it means we lack free memory

  if(prev)
  {
	  prev->next = (pthread_data_t*)calloc( 1, sizeof(pthread_data_t) );
	  tab = prev->next;
  }
  else
  {
	  tab = (pthread_data_t*)calloc( 1, sizeof(pthread_data_t) );
	  threadTab=tab;
  }


  tab->free = bitset( tab->free, 0 );
  return &tab->thread[0];
}

void vp_os_thread_create(THREAD_ROUTINE entry, THREAD_PARAMS data, THREAD_HANDLE *handle, ...)
{
  int32_t priority;
  char* name;
  void* stack_base;
  unsigned int stack_size;
  va_list va;


  va_start(va, (char*)handle);
  priority    = va_arg(va, int32_t);
  name        = va_arg(va, char *);
  stack_base  = va_arg(va, void *);
  stack_size  = va_arg(va, unsigned int);
  va_end(va);

  SUP_THREAD* thread = findFreeSlot();

  sup_thread_create(handle, thread, priority, entry, data, stack_size, name);
  sup_thread_resume(*handle);
}

void vp_os_thread_join(THREAD_HANDLE handle)
{
}

THREAD_HANDLE vp_os_thread_self(void)
{
  return sup_thread_current();
}

void vp_os_thread_suspend(THREAD_HANDLE handle)
{
  sup_thread_suspend(handle);
}

void vp_os_thread_resume(THREAD_HANDLE handle)
{
  sup_thread_resume(handle);
}

void vp_os_thread_yield(void)
{
  sup_thread_yield();
}

void vp_os_thread_priority(THREAD_HANDLE handle, int32_t priority)
{
  sup_thread_setpriority(handle, priority);
}
