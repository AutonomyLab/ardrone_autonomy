/**
 * @file malloc.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/19
 */

#include "VP_Os/vp_os_malloc.h"


#undef calloc
#undef malloc
#undef memset
#undef free
#undef realloc


void *
vp_os_calloc(size_t nmemb, size_t size)
{
#ifdef DEBUG_MODE
  void *res = calloc(nmemb, size);
  assert(res);
  return (res);
#else // ! DEBUG_MODE
  return calloc(nmemb, size);
#endif // <- DEBUG_MODE
}

void *
vp_os_malloc(size_t size)
{
#ifdef DEBUG_MODE
  void *res = malloc(size);
  assert(res);
  return (res);
#else // ! DEBUG_MODE
  return malloc(size);
#endif // <- DEBUG_MODE
}

void *
vp_os_malloc_no_assert(size_t size)
{
  return malloc(size);
}

void
vp_os_free(void *ptr)
{
#ifdef DEBUG_MODE
  assert(ptr);
  free(ptr);
#else // ! DEBUG_MODE
  free(ptr);
#endif // <- DEBUG_MODE
}

void
vp_os_sfree(void **ptr)
{
#ifdef DEBUG_MODE
  assert(*ptr);
  free(*ptr);
#else // ! DEBUG_MODE
  free(*ptr);
#endif // <- DEBUG_MODE
  *ptr=NULL;
}

// align_size has to be a power of two !!!
//
// The basic working of this algorithm is to allocate a bigger chunk of data than requested.
// This chunk of data must be big enough to contain an address aligned on requested boundary
// We also alloc 2 more words to keep base ptr (bptr) & requested size (size) of allocation
// bptr is the base pointer of this allocation
// _____ ______ ______ __________
//  ... | bptr | size |   ....   |
// _____|______|______|__________|
// 
void* vp_os_aligned_malloc(size_t size, size_t align_size)
{
  char *ptr, *aligned_ptr;
  int* ptr2;
  int allocation_size;
  size_t align_mask = align_size - 1;

  // Check if align_size is a power of two
  // If the result of this test is non zero then align_size is not a power of two
  if( align_size & align_mask )
    return NULL;

  // Allocation size is :
  //    - Requested user size
  //    - a size (align_size) to make sure we can align on the requested boundary
  //    - 8 more bytes to register base adress & allocation size 
  allocation_size = size + align_size + 2*sizeof(int);

  ptr = (char*) vp_os_malloc(allocation_size);
  if(ptr == NULL)
    return NULL;

  ptr2 = (int*)(ptr + 2*sizeof(int));
  aligned_ptr = ptr + 2*sizeof(int) + (align_size - ((size_t) ptr2 & align_mask));

  ptr2    = (int*)(aligned_ptr - 2*sizeof(int));
  *ptr2++ = (int) (aligned_ptr - ptr);
  *ptr2   = size;

  return aligned_ptr;
}

void vp_os_aligned_free(void *ptr)
{
  int* ptr2 = (int*)ptr - 2;

  vp_os_free( ((char*)ptr - *ptr2) );
}

void*
vp_os_aligned_realloc(void* ptr, size_t size, size_t align_size)
{
  void* ptr_ret;
  void* aligned_ptr;

  if( size == 0 )
  {
    ptr_ret = NULL;
    if( ptr != NULL )
      vp_os_aligned_free(ptr);
  }
  else
  {
    if( ptr != NULL )
    {
      int* ptr2 = (int*)ptr - 1;
      size_t old_size;

      aligned_ptr = ptr;

      old_size = *ptr2--;

      ptr_ret = vp_os_aligned_malloc(size, align_size);

      // Compute smallest size
      if( size > old_size )
      {
        size = old_size;
      }

      // Copy old data
      vp_os_memcpy( ptr_ret, aligned_ptr, size );

      vp_os_free( ((char*)ptr - *ptr2) );
    }
    else
    {
      ptr_ret = vp_os_aligned_malloc(size, align_size);
    }
  }

  return ptr_ret;
}

void*
vp_os_realloc(void *ptr, size_t size)
{
#ifdef DEBUG_MODE
  void *res = realloc(ptr, size);
  if (res==NULL) { perror(__FUNCTION__); }
   assert(res!=NULL);
  return (res);
#else // ! DEBUG_MODE
  return realloc(ptr, size);
#endif // <- DEBUG_MODE
}

