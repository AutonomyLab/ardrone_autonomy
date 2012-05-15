/**
 * @file malloc.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/19
 */

#ifndef _MALLOC_INCLUDE_OS_
#define _MALLOC_INCLUDE_OS_

#include <assert.h>
#include <string.h>
#include <stdlib.h>


#include <VP_Os/vp_os_types.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Same as calloc, see <stdlib.h>
 */
void *
vp_os_calloc(size_t nmemb, size_t size);

/**
 * Same as malloc, see <stdlib.h>
 */
void *
vp_os_malloc(size_t size);

/**
 * Same as vp_os_malloc, but without assert even if debug mode
 */
void *
vp_os_malloc_no_assert(size_t size);

/**
 * Same as free, see <stdlib.h>
 */
void
vp_os_free(void *ptr);
void
vp_os_sfree(void **ptr);
/**
 * Same as realloc, see <stdlib.h>
 */
void *
vp_os_realloc(void *ptr, size_t size);

/**
 * Same as vp_os_malloc, but returned value is aligned on align_size boundaries
 *  -> align_size has to be a power of two !!!
 */
void*
vp_os_aligned_malloc(size_t size, size_t align_size);


/**
 * Must be used to free memory allocated by vp_os_aligned_malloc correctly
 */
void
vp_os_aligned_free(void *ptr);

/**
 * Allow reallocation of aligned data
 *  -> align_size has to be a power of two !!!
 */
void*
vp_os_aligned_realloc(void* ptr, size_t size, size_t align_size);

static inline void *
vp_os_memset(void *s, int c, size_t n)
{
#ifdef DEBUG_MODE
  void *res;
  assert(s);
  res = memset(s, c, n);
  assert(res == s);
  return (res);
#else // ! DEBUG_MODE
  return memset(s, c, n);
#endif // <- DEBUG_MODE
}


static inline void *
vp_os_memcpy(void *dest, const void *src, size_t n)
{
#ifdef DEBUG_MODE
  void *res;
  assert(dest);
  res = memcpy(dest, src, n);
  assert(res == dest);
  return (res);
#else // ! DEBUG_MODE
  return memcpy(dest, src, n);
#endif // <- DEBUG_MODE
}


extern void *please_use_vp_os_calloc(size_t nmemb, size_t size);
extern void *please_use_vp_os_malloc(size_t size);
extern void  please_use_vp_os_free(void *ptr);
extern void *please_use_vp_os_realloc(void *ptr, size_t size);
extern void *please_use_vp_os_memset(void *s, int c, size_t n);

#undef calloc
#undef malloc
#undef memset
#undef free
#undef realloc

#define calloc  please_use_vp_os_calloc
#define malloc  please_use_vp_os_malloc
#define memset  please_use_vp_os_memset
#define free    please_use_vp_os_free
#define realloc please_use_vp_os_realloc

#ifdef __cplusplus
}
#endif


#endif // ! _MALLOC_INCLUDE_OS_

