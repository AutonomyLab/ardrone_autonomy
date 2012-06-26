/**
 * @file ATcodec_Buffer.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_malloc.h>

#include <ATcodec/ATcodec_Buffer.h>




void
ATcodec_Buffer_init (ATcodec_Buffer_t *s, size_t elementSize, int nbElementsStart)
{
  s->totalSize = elementSize*nbElementsStart;
  VP_OS_ASSERT(s->totalSize);

  s->data = vp_os_malloc(s->totalSize);
  VP_OS_ASSERT(s->data);

  s->topElement = NULL;
  s->nbElements = 0;   /* Buffer is actually emply. nbElementsStart refers to allocated memory size.*/
  s->elementSize = elementSize;
  
}

void
ATcodec_Buffer_popElement (ATcodec_Buffer_t *s, void *dest)
{
  VP_OS_ASSERT(s->nbElements);

  vp_os_memcpy(dest, s->topElement, s->elementSize);

  if (!--s->nbElements)
    {
      s->topElement = NULL;
    }
  else
    {
      s->topElement = (void *)((char *)s->topElement-s->elementSize);
    }
}

void
ATcodec_Buffer_justPopElement (ATcodec_Buffer_t *s)
{
  VP_OS_ASSERT(s->nbElements);

  if (!--s->nbElements)
    {
      s->topElement = NULL;
    }
  else
    {
      s->topElement = (void *)((char *)s->topElement-s->elementSize);
    }
}

void
ATcodec_Buffer_pushElement (ATcodec_Buffer_t *s, const void *element)
{
  void *oldPtr;

  if (s->nbElements*s->elementSize >= s->totalSize)
    {
      oldPtr = s->data;

      s->totalSize <<= 1;
      s->data = vp_os_realloc(s->data, s->totalSize);

      if (s->data != oldPtr)
	s->topElement = (void *)(((char*)s->topElement-(char*)oldPtr)+(char*)s->data);
    }

  if (!s->nbElements++)
    {
      s->topElement = s->data;
    }
  else
    {
      s->topElement = (void *)((char *)s->topElement+s->elementSize);
    }

  vp_os_memcpy(s->topElement, element, s->elementSize);
}

void
ATcodec_Buffer_pushElements (ATcodec_Buffer_t *s, const void *elements, int nb)
{
  void *oldPtr;
  VP_OS_ASSERT(nb>=0);

  while ((s->nbElements+nb-1)*s->elementSize >= s->totalSize)
    {
      oldPtr = s->data;

      s->totalSize <<= 1;
      s->data = vp_os_realloc(s->data, s->totalSize);

      if (s->data != oldPtr)
	//s->topElement = (void *)(((int)s->topElement-(int)oldPtr)+(int)s->data);
	  s->topElement = (void *)((char*)s->topElement - (char*)oldPtr + (char*)s->data);
    }

  if (!s->nbElements)
    {
      s->topElement = (char *)s->data+(nb-1)*s->elementSize;
      vp_os_memcpy(s->data, elements, nb*s->elementSize);
    }
  else
    {
      vp_os_memcpy((char *)s->topElement+s->elementSize, elements, nb*s->elementSize);
      s->topElement = (void *)((char *)s->topElement+nb*s->elementSize);
    }

  s->nbElements += nb;
}

void *
ATcodec_Buffer_getElement (ATcodec_Buffer_t *s, int index)
{
  VP_OS_ASSERT(index>=0 && index < s->nbElements);

  return (void *)((char*)s->data+index*s->elementSize);
}

void *
ATcodec_Buffer_topElement (ATcodec_Buffer_t *s)
{
  return s->topElement;
}

void
ATcodec_Buffer_destroy (ATcodec_Buffer_t *s)
{
  vp_os_free(s->data);

  s->data = NULL;
  s->elementSize = 0;
  s->nbElements = 0;
  s->topElement = NULL;
  s->totalSize = 0;
}
