/**
 * @file ATcodec_Sorted_List.c
 * @author aurelien.morelle@parrot.fr
 * @date 2007/01/30
 */
#include "VP_Os/vp_os_assert.h"
#include "VP_Os/vp_os_malloc.h"
#include "VP_Os/vp_os_types.h"

#include "ATcodec_Sorted_List.h"


#define ATCODEC_MAGIC_NUMBER 0xA7C00DEC


typedef struct _ATcodec_Sorted_List_header_
{
  uint32_t magic;

  struct _ATcodec_Sorted_List_header_ *next;
  struct _ATcodec_Sorted_List_header_ *previous;
  uint32_t sortValue;

  void *element;
}
ATcodec_Sorted_List_header_t;


#define ATCODEC_SYSTEM_PTR(ELEMENT) \
        ((ATcodec_Sorted_List_header_t *)(((int)(ELEMENT))-(int)&((ATcodec_Sorted_List_header_t *)NULL)->element))

void
ATcodec_Sorted_List_init (ATcodec_Sorted_List_t *list, size_t size)
{
  list->head = NULL;
  list->nb = 0;
  list->size = size;
}

void
ATcodec_Sorted_List_batchProcess (ATcodec_Sorted_List_t *list, ATcodec_element_processing process_func)
{
  ATcodec_Sorted_List_header_t *current = (ATcodec_Sorted_List_header_t *)list->head;
  ATcodec_Sorted_List_header_t *next;

  while(current)
    {
      next = current->next;
      process_func(&current->element);
      current = next;
    }
}

static void
ATcodec_Sorted_List_freeElement(void *element)
{
  vp_os_free(ATCODEC_SYSTEM_PTR(element));
}

void
ATcodec_Sorted_List_destroy (ATcodec_Sorted_List_t *list)
{
  ATcodec_Sorted_List_batchProcess(list, ATcodec_Sorted_List_freeElement);
}

void *
ATcodec_Sorted_List_headElement (ATcodec_Sorted_List_t *list)
{
  return &((ATcodec_Sorted_List_header_t *)list->head)->element;
}

void *
ATcodec_Sorted_List_nextElement (ATcodec_Sorted_List_t *list, void *element)
{
  ATcodec_Sorted_List_header_t *system_ptr = ATCODEC_SYSTEM_PTR(element);
  return &system_ptr->next->element;
}

void *
ATcodec_Sorted_List_previousElement (ATcodec_Sorted_List_t *list, void *element)
{
  ATcodec_Sorted_List_header_t *system_ptr = ATCODEC_SYSTEM_PTR(element);
  return &system_ptr->previous->element;
}

void *
ATcodec_Sorted_List_getElement (ATcodec_Sorted_List_t *list, unsigned int index)
{
  ATcodec_Sorted_List_header_t *current = (ATcodec_Sorted_List_header_t *)list->head;

  while(current && index--)
    {
      current = current->next;
    }

  return (current ? &current->element : NULL);
}

void
ATcodec_Sorted_List_removeElement (ATcodec_Sorted_List_t *list, void *element)
{
  ATcodec_Sorted_List_header_t *system_ptr;
  ATcodec_Sorted_List_header_t *next;
  ATcodec_Sorted_List_header_t *previous;

  VP_OS_ASSERT(element);

  system_ptr = ATCODEC_SYSTEM_PTR(element);

  VP_OS_ASSERT(system_ptr);
  VP_OS_ASSERT(system_ptr->magic == ATCODEC_MAGIC_NUMBER);

  next = system_ptr->next;
  previous = system_ptr->previous;

  if(previous)
    {
      previous->next = next;
    }
  else
    {
      list->head = next;
    }

  if(next)
    next->previous = previous;

  vp_os_free(system_ptr);

  list->nb--;
}

void
ATcodec_Sorted_List_insertElement (ATcodec_Sorted_List_t *list, const void *element, int sortValue)
{
  ATcodec_Sorted_List_header_t *current = (ATcodec_Sorted_List_header_t *)list->head;
  ATcodec_Sorted_List_header_t *previous = NULL;
  ATcodec_Sorted_List_header_t *ptr = (ATcodec_Sorted_List_header_t *)vp_os_malloc(list->size+sizeof(ATcodec_Sorted_List_header_t)-sizeof(void *));

  ptr->magic = ATCODEC_MAGIC_NUMBER;
  ptr->sortValue = sortValue;
  memcpy(&ptr->element, element, list->size);

  while(current && sortValue >= (int32_t)current->sortValue)
    {
      previous = current;
      current = current->next;
    }

  if(current && sortValue < (int32_t)current->sortValue)
    {
      if(current->previous)
	{
	  current->previous->next = ptr;
	}
      else
	{
	  list->head = ptr;
	}

      ptr->previous = current->previous;
      ptr->next = current;
      current->previous = ptr;
    }
  else
    {
      if(previous)
	{
	  previous->next = ptr;
	  ptr->previous = previous;
	  ptr->next = NULL;
	}
      else
	{
	  list->head = ptr;
	  ptr->previous = NULL;
	  ptr->next = NULL;
	}
    }

  list->nb++;
}
