/**
 * @file ATcodec_Sorted_List.h
 * @author aurelien.morelle@parrot.fr
 * @date 2007/01/30
 */

#ifndef _AT_CODEC_SORTED_LIST_INCLUDE_
#define _AT_CODEC_SORTED_LIST_INCLUDE_

typedef struct _ATcodec_Sorted_List_
{
	void *head;

	int nb;

	size_t size;
}
ATcodec_Sorted_List_t;


typedef void (*ATcodec_element_processing)(void *element);


void
ATcodec_Sorted_List_init (ATcodec_Sorted_List_t *list, size_t size);

void
ATcodec_Sorted_List_destroy (ATcodec_Sorted_List_t *list);


void *
ATcodec_Sorted_List_headElement (ATcodec_Sorted_List_t *list);

void *
ATcodec_Sorted_List_tailElement (ATcodec_Sorted_List_t *list);

void *
ATcodec_Sorted_List_getElement (ATcodec_Sorted_List_t *list, unsigned int index);


void
ATcodec_Sorted_List_insertElement (ATcodec_Sorted_List_t *list, const void *element, int sortValue);

void
ATcodec_Sorted_List_removeElement (ATcodec_Sorted_List_t *list, void *element);


void
ATcodec_Sorted_List_batchProcess (ATcodec_Sorted_List_t *list, ATcodec_element_processing process_func);


#endif // ! _AT_CODEC_SORTED_LIST_INCLUDE_
