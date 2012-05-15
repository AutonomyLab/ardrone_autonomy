
#include <stdio.h>

#include "VP_Os/vp_os_types.h"
#include "ATcodec/ATcodec_Sorted_List.h"

void print_element(void *element)
{
  printf("%d, ", *(uint32_t *)element);
}

void print_list(ATcodec_Sorted_List_t *list)
{
  printf("{ ");
  ATcodec_Sorted_List_batchProcess(list, print_element);
  printf("}\n");
}

#define INSERT(VALUE) \
  element = VALUE; \
  ATcodec_Sorted_List_insertElement(&list, &element, element); \
  print_list(&list)

#define REMOVE(INDEX) \
  ATcodec_Sorted_List_removeElement(&list, ATcodec_Sorted_List_getElement(&list, INDEX)); \
  print_list(&list)

int main(int argc, char **argv)
{
  ATcodec_Sorted_List_t list;
  int element;

  ATcodec_Sorted_List_init(&list, sizeof(int));
  print_list(&list);

  INSERT(10);
  INSERT(5);
  INSERT(7);
  INSERT(12);
  INSERT(1);

  REMOVE(3);

  INSERT(12);

  REMOVE(0);
  REMOVE(0);

  INSERT(3);
  INSERT(4);
  INSERT(6);

  REMOVE(0);
  REMOVE(0);
  REMOVE(0);
  REMOVE(0);
  REMOVE(0);
  REMOVE(0);

  INSERT(7);
  INSERT(12);
  INSERT(1);

  REMOVE(1);
  REMOVE(1);
  REMOVE(0);

  ATcodec_Sorted_List_destroy(&list);

  return EXIT_SUCCESS;
}
