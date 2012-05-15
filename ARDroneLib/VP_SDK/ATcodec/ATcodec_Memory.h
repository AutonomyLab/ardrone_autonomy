/**
 * @file ATcodec_Memory.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */

#ifndef _AT_CODEC_MEMORY_INCLUDE_
#define _AT_CODEC_MEMORY_INCLUDE_


typedef  int (*ATcodec_Getchar)(char **str); 
typedef void (*ATcodec_Putchar)(char **str, int c); 


typedef struct _ATcodec_Memory_s_
{
  const char *start;
  int size;
  int char_size;

  char *current;

  ATcodec_Getchar get;
  ATcodec_Putchar put;
}
ATcodec_Memory_t;


// Init

void
ATcodec_Memory_Init        (ATcodec_Memory_t *mem, const char *start, int size, int char_size, ATcodec_Getchar get, ATcodec_Putchar put);


// Put

void
ATcodec_Memory_Put_Char    (ATcodec_Memory_t *mem, int character);

void
ATcodec_Memory_Put_String  (ATcodec_Memory_t *mem, const char *str);

void
ATcodec_Memory_Raw_Put_Int (ATcodec_Memory_t *mem, int value);

void
ATcodec_Memory_Str_Put_Int (ATcodec_Memory_t *mem, int value);


// Get

int
ATcodec_Memory_Get_Char    (ATcodec_Memory_t *mem);

void
ATcodec_Memory_Unget_Char  (ATcodec_Memory_t *mem);

void
ATcodec_Memory_Get_String  (ATcodec_Memory_t *mem, char *dst);

int
ATcodec_Memory_Raw_Get_Int (ATcodec_Memory_t *mem);

int
ATcodec_Memory_Str_Get_Int (ATcodec_Memory_t *mem);


#endif // ! _AT_CODEC_MEMORY_INCLUDE_
