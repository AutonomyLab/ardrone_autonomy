/**
 * @file ATcodec_Memory.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */

#include <VP_Os/vp_os_assert.h>

#include <ATcodec/ATcodec_Memory.h>

// Macros
#define ATCODEC_MEMORY_SIZE_ASSERT(mem, size_inc) \
  do \
  { \
    VP_OS_ASSERT((unsigned)mem->current-(unsigned)mem->start + size_inc*mem->char_size < mem->size); \
  } \
  while(0)

#define ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem) \
  do \
  { \
    VP_OS_ASSERT(mem); \
    VP_OS_ASSERT(mem->start); \
    VP_OS_ASSERT(mem->char_size); \
    VP_OS_ASSERT(mem->get); \
    VP_OS_ASSERT(mem->put); \
  } \
  while(0)

#define ATCODEC_MEMORY_PUT_STRUCT_ASSERT(mem) \
  do \
  { \
    ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem); \
    VP_OS_ASSERT(mem->size); \
  } \
  while(0)

// Static get/put

static int
static_atcodec_getchar (char **str)
{
  VP_OS_ASSERT(str);
  VP_OS_ASSERT(*str);

  return *((*str)++);
}

static void
static_atcodec_putchar (char **str, int c)
{
  VP_OS_ASSERT(str);
  VP_OS_ASSERT(*str);

  *((*str)++) = (char)c;
}

// Init

void
ATcodec_Memory_Init (ATcodec_Memory_t *mem, const char *start, int size, int char_size, ATcodec_Getchar get, ATcodec_Putchar put)
{
  VP_OS_ASSERT(mem);
  VP_OS_ASSERT(start);
  VP_OS_ASSERT(char_size);

  mem->start = start;
  mem->size = size;
  mem->char_size = char_size;

  mem->current = (char *)start;

  mem->get = (get && size ? get : static_atcodec_getchar);
  mem->put = (put ? put : static_atcodec_putchar);
}


// Put

void
ATcodec_Memory_Put_Char (ATcodec_Memory_t *mem, int character)
{
  ATCODEC_MEMORY_PUT_STRUCT_ASSERT(mem);
  ATCODEC_MEMORY_SIZE_ASSERT(mem, 1);

  mem->put(&mem->current, character);
}

// \todo Does source string need to be passed like an ATcodec_Memory_t ?
void
ATcodec_Memory_Put_String (ATcodec_Memory_t *mem, const char *str)
{
  char *s = (char *)str;

  VP_OS_ASSERT(s);
  ATCODEC_MEMORY_PUT_STRUCT_ASSERT(mem);

  if(*s)
  {
    do
    {
      ATCODEC_MEMORY_SIZE_ASSERT(mem, 1);
      mem->put(&mem->current, *s++);
    }
    while ( *s );
  }
}

void
ATcodec_Memory_Raw_Put_Int (ATcodec_Memory_t *mem, int value)
{
  ATCODEC_MEMORY_PUT_STRUCT_ASSERT(mem);

  ATcodec_Memory_Put_Char(mem, (value>>24) & 0xFF);

  value &= 0x00FFFFFF;
  ATcodec_Memory_Put_Char(mem, (value>>16) & 0xFF);

  value &= 0x0000FFFF;
  ATcodec_Memory_Put_Char(mem, (value>>8) & 0xFF);

  value &= 0x000000FF;
  ATcodec_Memory_Put_Char(mem, value & 0xFF);
}

void
ATcodec_Memory_Str_Put_Int (ATcodec_Memory_t *mem, int value)
{
  int i;

  ATCODEC_MEMORY_PUT_STRUCT_ASSERT(mem);

  if (value < 0)
  {
    ATcodec_Memory_Put_Char(mem, '-');
    value = -value;
  }

  for ( i=1 ; value >= 10*i ; i*=10 );

  while (value)
  {
    VP_OS_ASSERT(i);
    ATcodec_Memory_Put_Char(mem, '0'+(char)(value/i));
    value -= i*(value/i);
    i /= 10;
  }

  for ( ; i ; i/=10 )
    ATcodec_Memory_Put_Char(mem, '0');
}

// Get

int
ATcodec_Memory_Get_Char (ATcodec_Memory_t *mem)
{
  ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem);

  return mem->get(&mem->current);
}

void
ATcodec_Memory_Unget_Char (ATcodec_Memory_t *mem)
{
  ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem);

  mem->current -= mem->char_size;
  VP_OS_ASSERT((unsigned)mem->current >= (unsigned)mem->start);
}

// \todo Does destination string need to be passed like an ATcodec_Memory_t ?
void
ATcodec_Memory_Get_String (ATcodec_Memory_t *mem, char *dst)
{
  int c;

  VP_OS_ASSERT(dst);
  ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem);

  do
  {
    c = mem->get(&mem->current);
    *dst++ = c;
  }
  while ( c );
}

int
ATcodec_Memory_Raw_Get_Int (ATcodec_Memory_t *mem)
{
  unsigned val;

  ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem);

  val = (unsigned) ATcodec_Memory_Get_Char(mem);
  val = ((unsigned)(ATcodec_Memory_Get_Char(mem) & 0xFF)) | (val << 8);
  val = ((unsigned)(ATcodec_Memory_Get_Char(mem) & 0xFF)) | (val << 8);
  val = ((unsigned)(ATcodec_Memory_Get_Char(mem) & 0xFF)) | (val << 8);

  return val;
}

int
ATcodec_Memory_Str_Get_Int (ATcodec_Memory_t *mem)
{
  int neg, val;
  char c;

  ATCODEC_MEMORY_GET_STRUCT_ASSERT(mem);

  c = (char) ATcodec_Memory_Get_Char(mem);
  neg = (c == '-' ? 0 : -1);

  if (!neg)
    c = (char) ATcodec_Memory_Get_Char(mem); // skips '-' character

  VP_OS_ASSERT(c >= '0' && c <= '9'); // not supported

  val = 0;
  do
  {
    val = 10*val + c-'0';
    c = (char) ATcodec_Memory_Get_Char(mem); // processes next characters
  }
  while (c >= '0' && c <= '9');

  ATcodec_Memory_Unget_Char(mem); // ungets last non-num character

  if(!neg)
    val = -val;

  return val;
}
