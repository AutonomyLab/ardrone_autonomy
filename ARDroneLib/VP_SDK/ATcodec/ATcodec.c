/**
 * @file ATcodec_Buffer.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <ATcodec/ATcodec.h>
#include <ATcodec/ATcodec_Buffer.h>

/************************************************************************
 * For private use in 'vp_atcodec_sscanf' and 'vp_atcodec_sprintf' functions
 ************************************************************************/

#ifdef AT_CODEC_DEBUG

#define PRINTF_DEBUG printf
#define CURCHARS_DEBUG(str_val, fmt_val) ATCODEC_PRINT("Current characters : str_val='%c' ; fmt_val='%c'\n", str_val, fmt_val)
#define PRINT_DEBUG ATCODEC_PRINT("Sortie ligne %i\n", __LINE__)

#else

static inline void tmp_printf(char *str, ...){}

#define PRINTF_DEBUG tmp_printf
#define CURCHARS_DEBUG(str_val, fmt_val)
#define PRINT_DEBUG

#endif

#define EXIT_LOOP_IF(cond) \
  if(cond) \
    { \
      bContinue = ATCODEC_FALSE; \
      PRINT_DEBUG; \
      break; \
    }

typedef struct _ListParams
{
  char *listLengthPtr;
  int list_length;
  char *list_ptr_b;
  char *list_ptr_e;
  char list_char;
}
ListParams;


ATCODEC_RET
vp_atcodec_sscanf(ATcodec_Memory_t *str, ATcodec_Memory_t *fmt, ATcodec_Memory_t *mem, int *len_dec)
{
  int fmt_val,
    str_val,
    tmp_val,
    tmp2_val = 0,
    nbOpen;
  char *stringLengthPtr;

  ATcodec_Buffer_t listsParams;
  ListParams listParams,
    *listParamsPtr;

  char *tmp_ptr,
    *tmp2_ptr = NULL;

  ATCODEC_RET bContinue = ATCODEC_TRUE;

  int inList = -1,
    escaped = -1,
    inFacultative = -1;

  ATcodec_Buffer_init(&listsParams, sizeof(ListParams), 1);

  fmt_val = ATcodec_Memory_Get_Char(fmt);
  str_val = ATcodec_Memory_Get_Char(str);

  while (bContinue == ATCODEC_TRUE && fmt_val != '\0')
    {
      if (fmt_val == '%' && escaped)
	{
	  switch (( fmt_val = ATcodec_Memory_Get_Char(fmt)))
	    {
	    case 'l' :
	      fmt_val = ATcodec_Memory_Get_Char(fmt);

	      EXIT_LOOP_IF(fmt_val != '{');

	      inList = 0;
	      listParams.list_ptr_b = fmt->current;
	      listParams.list_length = 0;

	      listParams.listLengthPtr = mem->current;
	      ATcodec_Memory_Raw_Put_Int(mem, 0xFFFFFFFF);

	      nbOpen = 0;
	      while(fmt_val != '\0' && ((fmt_val = ATcodec_Memory_Get_Char(fmt)) != '}' || !escaped || nbOpen--))
		{
		  if (fmt_val == '\\')
		    {
		      escaped = 0;
		    }
		  else if (fmt_val == '{' && escaped)
		    {
		      nbOpen++;
		    }
		  else
		    {
		      escaped = -1;
		    }
		}

	      EXIT_LOOP_IF(fmt_val == '\0'); // does a '}' lack ?

	      listParams.list_ptr_e = fmt->current;

	      listParams.list_char = ATcodec_Memory_Get_Char(fmt);

	      fmt->current = listParams.list_ptr_b;
	      fmt_val = ATcodec_Memory_Get_Char(fmt);

	      ATcodec_Buffer_pushElement(&listsParams, &listParams);
	      break;

	    case 'd' :
	      EXIT_LOOP_IF((str_val < '0' || str_val > '9') && str_val != '-'); // not supported

	      fmt_val = ATcodec_Memory_Get_Char(fmt);
	      EXIT_LOOP_IF(fmt_val >= '0' && fmt_val <= '9'); // not supported

	      ATcodec_Memory_Unget_Char(str);
	      tmp_val = ATcodec_Memory_Str_Get_Int(str);

	      str_val = ATcodec_Memory_Get_Char(str); // processes next characters
	      ATcodec_Memory_Raw_Put_Int(mem, tmp_val);
	      break;

	    case 's' :
	      fmt_val = ATcodec_Memory_Get_Char(fmt);

	      if (fmt_val == '\\')
		{
		  escaped = 0;
		  fmt_val = ATcodec_Memory_Get_Char(fmt);
		}

	      if (fmt_val == ']' && escaped) // supported
		{
		  fmt_val = ATcodec_Memory_Get_Char(fmt); // just skips this character
		  inFacultative = -1;
		}
	      else
		{
		  EXIT_LOOP_IF
		    (
		     (fmt_val == '[' && escaped) ||  // not supported yet
		     (fmt_val == '%' && escaped) ||  // not supported
		     str_val == fmt_val              // empty string case, prohibited
		     );
		}

	      escaped = -1;
	      stringLengthPtr = mem->current;
	      ATcodec_Memory_Raw_Put_Int(mem, 0xFFFFFFFF);

	      tmp_val = 0;
	      do
		{
		  ATcodec_Memory_Put_Char(mem, str_val);
		  tmp_val++;
		  str_val = ATcodec_Memory_Get_Char(str);
		}
	      while (str_val != fmt_val);
	      ATcodec_Memory_Put_Char(mem, '\0');
	      tmp_ptr = mem->current;
	      mem->current = stringLengthPtr;
	      ATcodec_Memory_Raw_Put_Int(mem, tmp_val);
	      mem->current = tmp_ptr;
	      break;

	    case 'c' :
	      fmt_val = ATcodec_Memory_Get_Char(fmt);
	      /**
	       * To match "%c", <str_val> can be any character excepted <str_lastchar>,
	       * but as we are already in the loop, it is different from <str_lastchar>.
	       * So, we just "eat" one character.
	       **/
	      str_val = ATcodec_Memory_Get_Char(str);
	      ATcodec_Memory_Put_Char(mem, str_val);
	      break;

	    default : // prohibited pattern
	      bContinue = ATCODEC_FALSE; // exits with failure
	      PRINT_DEBUG;
	      break;
	    }
	}
      else if (!inList && fmt_val == '}' && escaped)
	{
	  listParamsPtr = (ListParams *)ATcodec_Buffer_topElement(&listsParams);
	  VP_OS_ASSERT(listParamsPtr);

	  if (str_val == ',') // one list element parsed
	    {
	      str_val = ATcodec_Memory_Get_Char(str);
	      fmt->current = listParamsPtr->list_ptr_b;
	      fmt_val = ATcodec_Memory_Get_Char(fmt);
	      listParamsPtr->list_length++;
	    }
	  else
	    {
	      EXIT_LOOP_IF
		(
		 (listParamsPtr->list_char != '\0') ||
		 (str_val != listParamsPtr->list_char)
		 );

	      // entire list parsed
	      fmt->current = listParamsPtr->list_ptr_e;
	      fmt_val = ATcodec_Memory_Get_Char(fmt);
	      *listParamsPtr->listLengthPtr = ++listParamsPtr->list_length;
	      ATcodec_Buffer_justPopElement(&listsParams);
	      inList = ( ATcodec_Buffer_topElement(&listsParams) ? -1 : 0 );
	    }
	}
      else if (fmt_val == '\\' && escaped)
	{
	  fmt_val = ATcodec_Memory_Get_Char(fmt);
	  escaped = 0;
	}
      else if (fmt_val == '[' && escaped)
	{
	  if ((fmt_val = ATcodec_Memory_Get_Char(fmt)) == '\\')
	    {
	      escaped = 0;
	      fmt_val = ATcodec_Memory_Get_Char(fmt);
	    }

	  if (fmt_val != '%' || !escaped)
	    {
	      escaped = -1;

	      if (str_val == fmt_val)
		{
		  // it musts match the facultative pattern
		  ATcodec_Memory_Raw_Put_Int(mem, (int)0);
		  str_val = ATcodec_Memory_Get_Char(str); // processes next characters
		  fmt_val = ATcodec_Memory_Get_Char(fmt);
		  // nothing else to do, global loop does the job
		  inFacultative = 0;
		}
	      else
		{
		  // we skip the facultative pattern
		  ATcodec_Memory_Raw_Put_Int(mem, (int)-1);

		  while(fmt_val != '\0' && ((fmt_val = ATcodec_Memory_Get_Char(fmt)) != ']' || !escaped))
		    {
		      escaped = ( fmt_val == '\\' ? 0 : -1 );
		    }

		  EXIT_LOOP_IF(fmt_val == '\0'); // does a ']' lack ?

		  fmt_val = ATcodec_Memory_Get_Char(fmt);
		}
	    }
	  else
	    {
	      escaped = -1;

	      // we have to manage a '%' as beginning of the facultative pattern which is too complicated
	      // so, we skip the facultative pattern to test the character that follows it
	      tmp_ptr = fmt->current; // points to the '%' character
	      tmp_val = fmt_val; // is equal to '%'

	      while(fmt_val != '\0' && ((fmt_val = ATcodec_Memory_Get_Char(fmt)) != ']' || !escaped))
		{
		  escaped = ( fmt_val == '\\' ? 0 : -1 );
		}

	      EXIT_LOOP_IF(fmt_val == '\0'); // does a ']' lack ?

	      fmt_val = ATcodec_Memory_Get_Char(fmt);
	      if (fmt_val == '\\')
		{
		  escaped = 0;
		  tmp2_val = fmt_val;
		  tmp2_ptr = fmt->current;
		  fmt_val = ATcodec_Memory_Get_Char(fmt);
		}

	      if (str_val != fmt_val && !(!inList && fmt_val == '}' && str_val == ','))
		{
		  escaped = -1;
		  // we do not really support this case but we try to be lucky ;-)
		  fmt->current = tmp_ptr;
		  fmt_val = tmp_val;
		  // we try to match the facultative pattern
		  ATcodec_Memory_Raw_Put_Int(mem, (int)0);
		  // nothing else to do, global loop does the job
		  inFacultative = 0;
		}
	      else
		{
		  if (!escaped)
		    {
		      fmt_val = tmp2_val;
		      fmt->current = tmp2_ptr;
		      escaped = -1;
		    }
		  // we skip the facultative pattern
		  ATcodec_Memory_Raw_Put_Int(mem, (int)-1);
		}
	    }
	}
      else if (fmt_val == ']' && escaped)
	{
	  fmt_val = ATcodec_Memory_Get_Char(fmt);
	  inFacultative = -1;
	  // nothing to do : global loop does the job
	}
      else if (str_val == fmt_val) // matches
	{
	  // TODO : Verify as much as possible if rules are respected

	  EXIT_LOOP_IF
	    (
	     ((str_val == ',' || str_val == '}') && !inList && escaped) ||
	     ((str_val == '[' || str_val == ']') && !inFacultative && escaped)
	     );

	  str_val = ATcodec_Memory_Get_Char(str); // processes next characters
	  fmt_val = ATcodec_Memory_Get_Char(fmt);
	  escaped = -1;
	}
      else if (fmt_val != '\0') // it does not match
	{
	  bContinue = ATCODEC_FALSE; // exits with failure
	  PRINT_DEBUG;
	  CURCHARS_DEBUG(str_val, fmt_val);
	  break;
	}
    }

  while (bContinue == ATCODEC_TRUE && (fmt_val != '\0'))
    {
      if (fmt_val == ']')
	{
	  fmt_val = ATcodec_Memory_Get_Char(fmt);
	}
      else if (fmt_val == '[')
	{
	  ATcodec_Memory_Raw_Put_Int(mem, (int)-1); // we skip the facultative pattern

	  while(fmt_val != '\0' && ((fmt_val = ATcodec_Memory_Get_Char(fmt)) != ']' || !escaped))
	    {
	      escaped = ( fmt_val == '\\' ? 0 : -1 );
	    }

	  EXIT_LOOP_IF(fmt_val == '\0'); // does a ']' lack ?

	  fmt_val = ATcodec_Memory_Get_Char(fmt);
	}
      else if (!inList && fmt_val == '}')
	{
	  listParamsPtr = (ListParams *)ATcodec_Buffer_topElement(&listsParams);
	  VP_OS_ASSERT(listParamsPtr);

	  fmt_val = ATcodec_Memory_Get_Char(fmt);
	  *listParamsPtr->listLengthPtr = ++listParamsPtr->list_length;
	  ATcodec_Buffer_justPopElement(&listsParams);

	  inList = ( ATcodec_Buffer_topElement(&listsParams) ? -1 : 0 );
	}
      else
	{
	  bContinue = ATCODEC_FALSE;
	  PRINT_DEBUG;
	  CURCHARS_DEBUG(str_val, fmt_val);
	}
    }

  ATcodec_Buffer_destroy(&listsParams);
  *len_dec = (unsigned)str->current-(unsigned)str->start;
  return bContinue;
}

/**
 * Used to differentiate case where va_list is used from case where a memory area is used.
 */
static char ATCODEC_sprintf_memory_GetChar(ATcodec_Memory_t *mem, va_list *va)
{
  return ATcodec_Memory_Get_Char(mem);
}

static void ATCODEC_sprintf_memory_GetString(ATcodec_Memory_t *mem, va_list *va, char *dst)
{
  ATcodec_Memory_Get_String(mem, dst);
}

static int ATCODEC_sprintf_memory_Raw_GetInt(ATcodec_Memory_t *mem, va_list *va)
{
  return ATcodec_Memory_Raw_Get_Int(mem);
}

static char ATCODEC_sprintf_valist_GetChar(ATcodec_Memory_t *mem, va_list *va)
{
  return (char)va_arg(*va, int);
}

static void ATCODEC_sprintf_valist_GetString(ATcodec_Memory_t *mem, va_list *va, char *dst)
{
  strcpy(dst, va_arg(*va, char *));
}

static int ATCODEC_sprintf_valist_Raw_GetInt(ATcodec_Memory_t *mem, va_list *va)
{
  return va_arg(*va, int);
}

typedef struct _sprintf_funcs
{
  char (* ATCODEC_sprintf_GetChar      )(ATcodec_Memory_t *mem, va_list *va);
  void (* ATCODEC_sprintf_GetString    )(ATcodec_Memory_t *mem, va_list *va, char *dst);
  int  (* ATCODEC_sprintf_Raw_GetInt   )(ATcodec_Memory_t *mem, va_list *va);
}
sprintf_funcs;

static sprintf_funcs sprintf_funcs_memory =
{
  ATCODEC_sprintf_memory_GetChar,
  ATCODEC_sprintf_memory_GetString,
  ATCODEC_sprintf_memory_Raw_GetInt
};

static sprintf_funcs sprintf_funcs_valist =
{
  ATCODEC_sprintf_valist_GetChar,
  ATCODEC_sprintf_valist_GetString,
  ATCODEC_sprintf_valist_Raw_GetInt
};


ATCODEC_RET
ATCodec_common_sprintf(ATcodec_Memory_t *dest, int32_t *len, ATcodec_Memory_t *fmt, int params_from_memory, ATcodec_Memory_t *mem, va_list *va)
{
  int fmt_val,
    nbOpen,
    value;

  char str[1024];
  char *str_ptr;

  ATcodec_Buffer_t listsParams;
  ListParams listParams,
    *listParamsPtr;

  ATCODEC_RET bContinue = ATCODEC_TRUE;

  int escaped = -1,
    inList = -1;

  sprintf_funcs *funcs;

  if(params_from_memory)
    {
      funcs = &sprintf_funcs_memory;
    }
  else
    {
      funcs = &sprintf_funcs_valist;
    }

  ATcodec_Buffer_init(&listsParams, sizeof(ListParams), 1);

  fmt_val = ATcodec_Memory_Get_Char(fmt);

  while (bContinue == ATCODEC_TRUE && fmt_val != '\0')
    {
      if (fmt_val == '%' && escaped)
	{
	  switch ((fmt_val = ATcodec_Memory_Get_Char(fmt)))
	    {
	    case 'l' :
	      listParams.list_length = funcs->ATCODEC_sprintf_Raw_GetInt(mem, va);

	      fmt_val = ATcodec_Memory_Get_Char(fmt);

	      EXIT_LOOP_IF(fmt_val != '{');

	      inList = 0;
	      listParams.list_ptr_b = fmt->current;

	      nbOpen = 0;
	      while(fmt_val != '\0' && ((fmt_val = ATcodec_Memory_Get_Char(fmt)) != '}' || !escaped || nbOpen--))
		{
		  if (fmt_val == '\\')
		    {
		      escaped = 0;
		    }
		  else if (fmt_val == '{' && escaped)
		    {
		      nbOpen++;
		    }
		  else
		    {
		      escaped = -1;
		    }
		}

	      EXIT_LOOP_IF(fmt_val == '\0'); // does a '}' lack ?

	      fmt->current = listParams.list_ptr_b;

	      ATcodec_Buffer_pushElement(&listsParams, &listParams);
	      break;

	    case 'd' :
	      value = funcs->ATCODEC_sprintf_Raw_GetInt(mem, va);
	      ATcodec_Memory_Str_Put_Int (dest, value);
	      break;

	    case 's' :
	      str_ptr = &str[0];
	      funcs->ATCODEC_sprintf_GetString(mem, va, str_ptr);
	      while (*str_ptr)
		{
		  ATcodec_Memory_Put_Char(dest, *str_ptr++);
		}
	      break;

	    case 'c' :
	      ATcodec_Memory_Put_Char(dest, funcs->ATCODEC_sprintf_GetChar(mem, va));
	      break;

	    default : // prohibited pattern
	      bContinue = ATCODEC_FALSE; // exits with failure
	      PRINT_DEBUG;
	      break;
	    }
	}
      else if (!inList && fmt_val == '}' && escaped)
	{
	  listParamsPtr = (ListParams *)ATcodec_Buffer_topElement(&listsParams);
	  VP_OS_ASSERT(listParamsPtr);

	  if (--listParamsPtr->list_length)
	    {
	      fmt->current = listParamsPtr->list_ptr_b;
	      ATcodec_Memory_Put_Char(dest, ',');
	    }
	  else
	    {
	      ATcodec_Buffer_justPopElement(&listsParams);

	      inList = ( ATcodec_Buffer_topElement(&listsParams) ? -1 : 0 );
	    }
	}
      else if (fmt_val == '\\' && escaped)
	{
	  escaped = 0;
	}
      else if (fmt_val == '[' && escaped)
	{
	  if(!(int)funcs->ATCODEC_sprintf_Raw_GetInt(mem, va)) // dump facultative pattern
	    {
	      // nothing to do
	    }
	  else // do not dump facultative pattern
	    {
	      // skip facultative pattern
	      while(fmt_val != '\0' && ((fmt_val = ATcodec_Memory_Get_Char(fmt)) != ']' || !escaped))
		{
		  escaped = ( fmt_val == '\\' ? 0 : -1 );
		}

	      EXIT_LOOP_IF(fmt_val == '\0'); // does a ']' lack ?
	    }
	}
      else if (fmt_val != ']' || !escaped)
	{
	  escaped = -1;
	  ATcodec_Memory_Put_Char(dest, fmt_val);
	}

      fmt_val = ATcodec_Memory_Get_Char(fmt);
    }

  ATcodec_Buffer_destroy(&listsParams);
  ATcodec_Memory_Put_Char(dest, '\0');

  *len = (unsigned)dest->current-(unsigned)dest->start-1; //+1; Do not count the termination zero character
  return bContinue;
}


ATCODEC_RET
vp_atcodec_sprintf_valist(ATcodec_Memory_t *dest, int32_t *len, ATcodec_Memory_t *fmt, va_list *va)
{
  return ATCodec_common_sprintf(dest, len, fmt, 0, NULL, va);
}


ATCODEC_RET
vp_atcodec_sprintf_params(ATcodec_Memory_t *dest, int32_t *len, ATcodec_Memory_t *fmt, ATcodec_Memory_t *params)
{
  return ATCodec_common_sprintf(dest, len, fmt, 1, params, NULL);
}

