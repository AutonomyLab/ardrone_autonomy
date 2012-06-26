
/*-------------------------------------------------------------------------*/
/**
   @file    iniparser.c
   @author  N. Devillard
   @date    Sep 2007
   @version 3.0
   @brief   Parser for ini files.
*/
/*--------------------------------------------------------------------------*/
/*
    $Id: iniparser.c,v 1.1.2.1 2010-02-12 10:19:23 kleplat Exp $
    $Revision: 1.1.2.1 $
    $Date: 2010-02-12 10:19:23 $
*/
/*---------------------------- Includes ------------------------------------*/
#include <ctype.h>
#include <iniparser3.0b/src/iniparser.h>

#include <VP_Os/vp_os_print.h>

#include <Maths/matrices.h>


#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#include <config_keys.h>

/*---------------------------- Defines -------------------------------------*/
#define INI_INVALID_KEY     ((char*)-1)

/*---------------------------------------------------------------------------
                        Private to this module
 ---------------------------------------------------------------------------*/
/**
 * This enum stores the status for each parsed line (internal use only).
 */
typedef enum _line_status_ {
    LINE_UNPROCESSED,
    LINE_ERROR,
    LINE_EMPTY,
    LINE_COMMENT,
    LINE_SECTION,
    LINE_VALUE
} line_status ;

/*-------------------------------------------------------------------------*/
/**
  @brief	Convert a string to lowercase.
  @param	s	String to convert.
  @return	ptr to statically allocated string.

  This function returns a pointer to a statically allocated string
  containing a lowercased version of the input string. Do not free
  or modify the returned string! Since the returned string is statically
  allocated, it will be modified at each function call (not re-entrant).
 */
/*--------------------------------------------------------------------------*/
char * strlwc(const char * s)
{
    static char l[ASCIILINESZ+1];
    int i ;

    if (s==NULL) return NULL ;
    memset(l, 0, sizeof(l));
    i=0 ;
    while (s[i] && i<ASCIILINESZ) {
        l[i] = (char)tolower((int)s[i]);
        i++ ;
    }
    l[ASCIILINESZ]=(char)0;
    return l ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief	Remove blanks at the beginning and the end of a string.
  @param	s	String to parse.
  @return	ptr to statically allocated string.

  This function returns a pointer to a statically allocated string,
  which is identical to the input string, except that all blank
  characters at the end and the beg. of the string have been removed.
  Do not free or modify the returned string! Since the returned string
  is statically allocated, it will be modified at each function call
  (not re-entrant).
 */
/*--------------------------------------------------------------------------*/
static char * strstrip(char * s)
{
    static char l[ASCIILINESZ+1];
	char * last ;

    if (s==NULL) return NULL ;

	while (isspace((int)*s) && *s) s++;
	memset(l, 0, ASCIILINESZ+1);
	strncpy(l, s, sizeof(l)-1);
	last = l + strlen(l);
	while (last > l) {
		if (!isspace((int)*(last-1)))
			break ;
		last -- ;
	}
	*last = (char)0;
	return (char*)l ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get number of sections in a dictionary
  @param    d   Dictionary to examine
  @return   int Number of sections found in dictionary

  This function returns the number of sections found in a dictionary.
  The test to recognize sections is done on the string stored in the
  dictionary: a section name is given as "section" whereas a key is
  stored as "section:key", thus the test looks for entries that do not
  contain a colon.

  This clearly fails in the case a section name contains a colon, but
  this should simply be avoided.

  This function returns -1 in case of error.
 */
/*--------------------------------------------------------------------------*/
int iniparser_getnsec(dictionary * d)
{
    int i ;
    int nsec ;

    if (d==NULL) return -1 ;
    nsec=0 ;
    for (i=0 ; i<d->size ; i++) {
        if (d->key[i]==NULL)
            continue ;
        if (/*strchr(d->key[i], ':')*/d->values[i].type==INI_SECTION) {
            nsec ++ ;
        }
    }
    return nsec ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get name for section n in a dictionary.
  @param    d   Dictionary to examine
  @param    n   Section number (from 0 to nsec-1).
  @return   Pointer to char string

  This function locates the n-th section in a dictionary and returns
  its name as a pointer to a string statically allocated inside the
  dictionary. Do not free or modify the returned string!

  This function returns NULL in case of error.
 */
/*--------------------------------------------------------------------------*/
char * iniparser_getsecname(dictionary * d, int n)
{
    int i ;
    int foundsec ;

    if (d==NULL || n<0) return NULL ;
    foundsec=0 ;
    for (i=0 ; i<d->size ; i++) {
        if (d->key[i]==NULL)
            continue ;
        if (/*strchr(d->key[i], ':')==NULL*/ d->values[i].type==INI_SECTION) {
            foundsec++ ;
            if (foundsec>n)
                break ;
        }
    }
    if (foundsec<=n) {
        return NULL ;
    }
    return d->key[i] ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Duplicate a string
  @param    s String to duplicate
  @return   Pointer to a newly allocated string, to be freed with free()

  This is a replacement for strdup(). This implementation is provided
  for systems that do not have it.
 */
/*--------------------------------------------------------------------------*/

#ifndef __XSTRDUP_DEFINED__
#define __XSTRDUP_DEFINED__
static char * xstrdup(const char * s)
{
    char * t ;
    if (!s)
        return NULL ;
    t = malloc(strlen(s)+1) ;
    if (t) {
        strcpy(t,s);
    }
    return t ;
}
#endif

void iniparser_ptr2val(dictionary_value* value)
{
  if( value && value->ptr != NULL )
  {
    if(value->val)
    {
      free(value->val);
      value->val = NULL;
    }

    switch( value->type )
    {
    case INI_STRING:
      value->val = xstrdup(value->ptr);
      break;

    case INI_INT:
      value->val = malloc(64);
      sprintf(value->val, "%d", *(int*)(value->ptr));
      break;

    case INI_FLOAT:
      value->val = malloc(64);
      sprintf(value->val, "%.7e", *(float*)(value->ptr));
      break;

    case INI_DOUBLE:
      value->val = malloc(64);
      sprintf(value->val, "%.16e", *(double*)(value->ptr));
      break;

    case INI_BOOLEAN:
      value->val = malloc(64);
      sprintf(value->val, "%s", ( *(int*)(value->ptr) ? "TRUE" : "FALSE" ) );
      break;

    case INI_VECTOR:
      value->val = malloc(256);
      sprintf(value->val, "{ %.7e %.7e %.7e }", ((vector31_t*)value->ptr)->x, ((vector31_t*)value->ptr)->y, ((vector31_t*)value->ptr)->z);
      break;

    case INI_VECTOR21:
      value->val = malloc(256);
      sprintf(value->val, "{ %.7e %.7e }", ((vector21_t*)value->ptr)->x, ((vector21_t*)value->ptr)->y);
      break;

    case INI_MATRIX:
      value->val = malloc(1024);
      sprintf(value->val, "{  %.7e %.7e %.7e  %.7e %.7e %.7e  %.7e %.7e %.7e }",
                          ((matrix33_t*)value->ptr)->m11, ((matrix33_t*)value->ptr)->m12, ((matrix33_t*)value->ptr)->m13,
                          ((matrix33_t*)value->ptr)->m21, ((matrix33_t*)value->ptr)->m22, ((matrix33_t*)value->ptr)->m23,
                          ((matrix33_t*)value->ptr)->m31, ((matrix33_t*)value->ptr)->m32, ((matrix33_t*)value->ptr)->m33 );
      break;

    case INI_UNKNOW:
    default:
      break;
    }
  }
}

void iniparser_val2ptr(dictionary_value* value)
{
  if( value )
  {
    if( value->ptr != NULL && value->val != NULL )
    {
      switch( value->type )
        {
        case INI_STRING:
          strcpy(value->ptr, value->val);
          break;

        case INI_INT:
          *(int*)(value->ptr) = (int)strtol(value->val, NULL, 0);
          break;

        case INI_FLOAT:
          *(float*)(value->ptr) = (float)atof(value->val);
          break;

        case INI_DOUBLE:
          *(double*)(value->ptr) = (double)atof(value->val);
          break;

        case INI_BOOLEAN:
          {
            int ret;
            char c = value->val[0];

            if( c=='y' || c=='Y' || c=='1' || c=='t' || c=='T') {
              ret = 1 ;
            } else if (c=='n' || c=='N' || c=='0' || c=='f' || c=='F') {
              ret = 0 ;
            }
            else {
              ret = 0xdeadbeef;
            }

            *(int*)(value->ptr) = ret;
          }
          break;

        case INI_VECTOR:
          sscanf(value->val, "{ %e %e %e }", &((vector31_t*)value->ptr)->x, &((vector31_t*)value->ptr)->y, &((vector31_t*)value->ptr)->z);
          break;

        case INI_VECTOR21:
          sscanf(value->val, "{ %e %e }", &((vector21_t*)value->ptr)->x, &((vector21_t*)value->ptr)->y);
          break;

        case INI_MATRIX:
          sscanf(value->val, "{ %e %e %e %e %e %e %e %e %e }",
                             &((matrix33_t*)value->ptr)->m11, &((matrix33_t*)value->ptr)->m12, &((matrix33_t*)value->ptr)->m13,
                             &((matrix33_t*)value->ptr)->m21, &((matrix33_t*)value->ptr)->m22, &((matrix33_t*)value->ptr)->m23,
                             &((matrix33_t*)value->ptr)->m31, &((matrix33_t*)value->ptr)->m32, &((matrix33_t*)value->ptr)->m33 );
          break;

        case INI_UNKNOW:
        default:
          break;
        }
      }
    }
}



/*-------------------------------------------------------------------------*/
/**
  @brief    Transfer values from the dictionary to the bound variables
  @param    d   Dictionary to dump.
  @param    scope Scope for which values must be transfered (set to -1 to tranfer all values)
  @return   void
 */
/*--------------------------------------------------------------------------*/
void iniparser_vals2ptrs(dictionary * d , int scope)
{
    int     i ;

    if (d==NULL) return ;
    for (i=0 ; i<d->size ; i++) {

    	if (d->key[i]==NULL)
            continue ;

        if( d->values[i].ptr != NULL ) {
        	if (scope==-1 || d->values[i].scope==scope) {
        		iniparser_val2ptr(&d->values[i]);
        	}
        }
    }

    return ;
}


/*-------------------------------------------------------------------------*/
/**
  @brief    Transfer values from the bound variables to the dictionary.
  @param    d Dictionary to fill.
  @param    scope Scope for which values must be transfered (set to -1 to tranfer all values)
  @return   void
 */
/*--------------------------------------------------------------------------*/
void iniparser_ptrs2vals(dictionary * d , int scope)
{
    int     i ;

    if (d==NULL) return ;
    for (i=0 ; i<d->size ; i++) {

    	if (d->key[i]==NULL)
            continue ;

        if( d->values[i].ptr != NULL ) {
        	if (scope==-1 || d->values[i].scope==scope) {
        		iniparser_ptr2val(&d->values[i]);
        	}
        }
    }

    return ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Dump a dictionary to an opened file pointer.
  @param    d   Dictionary to dump.
  @param    f   Opened file pointer to dump to.
  @return   void

  This function prints out the contents of a dictionary, one element by
  line, onto the provided file pointer. It is OK to specify @c stderr
  or @c stdout as output files. This function is meant for debugging
  purposes mostly.
 */
/*--------------------------------------------------------------------------*/
void iniparser_dump(dictionary * d)
{
    int     i ;

    if (d==NULL) return ;
    for (i=0 ; i<d->size ; i++) {
        if (d->key[i]==NULL)
            continue ;
        if( d->values[i].ptr != NULL ) {
          switch(d->values[i].type)
          {
            case INI_STRING:
              PRINT("%s= %s [STRING]\n", d->key[i], d->values[i].val);
              break;

            case INI_INT:
              PRINT("%s= %d [INT]\n", d->key[i], *(int*)d->values[i].ptr);
              break;

            case INI_FLOAT:
              PRINT("%s= %f [FLOAT]\n", d->key[i], *(float*)d->values[i].ptr);
              break;

            case INI_DOUBLE:
              PRINT("%s= %lf [DOUBLE]\n", d->key[i], *(double*)d->values[i].ptr);
              break;

            case INI_BOOLEAN:
              PRINT("%s= %d [BOOLEAN]\n", d->key[i], *(int*)d->values[i].ptr);
              break;

            case INI_VECTOR:
              PRINT("%s= { %f %f %f } [VECTOR]\n", d->key[i], ((vector31_t*)d->values[i].ptr)->x,
                                                              ((vector31_t*)d->values[i].ptr)->y,
                                                              ((vector31_t*)d->values[i].ptr)->z);
              break;

            case INI_VECTOR21:
              PRINT("%s= { %f %f } [VECTOR21]\n", d->key[i],  ((vector21_t*)d->values[i].ptr)->x,
                                                              ((vector21_t*)d->values[i].ptr)->y);
              break;

            case INI_MATRIX:
              PRINT("%s= { %f %f %f %f %f %f %f %f %f } [MATRIX]\n", d->key[i],
                      ((matrix33_t*)d->values[i].ptr)->m11, ((matrix33_t*)d->values[i].ptr)->m12, ((matrix33_t*)d->values[i].ptr)->m13,
                      ((matrix33_t*)d->values[i].ptr)->m21, ((matrix33_t*)d->values[i].ptr)->m22, ((matrix33_t*)d->values[i].ptr)->m23,
                      ((matrix33_t*)d->values[i].ptr)->m31, ((matrix33_t*)d->values[i].ptr)->m32, ((matrix33_t*)d->values[i].ptr)->m33 );
              break;

            case INI_UNKNOW:
            default:
              PRINT("%s=[%s]\n", d->key[i], d->values[i].val);
              break;
          }
//             fprintf(f, "[%s]=[%s]\n", d->key[i], d->val[i]);
        } else {
          if (d->values[i].val!=NULL) {
            PRINT("%s=[%s]\n", d->key[i], d->values[i].val);
          }
          else {
            if( d->values[i].type == INI_SECTION )
              PRINT("[%s]\n", d->key[i]);
            else
              PRINT("%s=UNDEF\n", d->key[i]);
//             fprintf(f, "[%s]=UNDEF\n", d->key[i]);
        }
      }
    }
    return ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Save a dictionary to a loadable ini file
  @param    d   Dictionary to dump
  @param    f   Opened file pointer to dump to
  @param    flag_dump_k_shallows  If true, K_SHALLOW values are written on disk
  @return   void

  This function dumps a given dictionary into a loadable ini file.
  It is Ok to specify @c stderr or @c stdout as output files.
 */
/*--------------------------------------------------------------------------*/
void iniparser_dump_ini(dictionary * d, FILE * f)
{ iniparser_dump_ini_a4(d,f,0,1);    }
void iniparser_dump_ini_a3(dictionary * d, FILE * f , int flag_dump_k_shallows)
{ iniparser_dump_ini_a4(d,f,flag_dump_k_shallows,1); }

void iniparser_dump_ini_a4(dictionary * d, FILE * f , int flag_dump_k_shallows , int update_values_from_memory)
{
    int     i, j ;
    char    keym[ASCIILINESZ+1];
    int     nsec ;
    char *  secname ;
    int     seclen ;

    if (d==NULL || f==NULL) return ;

    nsec = iniparser_getnsec(d);
    if (nsec<1) {
        /* No section in file: dump all keys as they are */
        for (i=0 ; i<d->size ; i++) {
            if (d->key[i]==NULL)
                continue ;
            /* Stephane - add K_SHALLOW support */
            	if (!flag_dump_k_shallows)
            		if ((d->values[i].rw&K_SHALLOW)!=0)
            			continue;
            if(update_values_from_memory)
            	iniparser_ptr2val(&d->values[i]); // Make sure we are synchro before dump
            fprintf(f, "%s = %s\n", d->key[i], d->values[i].val);
        }
        return ;
    }
    for (i=0 ; i<nsec ; i++) {
        secname = iniparser_getsecname(d, i) ;
        seclen  = (int)strlen(secname);
        fprintf(f, "\n[%s]\n", secname);
        sprintf(keym, "%s:", secname);
        for (j=0 ; j<d->size ; j++) {
            if (d->key[j]==NULL)
                continue ;
            /* Stephane - add K_SHALLOW support */
            	if (!flag_dump_k_shallows)
            		if ((d->values[j].rw&K_SHALLOW)!=0)
            			continue;
            if (!strncmp(d->key[j], strlwc(keym), seclen+1)) {
            	if(update_values_from_memory)
            		iniparser_ptr2val(&d->values[j]); // Make sure we are synchro before dump
                fprintf(f,
                        "%-30s = %s\n",
                        d->key[j]+seclen+1,
                        d->values[j].val ? d->values[j].val : "");
            }
        }
    }
    fprintf(f, "\n");
    return ;
}

static dictionary_value* iniparser_getdictionaryvalue(dictionary * d, const char * key)
{
  char * lc_key;

  if (d==NULL || key==NULL)
    return NULL;

  lc_key = strlwc(key);
  return dictionary_get(d, lc_key);
}

int iniparser_alias_ex(dictionary * d, const char* kkey, int type, void* ptr, void (*cb)(void), char rw,int scope);
int iniparser_alias(dictionary * d, const char* kkey, int type, void* ptr, void (*cb)(void), char rw)
{	return iniparser_alias_ex(d,kkey,type,ptr,cb,rw,CAT_COMMON);  }

int iniparser_alias_ex(dictionary * d, const char* kkey, int type, void* ptr, void (*cb)(void), char rw,int scope)
{
  dictionary_value* value;
  char *pos;
  char key[ASCIILINESZ];

  strcpy(&key[0], kkey);

  // Create a new section ?
  pos = strchr(key, ':');
  if( pos )
  {
    *pos = '\0';
    value = iniparser_getdictionaryvalue(d, key);
    if( value == NULL)
      dictionary_set(d, key, NULL, INI_SECTION, NULL,NULL);
    *pos = ':';
  }
  else
  {
    return -1;
  }

  value = iniparser_getdictionaryvalue(d, key);

	// TODO: change hardcoded value
  if( value == NULL )
  {
    if( ptr != NULL )
    {
      // Create a new value
      value = dictionary_set(d, strlwc(key), NULL, type, ptr, cb);
      value->callback = cb;
      value->rw = rw;
      value->scope = scope;
      iniparser_ptr2val(value);

      if( cb )
        cb();
    }
    else
    {
      return -1;
    }
  }
  else
  {
    if( ptr != NULL )
    {
      // setup existing value
      value->type = type;
      value->ptr  = ptr;
      value->callback = cb;
      value->rw = rw;
      value->scope = scope;

			if(rw & K_NOBIND)
			{
				iniparser_ptr2val(value);
			}
			else
			{
				iniparser_val2ptr(value);
			}

      if( cb )
        cb();
    }
    else
    {
      dictionary_unset(d, key);
    }
  }

  return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key
  @param    d       Dictionary to search
  @param    key     Key string to look for
  @param    def     Default value to return if key not found.
  @return   pointer to statically allocated character string

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the pointer passed as 'def' is returned.
  The returned char pointer is pointing to a string allocated in
  the dictionary, do not free or modify it.
 */
/*--------------------------------------------------------------------------*/
char * iniparser_getstring(dictionary * d, const char * key, char * def)
{
  dictionary_value* value = iniparser_getdictionaryvalue(d, key);

  if( value == NULL )
    return def;

  return value->val;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key, convert to an int
  @param    d Dictionary to search
  @param    key Key string to look for
  @param    notfound Value to return in case of error
  @return   integer

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the notfound value is returned.

  Supported values for integers include the usual C notation
  so decimal, octal (starting with 0) and hexadecimal (starting with 0x)
  are supported. Examples:

  "42"      ->  42
  "042"     ->  34 (octal -> decimal)
  "0x42"    ->  66 (hexa  -> decimal)

  Warning: the conversion may overflow in various ways. Conversion is
  totally outsourced to strtol(), see the associated man page for overflow
  handling.

  Credits: Thanks to A. Becker for suggesting strtol()
 */
/*--------------------------------------------------------------------------*/
int iniparser_getint(dictionary * d, const char * key, int notfound)
{
  int i;

  dictionary_value* value = iniparser_getdictionaryvalue(d, key);

  if( value == NULL )
    return notfound;

  if( value->ptr )
  {
    i = *(int*)(value->ptr);
  }
  else
  {
    i = strtol(value->val, NULL, 0);
  }

  return i;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key, convert to a double
  @param    d Dictionary to search
  @param    key Key string to look for
  @param    notfound Value to return in case of error
  @return   double

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the notfound value is returned.
 */
/*--------------------------------------------------------------------------*/
double iniparser_getdouble(dictionary * dict, char * key, double notfound)
{
  double d;
  dictionary_value* value = iniparser_getdictionaryvalue(dict, key);

  if( value == NULL )
    return notfound;

  if( value->ptr )
  {
    switch( value->type )
    {
      case INI_FLOAT:
        d = (double)*(float*)(value->ptr);
        break;
      case INI_DOUBLE:
        d = *(double*)(value->ptr);
        break;
      default:
        d = notfound;
        break;
    }
  }
  else
  {
    d = (int)atof(value->val);
  }

  return d;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Get the string associated to a key, convert to a boolean
  @param    d Dictionary to search
  @param    key Key string to look for
  @param    notfound Value to return in case of error
  @return   integer

  This function queries a dictionary for a key. A key as read from an
  ini file is given as "section:key". If the key cannot be found,
  the notfound value is returned.

  A true boolean is found if one of the following is matched:

  - A string starting with 'y'
  - A string starting with 'Y'
  - A string starting with 't'
  - A string starting with 'T'
  - A string starting with '1'

  A false boolean is found if one of the following is matched:

  - A string starting with 'n'
  - A string starting with 'N'
  - A string starting with 'f'
  - A string starting with 'F'
  - A string starting with '0'

  The notfound value returned if no boolean is identified, does not
  necessarily have to be 0 or 1.
 */
/*--------------------------------------------------------------------------*/
int iniparser_getboolean(dictionary* dict, const char* key, int notfound)
{
  int ret;

  dictionary_value* value = iniparser_getdictionaryvalue(dict, key);

  if( value == NULL )
    return notfound;

  if( value->ptr )
  {
    ret = *(int*)(value->ptr);
  }
  else
  {
    char c = value->val[0];

    if( c=='y' || c=='Y' || c=='1' || c=='t' || c=='T') {
        ret = 1 ;
    } else if (c=='n' || c=='N' || c=='0' || c=='f' || c=='F') {
        ret = 0 ;
    } else {
        ret = notfound ;
    }
  }

  return ret;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Finds out if a given entry exists in a dictionary
  @param    ini     Dictionary to search
  @param    entry   Name of the entry to look for
  @return   integer 1 if entry exists, 0 otherwise

  Finds out if a given entry exists in the dictionary. Since sections
  are stored as keys with NULL associated values, this is the only way
  of querying for the presence of sections in a dictionary.
 */
/*--------------------------------------------------------------------------*/
int iniparser_find_entry( dictionary  *   ini, char* entry )
{
  return iniparser_getstring(ini, entry, INI_INVALID_KEY) != INI_INVALID_KEY ? 1 : 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Set an entry in a dictionary.
  @param    ini     Dictionary to modify.
  @param    entry   Entry to modify (entry name)
  @param    val     New value to associate to the entry.
  @return   int 0 if Ok, -1 otherwise.

  If the given entry can be found in the dictionary, it is modified to
  contain the provided value. If it cannot be found, -1 is returned.
  It is Ok to set val to NULL.
 */
/*--------------------------------------------------------------------------*/

int iniparser_setstring(dictionary * ini, const char * entry, const char * val)
{ return iniparser_setstring_a4(ini,entry,val,1); }

int iniparser_setstring_a4(dictionary * ini,const char * entry, const char * val , int trigger_callback)
{
  dictionary_value* value = dictionary_set(ini, strlwc(entry), val, INI_UNKNOW, NULL,NULL);

  if( value == NULL )
    return -1;

  iniparser_val2ptr(value);

  if( (trigger_callback) && (value->callback) )
    value->callback();

  return 0;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Delete an entry in a dictionary
  @param    ini     Dictionary to modify
  @param    entry   Entry to delete (entry name)
  @return   void

  If the given entry can be found, it is deleted from the dictionary.
 */
/*--------------------------------------------------------------------------*/
void iniparser_unset(dictionary * ini, char * entry)
{
    dictionary_unset(ini, strlwc(entry));
}

/*-------------------------------------------------------------------------*/
/**
  @brief	Load a single line from an INI file
  @param    input_line  Input line, may be concatenated multi-line input
  @param    section     Output space to store section
  @param    key         Output space to store key
  @param    value       Output space to store value
  @return   line_status value
 */
/*--------------------------------------------------------------------------*/
static line_status iniparser_line(
    char * input_line,
    char * section,
    char * key,
    char * value)
{
    line_status sta ;
    char        line[ASCIILINESZ+1] = { 0 };
    int         len ;

    strncpy(line, strstrip(input_line), sizeof(line)-1);
    len = (int)strlen(line);

    sta = LINE_UNPROCESSED ;
    if (len<1) {
        /* Empty line */
        sta = LINE_EMPTY ;
    } else if (line[0]=='#') {
        /* Comment line */
        sta = LINE_COMMENT ; 
    } else if (line[0]=='[' && line[len-1]==']') {
        /* Section name */
        sscanf(line, "[%[^]]", section);
        strcpy(section, strstrip(section));
        strcpy(section, strlwc(section));
        sta = LINE_SECTION ;
    } else if ((sscanf (line, "%[^=] = \"%[^\"]\"", key, value) == 2
           ||  sscanf (line, "%[^=] = '%[^\']'",   key, value) == 2
           ||  sscanf (line, "%[^=] = %[^;#]",     key, value) == 2) /*&& (strcmp(section,""))*/) {
        /* Usual key=value, with or without comments */
        strcpy(key, strstrip(key));
        strcpy(key, strlwc(key));
        strcpy(value, strstrip(value));
        /*
         * sscanf cannot handle '' or "" as empty values
         * this is done here
         */
        if (!strcmp(value, "\"\"") || (!strcmp(value, "''"))) {
            value[0]=0 ;
        }
        sta = LINE_VALUE ;
    } else if ((sscanf(line, "%[^=] = %[;#]", key, value)==2
           ||  sscanf(line, "%[^=] %[=]", key, value) == 2) /*&& (strcmp(section,""))*/) {
        /*
         * Special cases:
         * key=
         * key=;
         * key=#
         */
        strcpy(key, strstrip(key));
        strcpy(key, strlwc(key));
        value[0]=0 ;
        sta = LINE_VALUE ;
    } else {
        /* Generate syntax error */
        sta = LINE_ERROR ;
    }
    return sta ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Parse an ini file and return an allocated dictionary object
  @param    ininame Name of the ini file to read.
  @return   Pointer to newly allocated dictionary

  This is the parser for ini files. This function is called, providing
  the name of the file to be read. It returns a dictionary object that
  should not be accessed directly, but through accessor functions
  instead.

  The returned dictionary must be freed using iniparser_freedict().
 */
/*--------------------------------------------------------------------------*/
dictionary * iniparser_load(FILE* in, dictionary * dict )
{
    char line    [ASCIILINESZ+1] ;
    char section [ASCIILINESZ+1] ;
    char key     [ASCIILINESZ+1] ;
    char tmp     [ASCIILINESZ+1] ;
    char val     [ASCIILINESZ+1] ;

    int  last=0 ;
    int  len ;
    int  lineno=0 ;
    int  errs=0;
/*
    if ((in=fopen(ininame, "r"))==NULL) {
      fprintf(stderr, "iniparser: cannot open %s\n", ininame);
      return dict ;
    }
*/
    if( dict == NULL )
      dict = dictionary_new(0) ;

    if (!dict) {
        // fclose(in);
        return dict ;
    }

    memset(line,    0, ASCIILINESZ);
    memset(section, 0, ASCIILINESZ);
    memset(key,     0, ASCIILINESZ);
    memset(val,     0, ASCIILINESZ);
    last=0 ;

    while (fgets(line+last, ASCIILINESZ-last, in)!=NULL) {
        lineno++ ;
        len = (int)strlen(line)-1;
        /* Safety check against buffer overflows */
        if (line[len]!='\n') {
            fprintf(stderr,
                    "iniparser: input line too long in file (line %d)\n",
//                    ininame,
                    lineno);
            dictionary_del(dict);
            dict = NULL;
            // fclose(in);
            return dict ;
        }
        /* Get rid of \n and spaces at end of line */
        while ((len>=0) &&
                ((line[len]=='\n') || (isspace(line[len])))) {
            line[len]=0 ;
            len-- ;
        }
        /* Detect multi-line */
        if (line[len]=='\\') {
            /* Multi-line value */
            last=len ;
            continue ;
        } else {
            last=0 ;
        }
        switch (iniparser_line(line, section, key, val)) {
            case LINE_EMPTY:
            case LINE_COMMENT:
            break ;

            case LINE_SECTION:
            errs = dictionary_set(dict, section, NULL, INI_SECTION, NULL,NULL) != NULL ? 0 : -1;
            break ;

            case LINE_VALUE:
            if (strcmp(section,"")!=0)
              sprintf(tmp, "%s:%s", section, key);
            else
              strcpy(tmp,key);
            errs = iniparser_setstring(dict, tmp, val);
            // errs = dictionary_set(dict, tmp, val, UNKNOW, NULL) != NULL ? 0 : -1;
            break ;

            case LINE_ERROR:
            fprintf(stderr, "iniparser: syntax error in file (%d):\n",
//                    ininame,
                    lineno);
            fprintf(stderr, "-> %s\n", line);
            errs++ ;
            break;

            default:
            break ;
        }
        memset(line, 0, ASCIILINESZ);
        last=0;
        if (errs<0) {
            fprintf(stderr, "iniparser: memory allocation failure\n");
            break ;
        }
    }
    if (errs) {
        dictionary_del(dict);
        dict = NULL ;
    }
    // fclose(in);
    return dict ;
}

/*-------------------------------------------------------------------------*/
/**
  @brief    Free all memory associated to an ini dictionary
  @param    d Dictionary to free
  @return   void

  Free all memory associated to an ini dictionary.
  It is mandatory to call this function before the dictionary object
  gets out of the current context.
 */
/*--------------------------------------------------------------------------*/
void iniparser_freedict(dictionary * d)
{
    dictionary_del(d);
}

/* vim: set ts=4 et sw=4 tw=75 */
