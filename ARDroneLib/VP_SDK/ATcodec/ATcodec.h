/**
 * @file ATcodec_Buffer.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */

#ifndef _AT_CODEC_INCLUDE_
#define _AT_CODEC_INCLUDE_

#include <stdarg.h>

#include <VP_Os/vp_os_types.h>
#include <ATcodec/ATcodec_Memory.h>


#define INIT_CHAR_COUNT             128
#define INIT_SON_COUNT              1
#define INTERNAL_BUFFER_SIZE        1024

// For *_sprintf and *_sscanf debug
//#define AT_CODEC_DEBUG
// For common debug
//#define ATCODEC_DEBUG


// Return values
typedef enum _ATCODEC_Return_Values
{
  ATCODEC_TRUE,
  ATCODEC_FALSE
}
ATCODEC_RET;


#ifdef AT_CODEC_DEBUG
# define ATCODEC_DEBUG
#endif // <- AT_CODEC_DEBUG

#ifdef ATCODEC_DEBUG

# define ATCODEC_PRINT(...) \
         PRINT(__VA_ARGS__)
# define ATCODEC_ZERO_MEMSET(DEST, VALUE, SIZE) \
         vp_os_memset((DEST),(VALUE),(SIZE))

#else // ! ATCODEC_DEBUG

# define ATCODEC_PRINT(...) \
         do { } while(0)
# define ATCODEC_ZERO_MEMSET(DEST, VALUE, SIZE) \
         do { } while(0)

#endif // <- ATCODEC_DEBUG


/**
 * Inspired of the "Very complex scanf" from the CK5000 software, but again a little bit more complicated.
 *
 * This function allows the source string matching multiple model strings by using the '[' and ']' characters.
 * For example, if the model string is "AT*PAVI=[%d],[%d],[%d],[%d]", successful matching source strings can be "AT*PAVI=76,,," or "AT*PAVI=76,,,87".
 *
 * It recognizes "%d", "%s", "%c", "%l". To match a '%' character, "\\%" can be used in <fmt>.
 *
 * To formulate a list pattern, "%l" must be followed by a '{' character, then the part which can be repeated, and finally a closing '}'.
 *
 * Parameters are saved in a memory area given in parameter.
 *
 * CHANGES :
 *
 * 	- Shared memory saving of the parameters has been changed (no allocation for new parameters).
 *
 * 	- Facultative pattern management has been enhanced (not compatible with the old : closing character present, and opening character different).
 *
 * 	- List management has been added.
 *
 * 	- Spaces have to match exactly between <str> and <fmt> strings.
 *
 * 	- Access to characters has been optmized by using a pointer instead of accessing them with an index
 *
 * RULES :
 *
 * 	%s - The character which follows a "%s" in <fmt> cannot be the '[' or '%' characters.
 *
 * 	   - For "%s", the following character in <fmt> determines the closing of the string.
 * 	     An empty string does not match the "%s" pattern.
 *
 * 	%d - For "%d", the following character in <fmt> must be different from characters '0' to '9'.
 * 	     A matching "%d" pattern can begin with the '-' character.
 *
 * 	%c - "%c" can represent any character, excepted <str_lastchar>.
 *
 * 	[] - The character which follows a ']' closing character in <fmt> must not be a '%' or '[' character.
 * 	     The character which follows an '[' opening character in <fmt> must not be a '[' character but could be a '%' control character
 * 	     if the character which follows the respective ']' closing character make us able to determine if we try to match the facultative
 * 	     pattern or not. If this character does not correspond, we try to match the facultative pattern which begins with the '%' control character.
 *
 * 	   - Characters that are between a '[' and a ']' character cannot contain other '[' or ']', excepted if they are escaped by a '\\' in <fmt>.
 *
 * 	%l - Elements of the list are considered to be separated by the ',' character. So to use a ',' character in the format of a list
 * 	     element, you have to escape it by a '\\'.
 *
 * 	   - The character in <fmt> which follows the '}' character determines the closing of the list. Be careful not to have
 * 	     this character used in <str> for the elements of the list.
 *
 * 	   - Between the '{' and '}' characters, all special controls can be used. Imbricated "%l" are also supported.
 *
 * @param str			The source string to recognize
 *
 * @param fmt			The model string to match
 *
 * @param memory		The memory where to save parameters
 *
 * @param len_dec               Used to get length decoded in str
 *
 * @retVal ATCODEC_TRUE		If the strings match
 * @retVal ATCODEC_FALSE	If the strings do not match
 */
ATCODEC_RET
vp_atcodec_sscanf
	(
		ATcodec_Memory_t *str,

		ATcodec_Memory_t *fmt,

		ATcodec_Memory_t *memory,

		int *len_dec
	);

/**
 * Common private function, see below
 *
 * @param params_from_memory	1 : params are passed from a memory area / 0 : params are accessed using va_list
 * @param ...			Optional parameters used to fill patterns
 */
ATCODEC_RET
ATCodec_common_sprintf
	(
		ATcodec_Memory_t *dest,
		int32_t *len,

		ATcodec_Memory_t *fmt,

		int params_from_memory,
		ATcodec_Memory_t *params,
		va_list *va
	);

/**
 * Symmetrical function of the other one.
 *
 * The difficulty is to know to which pattern the arguments correspond. To resolve this,
 * we provide additionnal informations between parameters when necessary :
 *
 * 	- for "%l" : before the parameters of the "%l" pattern must be expressed the size of the list
 *
 * 	- for facultative patterns : a boolean is used to know if we follow the facultative pattern or if we skip it
 *
 * Of course, rules that must be respected for the other symmetrical function must also be respected when formulating
 * parameters. Otherwise, when decoded with the other, it will not work.
 *
 * @param dest			The buffer to receive the result. It has to be allocated before ...
 * @param len			The length of the produced string
 *
 * @param fmt			The model string to match
 *
 * @param ...			Optional parameters used to fill patterns
 *
 * @retVal ATCODEC_TRUE			If no error occured
 * @retVal ATCODEC_FALSE		If the model string is incorrect
 */
ATCODEC_RET
vp_atcodec_sprintf_valist
	(
		ATcodec_Memory_t *dest,
		int32_t *len,

		ATcodec_Memory_t *fmt,

		va_list *va
	);

/**
 * Same function as the previous, but with parameters passed through a memory area.
 *
 * The difficulty is to know to which pattern the arguments correspond. To resolve this,
 * we provide additionnal informations between parameters when necessary :
 *
 * 	- for "%l" : before the parameters of the "%l" pattern must be expressed the size of the list
 *
 * 	- for facultative patterns : a boolean is used to know if we follow the facultative pattern or if we skip it
 *
 * Of course, rules that must be respected for the other symmetrical function must also be respected when formulating
 * parameters. Otherwise, when decoded with the other, it will not work.
 *
 * @param dest			The buffer to receive the result. It has to be allocated before ...
 * @param len			The length of the produced string
 *
 * @param fmt			The model string to match
 *
 * @param params		Optional parameters used to fill patterns
 *
 * @retVal ATCODEC_TRUE			If no error occured
 * @retVal ATCODEC_FALSE		If the model string is incorrect
 */
ATCODEC_RET
vp_atcodec_sprintf_params
	(
		ATcodec_Memory_t *dest,
		int32_t *len,

		ATcodec_Memory_t *fmt,

		ATcodec_Memory_t *params
	);


#endif // ! _AT_CODEC_INCLUDE_
