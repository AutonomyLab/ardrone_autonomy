/**
 *  \brief    VP OS. Error Handling
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 26/03/2007
 */

#ifndef _VP_OS_ERROR_HANDLING_H_
#define _VP_OS_ERROR_HANDLING_H_


#include <VP_Os/vp_os_types.h>


#define VP_OS_MAX_NUM_ERROR_HANDLERS      8
#define VP_OS_MAX_NUM_ERROR_MESSAGE_SIZE  256


/**
 * \brief API_SDK_SIGNATURE definition is used to parse the error messages.
 * \brief COM_SDK_SIGNATURE definition is used to parse the error messages.
 * \todo  Put these declarations in the makefile if possible
 */
#define VP_API_SDK_SIGNATURE    0x00AA
#define VP_COM_SDK_SIGNATURE    0x00BB
#define API_SDK_VERSION         1
#define COM_SDK_VERSION         1

typedef const char* (*vp_os_error_handler_t)(int32_t errorCode);

void vp_os_install_error_handler(uint32_t signature, vp_os_error_handler_t handler);

/**
 * @fn      error api message display
 * @param   int errorCode : error message in the list of error state message
 * @return  return the error Message
 */
const char* vp_os_get_error_message(uint32_t errorCode);

#endif // _VP_OS_ERROR_HANDLING_H_
