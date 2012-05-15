/**
 *  \brief    VP Api. Error management
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/12/2006
 *  \date     modification  19/03/2007
 */

#ifndef _VP_API_ERROR_H_
#define _VP_API_ERROR_H_


#include <VP_Os/vp_os_error_handling.h>


/**
 * \enum List of error state message
 */
enum {
  VP_API_OK = (VP_API_SDK_SIGNATURE << 16),
  VP_API_UNKNOW_ERROR,
  VP_API_INPUT_ERROR,
  VP_API_INPUT_CAMIF_SELECTION,
  VP_API_INPUT_CAMIF_ERROR,
  VP_API_INPUT_FILE_SELECTION,
  VP_API_INPUT_FILE_ERROR,
  VP_API_INPUT_SOCKET_BLUETOOTH_BNEP,
  VP_API_INPUT_SOCKET_BLUETOOTH_BNEP_ERROR,
  VP_API_INPUT_SOCKET_BLUETOOTH_RFCOMM,
  VP_API_INPUT_SOCKET_BLUETOOTH_RFCOMM_ERROR,
  VP_API_INPUT_SOCKET_BLUETOOTH_FTP,
  VP_API_INPUT_SOCKET_BLUETOOTH_FTP_ERROR,
  VP_API_INPUT_SOCKET_BLUETOOTH_BIP,
  VP_API_INPUT_SOCKET_BLUETOOTH_BIP_ERROR,
  VP_API_OUTPUT_FILE_SELECTION,
  VP_API_OUTPUT_SOCKET_SELECTION,
  VP_API_OUTPUT_BUFFER_SELECTION,
  VP_API_OUTPUT_ERROR,
  VP_API_MAX_NUM_ERROR
};


/**
 * @fn      error api message process
 * @param   int errorCode : error message in the list of error state message
 * @param   char **errorMessage : string of the message
 * @return  return the error Message
 */
const char* vp_api_format_message(uint32_t error_code);



#endif // _VP_API_ERROR_H_
