/**
 *  \brief    VP Api. Error management
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/12/2006
 *  \date     modification  19/03/2007
 */

///////////////////////////////////////////////
// INCLUDES

#include <VP_Api/vp_api_error.h>

/**
 * \struct private definition of structure message
 * \brief  This structure is used to create the list of error message
 * \brief  the Parrot Video Software Development Kit
 */
typedef struct _vp_api_msg_
{

  uint32_t    index;
  const char *msg;

} vp_api_msg_t;

static const vp_api_msg_t vp_api_errorMsg[] =
{ 
  {VP_API_OK,                                            "OK"                                    },
  {VP_API_UNKNOW_ERROR,                                  "Unknown error"                         },
  {VP_API_INPUT_ERROR,                                   "Input error"                           },
  {VP_API_INPUT_CAMIF_SELECTION,                         "Input CAMIF"                           },
  {VP_API_INPUT_CAMIF_ERROR    ,                         "Input CAMIF error"                     },
  {VP_API_INPUT_FILE_SELECTION,                          "Input FILE"                            },
  {VP_API_INPUT_FILE_ERROR,                              "Input FILE error"                      },
  {VP_API_INPUT_SOCKET_BLUETOOTH_BNEP,                   "Input Socket Bluetooth BNEP"           },
  {VP_API_INPUT_SOCKET_BLUETOOTH_BNEP_ERROR,             "Input Socket Bluetooth BNEP Error"     },
  {VP_API_INPUT_SOCKET_BLUETOOTH_RFCOMM,                 "Input Socket Bluetooth RFComm"         },
  {VP_API_INPUT_SOCKET_BLUETOOTH_RFCOMM_ERROR,           "Input Socket Bluetooth RFComm Error"   },
  {VP_API_INPUT_SOCKET_BLUETOOTH_FTP,                    "Input Socket Bluetooth FTP"            },
  {VP_API_INPUT_SOCKET_BLUETOOTH_FTP_ERROR,              "Input Socket Bluetooth FTP Error"      },
  {VP_API_INPUT_SOCKET_BLUETOOTH_BIP,                    "Input Socket Bluetooth BIP"            },
  {VP_API_INPUT_SOCKET_BLUETOOTH_BIP_ERROR,              "Input Socket Bluetooth BIP Error"      },
  {VP_API_OUTPUT_FILE_SELECTION,                         "Output file selection"                 },
  {VP_API_OUTPUT_SOCKET_SELECTION,                       "Output socket selection"               },
  {VP_API_OUTPUT_BUFFER_SELECTION,                       "Output buffer selection"               },
  {VP_API_OUTPUT_ERROR,                                  "Output error"                          },
};


const char* vp_api_format_message(uint32_t error_code)
{
  if(error_code >= VP_API_MAX_NUM_ERROR)
          error_code = VP_API_UNKNOW_ERROR;

  return vp_api_errorMsg[error_code & 0xFFFF].msg;
}
