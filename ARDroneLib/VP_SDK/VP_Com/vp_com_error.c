#include <VP_Com/vp_com_error.h>

typedef struct _vp_com_msg_t
{
  int index;
  const char *msg;
} vp_com_msg_t;

static const vp_com_msg_t comMsg[] = { 
  { VP_COM_OK,                "OK"                                },
  { VP_COM_UNKNOW_ERROR,      "Unknow error"                      },
  { VP_COM_ADAPTORNOTFOUND,   "Local ethernet adaptor not found"  },
  { VP_COM_PARAMERROR,        "Bad parameter"                     },
  { VP_COM_NOTSUPPORTED,      "Not supported"                     },
  { VP_COM_INITERROR,         "Com not initialized"               },
  { VP_COM_ADAPTORERROR,      "Local ethernet adaptor error"      },
  { VP_COM_HOSTNOTREACHABLE,  "Unable to reach remote device"     },
  { VP_COM_SOCKETERROR,       "Socket error"                      },
  { VP_COM_NOTCONNECTED,      "Not connected"                     }
};

const char* vp_com_formatMessage(int32_t error_code)
{
  if(error_code >= VP_COM_MAX_NUM_ERROR)
    error_code = VP_COM_UNKNOW_ERROR;

  return comMsg[error_code & 0xFFFF].msg;
}
