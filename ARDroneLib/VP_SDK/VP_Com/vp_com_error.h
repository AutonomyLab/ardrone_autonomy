/**
 *  \brief    Com Api for video sdk. Error handling.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  2.0
 *  \date     20/12/2006
 */

#ifndef _VP_COM_ERROR_H_
#define _VP_COM_ERROR_H_


#include <VP_Os/vp_os_error_handling.h>


#define VP_COM_CHECK(res) if(res != VP_COM_OK) return res

enum {
  VP_COM_ERROR = -1,
  VP_COM_OK = (VP_COM_SDK_SIGNATURE << 16),
  VP_COM_UNKNOW_ERROR,
  VP_COM_ADAPTORNOTFOUND,
  VP_COM_PARAMERROR,
  VP_COM_NOTSUPPORTED,
  VP_COM_INITERROR,
  VP_COM_ADAPTORERROR,
  VP_COM_HOSTNOTREACHABLE,
  VP_COM_SOCKETERROR,
  VP_COM_NOTCONNECTED,
  VP_COM_MAX_NUM_ERROR
};

const char* vp_com_formatMessage(int32_t errorCode);

#endif // _VP_COM_ERROR_H_
