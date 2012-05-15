/**
 *  \brief    VP OS. Error Handling
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 26/03/2007
 */

#include <VP_Os/vp_os_error_handling.h>
#include <VP_Os/vp_os_malloc.h>

#define VP_OS_ERROR_MESSAGE "Erreur"

typedef struct _vp_os_error_handlers_t
{

  uint32_t                signature;
  vp_os_error_handler_t   error_handler;

} vp_os_error_handlers_t;

static uint32_t vp_os_num_handlers = 0;
static vp_os_error_handlers_t vp_os_error_handlers[VP_OS_MAX_NUM_ERROR_HANDLERS];
static char vp_os_error_message[VP_OS_MAX_NUM_ERROR_MESSAGE_SIZE];

void vp_os_install_error_handler(uint32_t signature, vp_os_error_handler_t error_handler)
{
  vp_os_error_handlers[vp_os_num_handlers].signature      = signature;
  vp_os_error_handlers[vp_os_num_handlers].error_handler  = error_handler;

  vp_os_num_handlers ++;
}

const char* vp_os_get_error_message(uint32_t error_code)
{
  uint32_t signature  = error_code >> 16;
  uint32_t found      = 0;
  uint32_t i;

  vp_os_memset(vp_os_error_message, 0, VP_OS_MAX_NUM_ERROR_MESSAGE_SIZE);

  for( i = 0; !found && i < vp_os_num_handlers; i++)
  {
    if(signature == vp_os_error_handlers[i].signature)
      found++;
  }

  if(found)
  {
    strcpy(vp_os_error_message, vp_os_error_handlers[i].error_handler(error_code) );
  }
  else
  {
    // generic error message
    strcpy(vp_os_error_message, VP_OS_ERROR_MESSAGE);
    return vp_os_error_message;
  }

  return vp_os_error_message;
}
