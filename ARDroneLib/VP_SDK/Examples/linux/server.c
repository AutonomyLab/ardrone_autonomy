// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck\@parrot.fr>
//  Date   : 03/01/2007
//
//  Parrot Video SDK : Examples/
//
// ----------------------------------------------

#include <VP_Os/vp_os_print.h>
#include <VP_Api/vp_api_error.h>
#include <Examples/common/com_server.h>

int main()
{
  PRINT("Thread Server Start\n");

  if(FAILED(init_com_server()))
  {
    PRINT("Failed to init\n");
    return;
  }

  if(FAILED(run_com_server(COM_RFCOMM)))
  {
    PRINT("Failed to run\n");
  }
  else
  {
    uint32_t size = 10;
    char msg[10];

    memset(msg,0,10);

    PRINT("Connection accepted\n");

    if(SUCCEED(read_server(msg,&size)))
      PRINT("msg: %s\n",msg);
  }

  if(FAILED(shutdown_com_server()))
  {
    PRINT("Failed to shutdown\n");
  }

  PRINT("Thread Server End\n");
}
