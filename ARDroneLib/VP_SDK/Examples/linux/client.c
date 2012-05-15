// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck\@parrot.fr>
//  Date   : 03/01/2007
//
//  Parrot Video SDK : Examples/linux
//
// ---------------------------------------------- 

#include <VP_Os/vp_os_print.h>
#include <VP_Api/vp_api_error.h>
#include <Examples/common/com_client.h>

int main(void)
{
  if(FAILED(init_com_client()))
  {
    PRINT("Failed to init\n");
    return -1;
  }

  if(FAILED(run_com_client(COM_BNEP)))
  {
    PRINT("Failed to run\n");
  }
  else
  {
    int size = 17;
    char msg[16];

    PRINT("Connected to server\n");

    memset(msg,0,size);
    while(FAILED(read_client(msg,&size)))
      size = 16;

    PRINT("received: %s\n",msg);
  }

  if(FAILED(shutdown_com_client()))
  {
    PRINT("Failed to shutdown\n");
    return -1;
  }

  return 0;
}
