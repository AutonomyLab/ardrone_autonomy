#include <Examples/common/common.h>
#include <VP_Os/vp_os_print.h>

void deviceinquiry_df(bdaddr_t* address,const char* name)
{
  PRINT("%s - %x:%x:%x:%x:%x:%x\n",
        name, address->b[0], address->b[1], address->b[2], address->b[3], address->b[4], address->b[5]);
}

void adapterinquiry_df(const char* name)
{
  PRINT("%s\n",name);
}
