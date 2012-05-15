/**
 *  \brief    VP Api. Filter encoder/decoder stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck\@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle\@parrot.fr>
 *  \author   Thomas Landais <thomas.landais\@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  21/03/2007
 */

///////////////////////////////////////////////
// INCLUDES

#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>


#define STACK_SIZE 40000


PROTO_THREAD_ROUTINE(thread1,nomParams);
PROTO_THREAD_ROUTINE(thread2,nomParams);
PROTO_THREAD_ROUTINE(thread3,nomParams);

BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(thread1,20)
  THREAD_TABLE_ENTRY(thread2,20)
  THREAD_TABLE_ENTRY(thread3,20)
END_THREAD_TABLE

PROTO_THREAD_ROUTINE(thread1,nomParams)
{
  PRINT("Thread 1 Start\n");

  while(1)
    {
      vp_os_delay(100);
      PRINT("Thread 1 Loop\n");
    }
}

PROTO_THREAD_ROUTINE(thread2,nomParams)
{
  PRINT("Thread 2 Start\n");

  while(1)
    {
      vp_os_delay(200);
      PRINT("Thread 2 Loop\n");
    }
}

PROTO_THREAD_ROUTINE(thread3,nomParams)
{
  PRINT("Thread 3 Start\n");

  while(1)
    {
      vp_os_delay(300);
      PRINT("Thread 3 Loop\n");
    }
}

void runApplication(void)
{
  START_THREAD(thread1, NO_PARAM);
  START_THREAD(thread2, NO_PARAM);
  START_THREAD(thread3, NO_PARAM);

  JOIN_THREAD(thread1);
  JOIN_THREAD(thread2);
  JOIN_THREAD(thread3);
}

int main(int argc, char **argv)
{
  runApplication();

  return 0;
}
