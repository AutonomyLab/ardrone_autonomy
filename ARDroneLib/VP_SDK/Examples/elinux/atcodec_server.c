#include <stdlib.h>

#define AT_MESSAGES_HEADER "ATcodec/ATcodec_Messages_ex.h"

#include "ATcodec/ATcodec_api.h"
#include "VP_Os/vp_os_types.h"
#include "VP_Os/vp_os_thread.h"
#include <Examples/common/atcodec_server.h>

#ifndef AT_MESSAGES_HEADER
#error You need to define AT_MESSAGES_HEADER
#endif

int main(int argc, char **argv)
{
  THREAD_HANDLE atcodec_test_handle;

  AT_CODEC_FUNCTIONS_PTRS ptrs =
    {
      .init     = AT_CODEC_init,
      .shutdown = AT_CODEC_shutdown,
      .open     = AT_CODEC_open,
      .close    = AT_CODEC_close,
      .read     = AT_CODEC_read,
      .write    = AT_CODEC_write,
    };

  ptrs.init = AT_CODEC_init;
  ptrs.shutdown = AT_CODEC_shutdown;
  ptrs.open = AT_CODEC_open;
  ptrs.close = AT_CODEC_close;
  ptrs.read = AT_CODEC_read;
  ptrs.write = AT_CODEC_write;

  ATcodec_Init_Library(&ptrs);

  vp_os_thread_create(thread_ATcodec_Commands_Server, 0, &atcodec_test_handle);
  vp_os_thread_join(atcodec_test_handle);

  return EXIT_SUCCESS;
}
