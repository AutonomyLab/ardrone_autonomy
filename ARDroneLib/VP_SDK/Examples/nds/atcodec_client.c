
#include <stdio.h>
#include <stdlib.h>

#include <ATcodec/ATcodec_api.h>
#include <VP_Stages/vp_stages_io_com.h>

#define AT_MESSAGES_HEADER "ATcodec/ATcodec_Messages_ex.h"

#include "ATcodec/ATcodec_api.h"
#include "VP_Os/vp_os_types.h"
#include "VP_Os/vp_os_thread.h"
#include "VP_Os/vp_os_delay.h"
#include "VP_Os/vp_os_print.h"
#include <Examples/common/atcodec_client.h>

#include <nds.h>

#ifndef AT_MESSAGES_HEADER
#error You need to define AT_MESSAGES_HEADER
#endif

extern AT_CODEC_MSG_IDS ids;

#define INTRA_CMD_DELAY 10


volatile int frame = 0;


//---------------------------------------------------------------------------------
void Vblank()
{
  //---------------------------------------------------------------------------------
  frame++;
}


//---------------------------------------------------------------------------------
void initNDS()
{
  //---------------------------------------------------------------------------------
  irqInit();
  irqSet(IRQ_VBLANK, Vblank);
  irqEnable(IRQ_VBLANK);
  videoSetMode(0);	//not using the main screen
  videoSetModeSub(MODE_0_2D | DISPLAY_BG0_ACTIVE);	//sub bg 0 will be used to print text
  vramSetBankC(VRAM_C_SUB_BG);

  SUB_BG0_CR = BG_MAP_BASE(31);
	
  BG_PALETTE_SUB[255] = RGB15(31,31,31);	//by default font will be rendered with color 255
	
  //consoleInit() is a lot more flexible but this gets you up and running quick
  consoleInitDefault((u16*)SCREEN_BASE_BLOCK_SUB(31), (u16*)CHAR_BASE_BLOCK_SUB(0), 16);
}


//---------------------------------------------------------------------------------
THREAD_RET
thread_send_commands (THREAD_PARAMS data)
{
  //---------------------------------------------------------------------------------
  while(1)
    {
      ATcodec_Queue_Message_valist(ids.AT_MSG_ATCMD_CGMI);
      vp_os_delay(INTRA_CMD_DELAY);
      ATcodec_Queue_Message_valist(ids.AT_MSG_ATCMD_CGMI);
      vp_os_delay(INTRA_CMD_DELAY);
      //vp_os_thread_yield();

      ATcodec_Queue_Message_valist(ids.AT_MSG_ATCMD_PM_QW,1);
      vp_os_delay(INTRA_CMD_DELAY);
      ATcodec_Queue_Message_valist(ids.AT_MSG_ATCMD_PM_QW,1);
      vp_os_delay(INTRA_CMD_DELAY);
      //vp_os_thread_yield();

      ATcodec_Queue_Message_valist(ids.AT_MSG_ATCMD_PM_EXE,1,50);
      vp_os_delay(INTRA_CMD_DELAY);
      ATcodec_Queue_Message_valist(ids.AT_MSG_ATCMD_PM_EXE,1,50);
      vp_os_delay(INTRA_CMD_DELAY);
      //vp_os_thread_yield();
    }
}


//---------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //---------------------------------------------------------------------------------
  touchPosition touchXY;
  THREAD_HANDLE atcodec_test_handle;
  THREAD_HANDLE atcodec_test_handle2;
  THREAD_HANDLE cmds_handle;

  initNDS();

  AT_CODEC_FUNCTIONS_PTRS ptrs =
    {
      .init     = AT_CODEC_Client_init,
      .shutdown = AT_CODEC_Client_shutdown,
      .open     = AT_CODEC_Client_open,
      .close    = AT_CODEC_Client_close,
      .read     = AT_CODEC_Client_read,
      .write    = AT_CODEC_Client_write,
    };

  ATcodec_Init_Library(&ptrs);

  vp_os_thread_create(thread_ATcodec_Commands_Client, 0, &atcodec_test_handle);
  vp_os_thread_create(thread_send_commands, 0, &cmds_handle);
  vp_os_thread_create(thread_ATcodec_Commands_Server, 0, &atcodec_test_handle2);

  vp_os_thread_join(cmds_handle);
  vp_os_thread_join(atcodec_test_handle2);
  vp_os_thread_join(atcodec_test_handle);

  iprintf("      Hello DS dev'rs\n");
  iprintf("     www.devkitpro.org\n");
  iprintf("   www.drunkencoders.com");

  while(1) {
	
    swiWaitForVBlank();
    touchXY=touchReadXY();

    // print at using ansi escape sequence \x1b[line;columnH
    iprintf("\x1b[10;0HFrame = %d",frame);
    iprintf("\x1b[16;0HTouch x = %04X, %04X\n", touchXY.x, touchXY.px);
    iprintf("Touch y = %04X, %04X\n", touchXY.y, touchXY.py);
	
  }

  return EXIT_SUCCESS;
}

