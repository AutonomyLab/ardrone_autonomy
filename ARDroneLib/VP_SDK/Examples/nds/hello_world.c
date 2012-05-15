/**
 * \file examples_oneThread.c
 * \brief How to launch a thread with VP_SDK
 */

///////////////////////////////////////////////
// INCLUDES

#include <nds.h>
#include <stdio.h>


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
int main(int argc, char **argv)
{
  //---------------------------------------------------------------------------------
  touchPosition touchXY;

  initNDS();

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

  return 0;
}

