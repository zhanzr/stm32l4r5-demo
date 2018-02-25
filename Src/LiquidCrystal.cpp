#include "LiquidCrystal.h"

void LiquidCrystal::SimpleDelay(uint32_t t)
{
	uint32_t d = t * 1;
	while(--d)
	{
		__NOP();
	}
}

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the Micro controller doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LiquidCrystal::LiquidCrystal(void)
{
	init();
}

void LiquidCrystal::init(void)
{
	SimpleDelay(10);	
	
	E_L();
	SimpleDelay(10);
	
	//Change to 4 bit mode, reepeat for fault tolerance
	WrCmd_4(0x28);
	SimpleDelay(10);
	WrCmd_4(0x28);
	SimpleDelay(10);
	WrCmd_4(0x28);
	SimpleDelay(10);
	
	WrCmd_4(0x08);
	SimpleDelay(10);
	WrCmd_4(0x01);	
	SimpleDelay(10);
	WrCmd_4(0x06);
	SimpleDelay(10);
	WrCmd_4(0x0c);
	SimpleDelay(10);
}
