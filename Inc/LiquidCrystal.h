#ifndef LiquidCrystal_h
#define LiquidCrystal_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "i2c.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

//Push Pull IO
#define RS_IO		0    
#define RW_IO		1   
#define E_IO		2

//Open Drain IO
#define BL_IO		3   

#define DB4_IO	4    
#define DB5_IO	5   	
#define DB6_IO	6    
#define DB7_IO	7   

class LiquidCrystal{
public:
	LiquidCrystal(void);

	void init(void);
    
  void SetPos(uint8_t l,uint8_t hori)
	{
		const static uint8_t pos_tab[] = {0x80, 0xc0};
		
		WrCmd_4(pos_tab[l] + hori);	
	}

	void Display(uint8_t l,uint8_t hori,uint8_t *s)
	{
		SetPos(l, hori);
		while(*s)
		{
			WrDat_4(*s);
			s++;
		}
	}
	
	static void SimpleDelay(uint32_t t);
	static const uint8_t LINE_NUM = 2;
	
private:		
	uint8_t _data;
	uint8_t _r_data;

	void RS_H(void)
	{
		_data |= (1<<RS_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void RS_L(void)
	{
		_data &= ~(1<<RS_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void RW_H(void)
	{
		_data |= (1<<RW_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void RW_L(void)
	{
		_data &= ~(1<<RW_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void E_H(void)
	{
		_data |= (1<<E_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void E_L(void)
	{
		_data &= ~(1<<E_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D4_H(void)
	{
		_data |= (1<<DB4_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D4_L(void)
	{
		_data &= ~(1<<DB4_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D5_H(void)
	{
		_data |= (1<<DB5_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D5_L(void)
	{
		_data &= ~(1<<DB5_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D6_H(void)
	{
		_data |= (1<<DB6_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D6_L(void)
	{
		_data &= ~(1<<DB6_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	void D7_H(void)
	{
		_data |= (1<<DB7_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);	
	}

	void D7_L(void)
	{
		_data &= ~(1<<DB7_IO);
		i2c_write(PCF8574A_ADDR, &_data, 1);		
	}

	uint32_t ReadD4(void)
	{
		_r_data = i2c_receive(PCF8574A_ADDR);		
		return (_r_data & (1<<DB4_IO));
	}

	uint32_t ReadD5(void)
	{
		_r_data = i2c_receive(PCF8574A_ADDR);		
		return (_r_data & (1<<DB5_IO));
	}

	uint32_t ReadD6(void)
	{
		_r_data = i2c_receive(PCF8574A_ADDR);		
		return (_r_data & (1<<DB6_IO));
	}

	uint32_t ReadD7(void)
	{
		_r_data = i2c_receive(PCF8574A_ADDR);		
		return (_r_data & (1<<DB7_IO));
	}
	
	void DB4_Wr(uint8_t dat)
	{
		(0==(dat&0x08))?D7_L():D7_H();
		(0==(dat&0x04))?D6_L():D6_H();
		(0==(dat&0x02))?D5_L():D5_H();
		(0==(dat&0x01))?D4_L():D4_H();
		SimpleDelay(10);
	}

	void WaitAvail(void)
	{
		uint32_t tmpTimeOut = 12000;
		
		DB4_Wr(0xff);
		SimpleDelay(10);

		RS_L(); 
		SimpleDelay(10);
		RW_H();
		SimpleDelay(10);
		E_H();
		SimpleDelay(10);
		while((0!=ReadD7()) && (0!=tmpTimeOut))
		{
			__NOP();
			--tmpTimeOut;
		}
		SimpleDelay(10);
		E_L(); 
	}

	void WrCmd_4 (uint8_t cmd)
	{
		WaitAvail();

		SimpleDelay(10);

		RS_L(); 
		SimpleDelay(10);
		RW_L();   	
		SimpleDelay(10);
		DB4_Wr(cmd>>4);
		SimpleDelay(10);
		E_H();
		SimpleDelay(10);
		E_L();
		SimpleDelay(10);
		DB4_Wr(cmd);
		SimpleDelay(10);
		E_H();
		SimpleDelay(10);
		E_L();	
		SimpleDelay(10);
	}

	void WrDat_4 (uint8_t dat)
	{
		WaitAvail();
		SimpleDelay(10);		
		RS_H(); 
		SimpleDelay(10);		
		RW_L();   	
		SimpleDelay(10);		
		DB4_Wr(dat>>4);
		SimpleDelay(10);		
		E_H();
		SimpleDelay(10);		
		E_L();
		SimpleDelay(10);		
		DB4_Wr(dat);
		SimpleDelay(10);		
		E_H();
		SimpleDelay(10);		
		E_L();		
		SimpleDelay(10);		
	}	

};

#ifdef __cplusplus
}
#endif

#endif
