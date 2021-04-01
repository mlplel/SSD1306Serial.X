/**
 @Description:
    SSD1306  serial OLED display interface.

 @File Name:
    SSD1306_serial.h
 @Version 
  1.0

  @Summary:
 Will need one of the SSD1306_XXX.c files.
 And need the SSD1306_charsets.c file.
  
*/

/*
    Copyright 2021 Michael Jacobs

    Permission is hereby granted, free of charge, to any person obtaining a 
    copy of this software and associated documentation files (the "Software"), 
    to deal in the Software without restriction, including without limitation 
    the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included 
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
    OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

#ifndef SSD1306_SERIAL_H
#define	SSD1306_SERIAL_H

#include <xc.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
extern const uint8_t chset10x16[134][20]; 
    
/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * 
 *  Select serial mode and serial module to use here.
 *  Only 1 display module is supported.  
 * 
 * ----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
//#define SSD1306_USE_I2C        // use SSD1306_I2C.c
//#define SSD1306_USE_SPI3W      // use SSD1306_SPI3.c
#define SSD1306_USE_SPI4W       // use SSD1306_SPI4.c
    
#define SSD1306_USE_RES     // if module uses reset pin
    
#ifdef SSD1306_USE_I2C      // Select I2C module to use here.
    //#define SSD1306_I2C_USE_I2C1
    #define SSD1306_I2C_USE_I2C2 
    //#define SSD1306_I2C_USE_I2C3
#endif

#ifdef SSD1306_USE_SPI3W    // Select spi 3 wire module to use here.
    #define SSD1306_SPI3_USE_SPI1
    //#define SSD1306_SPI3_USE_SPI2
    //#define SSD1306_SPI3_USE_SPI3
#endif
#ifdef SSD1306_USE_SPI4W    // Select spi 4 wire module to use here.
    #define SSD1306_SPI4_USE_SPI1
    //#define SSD1306_SPI4_USE_SPI2
    //#define SSD1306_SPI4_USE_SPI3   
  
#endif   
   
 /**
  @Description
    Set up PPS and IO pins. This has to be first thing done.
    The I/O pins used have to assigned here.
  @Param
    None.
  @Returns
    None
  @Example
    Best to move this code to the I/O setup of program.      
*/ 
void display_pinsetup(void);


/**
  @Description
    Initalizes the serial module used.
    Does not initalize the OLED module.
  @Param
    None.
  @Returns
    None
  @Example    
*/
void display_init(void);

/**
  @Description
    reset pin control for the module.
  @Param
    level  false = low,   true = high.
  @Returns
    None
  @Example
*/
void display_resetpin(bool level);


/**
  @Description
    Initalizes the OLED module.  
  @Param
    None.
  @Returns
    None
  @Example 
    Call after the oled module is reset.
*/
void display_start(void);


/**
  @Description
    Sets all the display memory on module to 0.  Pixel off in normal mode.
  @Param
    None.
  @Returns
    None
  @Example 
    Call after display_poweron() to clear display memory.
*/
void display_clearall(void);


/**
  @Description
    Display a character from chset10x16 array.
  @Param
    ch is character from array to display. 
    x is left side of chr in display horizontal pixels 0 - 127.
    y is top of chr in display vertical rows 0 - 7.
  @Returns
    None
  @Example    
*/
void display_charat(uint16_t ch, uint8_t x, uint8_t y);



#ifdef	__cplusplus
}
#endif

#endif	/* SSD1306_SERIAL_H */

