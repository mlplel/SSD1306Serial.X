/******************************************************************************
 @Description:
    SSD1306  I2C OLED display interface.
  
 @Version 
  1.0

 
 @File Name:
    SSD1306_I2C.c

  
*******************************************************************************/

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

#include "SSD1306_serial.h"
#include <stdlib.h>
#include <stdbool.h>

#ifdef SSD1306_USE_I2C

typedef enum {
    DISPSTATE_IDLE,
    DISPSTATE_COMMAND,
    DISPSTATE_STARTDATA,
    DISPSTATE_DATA,
    DISPSTATE_SENDADDR,
    DISPSTATE_STOP,
    DISPSTATE_BUSY        
} DISP_STATE;

typedef enum {
    DISPCMDTYPE_STD,
    DISPCMDTYPE_FIXED,
    DISPCMDTYPE_REPEAT
} DISPCMDTYPE;

typedef enum  {
    DISPLAYDATA_SIZE = 32,
    DISPLAYDATA_BLOCKSIZE = 40
} DISPLAYDATASIZES;

typedef struct {
    uint16_t active;
    uint8_t data[DISPLAYDATA_SIZE];
} DISPLAYDATA;

typedef enum {
    DATABLOCKSTATUS_EMPTY,
    DATABLOCKSTATUS_FULL,
    DATABLOCKSTATUS_ACTIVE
} DATABLOCKSTATUS;

typedef struct {
    DISPCMDTYPE type;
    uint16_t command_length;
    uint16_t data_length;
    uint8_t *pdata;    
}DISP_DATABLOCK;

/*
 *  Internal function prototypes  
 */
void disp_memory_init();
uint8_t* disp_memory_alloc();
void disp_memory_free(uint8_t* p);
DATABLOCKSTATUS disp_write(DISPCMDTYPE t, uint16_t cmd_l, uint16_t data_l, uint8_t *p);
void i2c_stop(void);

//  buffer for memory allocation.
static DISPLAYDATA dispmem[DISPLAYDATA_BLOCKSIZE];          
volatile static DISP_STATE dispstate = DISPSTATE_IDLE;
// circular display buffer.
static DISP_DATABLOCK dispdatablocks[DISPLAYDATA_BLOCKSIZE];
volatile static DISP_DATABLOCK *startblock;
volatile static DISP_DATABLOCK *endblock;
static  DISP_DATABLOCK activeblock;
volatile static DATABLOCKSTATUS datablockstatus;
static uint8_t *pbuffer = NULL;
// set i2c address here.
static uint8_t address = 0x78;   // 0x3C or 0x3D << 1 = 0x78 0r 0x7A

#ifdef SSD1306_I2C_USE_I2C1

#define I2CCONL I2C1CONL
#define I2CCONLbits I2C1CONLbits
#define I2CSEN I2C1CONLbits.SEN
#define I2CPEN I2C1CONLbits.PEN
#define I2CCONH I2C1CONH
#define I2CSTAT I2C1STAT
#define I2CSTATbits I2C1STATbits
#define I2CSTATIWCOL I2C1STATbits.IWCOL
#define I2CSTATACK I2C1STATbits.ACKSTAT
#define I2CBRG I2C1BRG
#define I2CTRN I2C1TRN

#define MI2CInterrupt _MI2C1Interrupt
#define IRQM_PRIORITY IPC4bits.MI2C1IP
#define IRQM_F IFS1bits.MI2C1IF
#define IRQM_E IEC1bits.MI2C1IE

#endif

#ifdef SSD1306_I2C_USE_I2C2

#define I2CCONL I2C2CONL
#define I2CCONLbits I2C2CONLbits
#define I2CSEN I2C2CONLbits.SEN
#define I2CPEN I2C2CONLbits.PEN
#define I2CCONH I2C2CONH
#define I2CSTAT I2C2STAT
#define I2CSTATbits I2C2STATbits
#define I2CSTATIWCOL I2C2STATbits.IWCOL
#define I2CSTATACK I2C2STATbits.ACKSTAT
#define I2CBRG I2C2BRG
#define I2CTRN I2C2TRN

#define MI2CInterrupt _MI2C2Interrupt
#define IRQM_PRIORITY IPC9bits.MI2C2IP
#define IRQM_F IFS2bits.MI2C2IF
#define IRQM_E IEC2bits.MI2C2IE

#endif

#ifdef SSD1306_I2C_USE_I2C3

#define I2CCONL I2C3CONL
#define I2CCONLbits I2C3CONLbits
#define I2CSEN I2C3CONLbits.SEN
#define I2CPEN I2C3CONLbits.PEN
#define I2CCONH I2C3CONH
#define I2CSTAT I2C3STAT
#define I2CSTATbits I2C3STATbits
#define I2CSTATIWCOL I2C3STATbits.IWCOL
#define I2CSTATACK I2C3STATbits.ACKSTAT
#define I2CBRG I2C3BRG
#define I2CTRN I2C3TRN

#define MI2CInterrupt _MI2C3Interrupt
#define IRQM_PRIORITY IPC35bits.MI2C3IP
#define IRQM_F IFS8bits.MI2C3IF
#define IRQM_E IEC8bits.MI2C3IE

#endif

/******************************************************************************
 *
 *                  DISPLAY PIN ASSIGNMENT SECTION
 * 
 *          Assignment of mcu I/O to OLED module pins.
 * 
 *          Current:    RD4 --> OLED active low reset pin.
 *                      RC5/ASCL2 --> OLED D0 SCL clock in pin.
 *                      RC4/ASDA2 --> OLED D1 SDA pin.
 * 
 *****************************************************************************/
/*-----------------------------------------------------------------------------
 * 
 * DISP_RES pin:
 *          Active low for reset of OLED module.  3 usec min low time.
 *          This I/O pin is connected to the RES pin on OLED module.
 * 
 *----------------------------------------------------------------------------*/
#ifdef SSD1306_USE_RES
    #define DISP_RES_in     PORTDbits.RD4
    #define DISP_RES_out    LATDbits.LATD4
#endif


/*-----------------------------------------------------------------------------
 * 
 *  Assign I/O pins for the module used.
 *  See output selection for remappable pins in datasheet.  
 *  If possible move this to main I/O setup section of program.
 * 
 *----------------------------------------------------------------------------*/
void display_pinsetup(void){
    //  fill in for IO pins used.
#ifdef SSD1306_USE_RES
    LATDbits.LATD4 = 1;             // RD4 --> DISP_RES    
    TRISDbits.TRISD4 = 0;
#endif
    
    ODCCbits.ODCC4 = 1;     // set to OpenDrain mode
    ODCCbits.ODCC5 = 1;     // want SDA and SCL pulled high when I2c 
                            // module is turned on to avoid bus 
                            // collision error being set.
    
    // Reminder to set I/O pins to digital if have analog function.
    // ANSELxbits.XXXXX  
}

/*-----------------------------------------------------------------------------
 * 
 *  Called to setup SPI module, memory and buffer routines.
 * 
 *----------------------------------------------------------------------------*/
void display_init(void){
    
    disp_memory_init();
    startblock = dispdatablocks;
    endblock = dispdatablocks;
    datablockstatus = DATABLOCKSTATUS_EMPTY;
 
#ifdef SSD1306_USE_RES    
    DISP_RES_out = 1;
#endif
    
    //I2CBRG = 0x1EB;     // 0x1EB baud rate set for 100KHz
    //I2CCONL = 0x0200;   // enable i2c 7bit addr
    
    I2CBRG = 0x73;      // 400Khz baud rate
    I2CCONL = 0x0000;   // enable 12c 7bit addr.
    I2CCONH = 0x0000;
    I2CSTAT = 0x00; 
    I2CCONLbits.I2CEN = 1;
    
    IRQM_F = 0;
    IRQM_E = 1;
}

/*-----------------------------------------------------------------------------
 *  I2C Master interrupt.  
 *  interrupt on different i2c states.
 *----------------------------------------------------------------------------*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) MI2CInterrupt ( void ){
    
    static bool contl = true;
    IRQM_F = 0;         // clear interrupt flag
    
    if(I2CSTATIWCOL){        
        I2CSTATIWCOL = 0;
        // write collision occured.  Will probably have to reset
        // I2C module here.  Should only occur if multiple deviecs on bus 
        // or there is an electrical problem.
        dispstate = DISPSTATE_IDLE;        
    }
    
    switch(dispstate){
        case DISPSTATE_IDLE:
            if(datablockstatus != DATABLOCKSTATUS_EMPTY){
                // there is data to send.
                activeblock.type = startblock->type;
                activeblock.command_length = startblock->command_length;
                activeblock.data_length = startblock->data_length;
                activeblock.pdata = startblock->pdata;
                pbuffer = activeblock.pdata;
                startblock++;
                if(startblock == (dispdatablocks + DISPLAYDATA_BLOCKSIZE)){
                    startblock = dispdatablocks;
                }
                datablockstatus = DATABLOCKSTATUS_ACTIVE;
                if(startblock == endblock){
                    datablockstatus = DATABLOCKSTATUS_EMPTY;
                } 
                dispstate = DISPSTATE_SENDADDR;
                I2CSEN = 1;         // send start.
            }
            break;
        case DISPSTATE_SENDADDR:
            // send address with r/w bit cleared
            I2CTRN = address;
            contl = true;
            if(activeblock.command_length > 0){
                dispstate = DISPSTATE_COMMAND;
            }
            else if(activeblock.data_length > 0){
                dispstate = DISPSTATE_STARTDATA;
            }
            else {
                dispstate = DISPSTATE_STOP;
            }
            break;
        case DISPSTATE_COMMAND:
            if(I2CSTATACK){     // no acknowledge from display
                i2c_stop();
            }
            if(contl){
                I2CTRN = 0x80;   // control byte is command
                contl = false;
            }
            else {
                contl = true;
                I2CTRN = *activeblock.pdata;
                activeblock.pdata++;
                activeblock.command_length--;
                if(activeblock.command_length == 0){
                    // command bytes are finished.
                    if(activeblock.data_length == 0){
                        dispstate = DISPSTATE_STOP;
                    }
                    else {
                        dispstate = DISPSTATE_STARTDATA;
                    }                    
                }
            }
            break;
        case DISPSTATE_STARTDATA:
            if(I2CSTATACK){     // no acknowledge from display
                i2c_stop();
            }
            I2CTRN = 0x40;      //control byte is start data
            dispstate = DISPSTATE_DATA;
            break;
        case DISPSTATE_DATA:
            if(I2CSTATACK){     // no acknowledge from display
                i2c_stop();
            }
            I2CTRN = *activeblock.pdata;
            if(activeblock.type != DISPCMDTYPE_REPEAT){
                activeblock.pdata++;
            }
            activeblock.data_length--;
            if(activeblock.data_length == 0){
                dispstate = DISPSTATE_STOP;
            }
            break;
        case DISPSTATE_STOP:
            i2c_stop();
            break;           
        default:
            break;
    }
}

/*
 * 
 */
void i2c_stop(void){    
    I2CPEN = 1;         // set to stop
    dispstate = DISPSTATE_IDLE;
    if(activeblock.type == DISPCMDTYPE_STD){
        disp_memory_free(pbuffer);
    }    
}


/*-----------------------------------------------------------------------------
 *      Initialize memory allocation.
 *----------------------------------------------------------------------------*/
void disp_memory_init(){
    for(int i = 0; i < DISPLAYDATA_BLOCKSIZE; i++){
        dispmem[i].active = 0;
    }
}

/*-----------------------------------------------------------------------------
 *      set reset pin to level  true = high  false = low.
 *----------------------------------------------------------------------------*/
void display_resetpin(bool level){
#ifdef SSD1306_USE_RES 
    DISP_RES_out = level;
#endif
}

/*-----------------------------------------------------------------------------
 *  Returns a uint8_t pointer to a DISPLAYDATA_SIZE buffer on success.
 *  Returns NULL on failure. 
 *----------------------------------------------------------------------------*/
uint8_t* disp_memory_alloc(){
    for(int i = 0; i < DISPLAYDATA_BLOCKSIZE; i++){
        if(dispmem[i].active == 0){
            dispmem[i].active = 1;
            return &(dispmem[i].data[0]);
        }
    }
    return NULL;
}

/*-----------------------------------------------------------------------------
 *  Free a previously allocated buffer returned from disp_memory_alloc().
 *----------------------------------------------------------------------------*/
void disp_memory_free(uint8_t* p){
    for(int i = 0; i < DISPLAYDATA_BLOCKSIZE; i++){
        if(p == &(dispmem[i].data[0])){
            // location found free it.
            dispmem[i].active = 0;
            return;
        }
    }
}

/*-----------------------------------------------------------------------------
 * 
 * add display command and data to circular buffer if not full.
 * t -->  command type.
 * cmd_l -->  length of command in bytes.
 * data_l --> length of data in bytes.
 * p --> pointer to command and data bytes.
 *
 * returns status either FULL or ACTIVE. 
 *----------------------------------------------------------------------------*/
DATABLOCKSTATUS disp_write(DISPCMDTYPE t, uint16_t cmd_l, uint16_t data_l, uint8_t *p){
    if(datablockstatus == DATABLOCKSTATUS_FULL){
        if(t == DISPCMDTYPE_STD){
            disp_memory_free(p);
        }
        return DATABLOCKSTATUS_FULL;
    }
    // assign data to next open data block.
    endblock->type = t;
    endblock->command_length = cmd_l;
    endblock->data_length = data_l;
    endblock->pdata = p;
    endblock++;
    
    if(endblock == (dispdatablocks + DISPLAYDATA_BLOCKSIZE)){
        endblock = dispdatablocks;
    }
    datablockstatus = DATABLOCKSTATUS_ACTIVE;
    if(endblock == startblock){
        datablockstatus = DATABLOCKSTATUS_FULL;
    }
    
    if(dispstate == DISPSTATE_IDLE){
        // force interrupt to start spi data transfer
        IRQM_F = 1;        
    }
    return DATABLOCKSTATUS_ACTIVE;    
}


/*------------------------------------------------------------------------------
 *  Initialize and set parameters of OLED display.
 *----------------------------------------------------------------------------*/
static uint8_t ssd1306set[25] = {
    0xAE, // turn off
    0xD5, 0x80, // set clock / oscillator 
    0xA8, 0x3F, // set multiplex ratio
    0xD3, 0x00, // set display offset
    0x8D, 0x14, // set charge pump 
    0x40, // set display ram start line
    0xA6, // normal display
    0xA4, // display from ram
    0xA1, // set segment remap
    0xC8, // set com scan direction
    0xDA, 0x12, // set com pins configuration
    0x81, 0x80, // set contrast
    0xD9, 0xF1, // set pre-charge period
    0xDB, 0x40, // set Vcomh
    0x20, 0x00, // set memory address mode --> horizontal.
    0xAF // turn on
};
void display_start(void){       
    disp_write(DISPCMDTYPE_FIXED, 25, 0, ssd1306set);    
}


/*------------------------------------------------------------------------------
 *  Clear all display memory
 *----------------------------------------------------------------------------*/
void display_clearall(void){
    static uint8_t cd1[7] = {
        0x21, 0x00, 0x7F,
        0x22, 0x00, 0x07,
        0x00
    };    
    disp_write(DISPCMDTYPE_REPEAT, 6, 1024, cd1);    
}


/*-----------------------------------------------------------------------------
 *  ch = index to chset.
 *  x = horizontal pos in pixels.  0 - 127
 *  y = row position.              0 - 7
 *----------------------------------------------------------------------------*/
void display_charat(uint16_t ch, uint8_t x, uint8_t y){
    
    uint8_t* pdata;
    if((pdata = disp_memory_alloc()) == NULL) {
        //memory alloc error.
        return;
    }
    
    *(pdata + 0) = 0x21;
    *(pdata + 1) = x;
    *(pdata + 2) = (x + 9);
    *(pdata + 3) = 0x22;
    *(pdata + 4) = y;
    *(pdata + 5) = (y + 1);
    
    for(int i = 0; i < 20; i++){
        *(pdata + i + 6) = chset10x16[ch][i];
    }    
    disp_write(DISPCMDTYPE_STD, 6, 20, pdata);   
}

#endif
/**
 end of file
*/
 