/******************************************************************************
 @Description:
    SSD1306  SPI 4 wire OLED display interface.
  
 @Version 
  1.0

 
 @File Name:
    SSD1306_SPI4.c

  
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

#ifdef SSD1306_USE_SPI4W

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
bool spi_fill_buffer(void);
void spi_block_finished(void);

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



#ifdef SSD1306_SPI4_USE_SPI1
    
#define SPICON1L SPI1CON1L
#define SPICON1Lbits SPI1CON1Lbits
#define SPICON1H SPI1CON1H
#define SPICON2L SPI1CON2L
#define SPISTATL SPI1STATL
#define SPISTATLbits SPI1STATLbits
#define SPISTATH SPI1STATH
#define SPIIMSKL SPI1IMSKL
#define SPIIMSKH SPI1IMSKH
#define SPIBUFL SPI1BUFL
#define SPIBRGL SPI1BRGL    
    
#define SPIRXInterrupt _SPI1RXInterrupt
#define SPITXInterrupt _SPI1TXInterrupt
#define SPIInterrupt  _SPI1Interrupt 

#define IRQTX_PRIORITY IPC2bits.SPI1TXIP
#define IRQTX_F IFS0bits.SPI1TXIF
#define IRQTX_E IEC0bits.SPI1TXIE

#define IRQRX_PRIORITY IPC2bits.SPI1RXIP
#define IRQRX_F IFS0bits.SPI1RXIF
#define IRQRX_E IEC0bits.SPI1RXIE

#define IRQG_PRIORITY IPC31bits.SPI1IP
#define IRQG_F IFS7bits.SPI1IF
#define IRQG_E IEC7bits.SPI1IE
    
#endif

#ifdef SSD1306_SPI4_USE_SPI2
    
#define SPICON1L SPI2CON1L
#define SPICON1Lbits SPI2CON1Lbits
#define SPICON1H SPI2CON1H
#define SPICON2L SPI2CON2L
#define SPISTATL SPI2STATL
#define SPISTATLbits SPI2STATLbits
#define SPISTATH SPI2STATH
#define SPIIMSKL SPI2IMSKL
#define SPIIMSKH SPI2IMSKH
#define SPIBUFL SPI2BUFL
#define SPIBRGL SPI2BRGL    
    
#define SPIRXInterrupt _SPI2RXInterrupt
#define SPITXInterrupt _SPI2TXInterrupt
#define SPIInterrupt  _SPI2Interrupt 

#define IRQTX_PRIORITY IPC7bits.SPI2TXIP
#define IRQTX_F IFS1bits.SPI2TXIF
#define IRQTX_E IEC1bits.SPI2TXIE

#define IRQRX_PRIORITY IPC7bits.SPI2RXIP
#define IRQRX_F IFS1bits.SPI2RXIF
#define IRQRX_E IEC1bits.SPI2RXIE

#define IRQG_PRIORITY IPC31bits.SPI2IP
#define IRQG_F IFS7bits.SPI2IF
#define IRQG_E IEC7bits.SPI2IE
    
#endif

#ifdef SSD1306_SPI4_USE_SPI3
    
#define SPICON1L SPI3CON1L
#define SPICON1Lbits SPI3CON1Lbits
#define SPICON1H SPI3CON1H
#define SPICON2L SPI3CON2L
#define SPISTATL SPI3STATL
#define SPISTATLbits SPI3STATLbits
#define SPISTATH SPI3STATH
#define SPIIMSKL SPI3IMSKL
#define SPIIMSKH SPI3IMSKH
#define SPIBUFL SPI3BUFL
#define SPIBRGL SPI3BRGL    
    
#define SPIRXInterrupt _SPI3RXInterrupt
#define SPITXInterrupt _SPI3TXInterrupt
#define SPIInterrupt  _SPI3Interrupt 

#define IRQTX_PRIORITY IPC15bits.SPI3TXIP
#define IRQTX_F IFS3bits.SPI3TXIF
#define IRQTX_E IEC3bits.SPI3TXIE

#define IRQRX_PRIORITY IPC14bits.SPI3RXIP
#define IRQRX_F IFS3bits.SPI3RXIF
#define IRQRX_E IEC3bits.SPI3RXIE

#define IRQG_PRIORITY IPC32bits.SPI3IP
#define IRQG_F IFS8bits.SPI3IF
#define IRQG_E IEC8bits.SPI3IE
    
#endif

/*-----------------------------------------------------------------------------
 *  Set SPI baud rate here. See datasheet and chip Fp clock for setting.
 *----------------------------------------------------------------------------*/
#define SSD1306_SPI_BAUD_VALUE 0x04         // 10 Mhz

/******************************************************************************
 *
 *                  DISPLAY PIN ASSIGNMENT SECTION
 * 
 *          Assignment of mcu I/O to OLED module pins.
 * 
 *          Current:    RD2 --> OLED active low CS pin.  
 *                      RD3 --> OLED D/C low command, high data pin.
 *                      RD4 --> OLED active low reset pin.
 *                      RC4/RP52 --> OLED D1 SDIN pin.
 *                      RC5/RP53 --> OLED D0 SCLK pin.
 * 
 *****************************************************************************/
/*-----------------------------------------------------------------------------
 * 
 * DISP_DC pin:  
 *          Active low for command. Active high for data.
 *          This I/O pin is connected to the DC pin on 4 wire spi OLED module.
 * 
 * DISP_RES pin:
 *          Active low for reset of OLED module.  3 usec min low time.
 *          This I/O pin is connected to the RES pin on OLED module.
 * 
 * DISP_CS pin:
 *          Active low chip select for spi bus.
 *          This I/O pin is connected to the CS pin on OLED module.
 * 
 *----------------------------------------------------------------------------*/
#define DISP_DC_in      PORTDbits.RD3
#define DISP_DC_out     LATDbits.LATD3

#define DISP_RES_in     PORTDbits.RD4
#define DISP_RES_out    LATDbits.LATD4

#define DISP_CS_in      PORTDbits.RD2
#define DISP_CS_out     LATDbits.LATD2

/*-----------------------------------------------------------------------------
 * 
 *  Assign I/O pins for the SPI module used.
 *  See output selection for remappable pins in datasheet.  
 *  If possible move this to main I/O setup section of program.
 * 
 *----------------------------------------------------------------------------*/
void display_pinsetup(void){
    //  fill in for IO pins used.
    LATDbits.LATD2 = 1;             // RD2 --> DISP_CS
    LATDbits.LATD3 = 0;             // RD3 --> DISP_DC
    LATDbits.LATD4 = 1;             // RD4 --> DISP_RES
    
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;
    
    // ANSELxbits.XXXXX    reminder to set I/O pins to digital.    
    
    __builtin_write_RPCON(0x0000);       // unlock PPS
     
    //  ------ remember to change values if using different SPI modules. ------
    RPOR10bits.RP52R = 0x05;    // RP52 -- RC4 =  SDO for SPI1
    RPOR10bits.RP53R = 0x06;    // RP53 -- RC5 =  SCK for SPI1 
    
    __builtin_write_RPCON(0x0800);      // lock PPS
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
    
    DISP_RES_out = 1;
    DISP_CS_out = 1;
    DISP_DC_out = 0;
    
    // IGNROV = 1 and IGNTUR = 1.  
    SPICON1H = 0x3000;
    SPICON2L = 0x0000;
    
    // shift register empty generates an interrupt
    SPIIMSKL = 0x0088;
    SPIIMSKH = 0x0000;
    SPIBRGL = SSD1306_SPI_BAUD_VALUE;
    
    SPICON1L = 0x0071;
    SPICON1Lbits.SPIEN = 1;
    
    //IRQTX_PRIORITY = 1;
    IRQG_PRIORITY = 1;
    //IRQTX_F = 0;
    IRQG_F = 0;
    //IRQTX_E = 1;       
    IRQG_E = 1;
}


/*-----------------------------------------------------------------------------
 *  SPI general interrupt routine.  
 *  interrupt when shift register is empty
 *----------------------------------------------------------------------------*/
void __attribute__ ( ( interrupt, no_auto_psv ) )  SPIInterrupt ( void ){ 
    IRQG_F = 0;
    
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
                
                if(activeblock.command_length > 0){  
                    dispstate = DISPSTATE_COMMAND;
                    DISP_DC_out = 0;
                    // D/C line is in command state
                    if(spi_fill_buffer()){  
                        // all command data has been sent.
                        if(activeblock.data_length == 0){ 
                            // all date from this block has been sent.
                            spi_block_finished();
                        }
                        else {
                            // now have to switch to data mode.
                            dispstate = DISPSTATE_DATA;
                        } 
                    } 
                }
                else {
                    // in this implementation should not get here.
                    if(activeblock.data_length > 0){
                        // data only block.
                        dispstate = DISPSTATE_DATA;
                        DISP_DC_out = 1;
                        // D/C line is in data state.
                        if(spi_fill_buffer()){
                            // all data has been sent this block is finished.
                            spi_block_finished();
                        }                            
                    }
                }                
            }
            break;
        case DISPSTATE_COMMAND:
            DISP_DC_out = 0;
            if(spi_fill_buffer()){
                // all command data has been sent.
                if(activeblock.data_length == 0){
                    // all data from this block has been sent.
                    spi_block_finished();
                }
                else {
                    // now have to switch to data mode.
                    dispstate = DISPSTATE_DATA;
                }
            }
            break;
        case DISPSTATE_DATA:
            DISP_DC_out  = 1;
            if(spi_fill_buffer()){
                spi_block_finished();
            }            
            break;
        default:
            // should not get here
            break;
    } 
}
// disabled
void __attribute__ ( ( interrupt, no_auto_psv ) ) SPIRXInterrupt ( void ){ 
    IRQRX_F = 0;
}
//  disabled
void __attribute__ ( ( interrupt, no_auto_psv ) ) SPITXInterrupt ( void ){   
    IRQTX_F = 0;    
}


/*------------------------------------------------------------------------------
 *      returns true when have reached the end of command bytes or data bytes.
 *----------------------------------------------------------------------------*/
bool spi_fill_buffer(){
    if(dispstate == DISPSTATE_COMMAND){
        while(1){
            SPIBUFL = *activeblock.pdata;
            activeblock.pdata++;
            activeblock.command_length--;
            if(activeblock.command_length == 0){
                // command date send is finished.
                return true;
            }
            if(SPISTATLbits.SPITBF == 1){
                // buffer is full 
                return false;
            }
        } 
    }
    else if(dispstate == DISPSTATE_DATA){
        while(1){
            SPIBUFL = *activeblock.pdata;
            if(activeblock.type != DISPCMDTYPE_REPEAT){
                activeblock.pdata++;
            }
            activeblock.data_length--;
            if(activeblock.data_length == 0){
                // data send is finished.
                return true;
            }
            if(SPISTATLbits.SPITBF == 1){
                // buffer is full 
                return false;
            }            
        }  
    } 
    return false;
}

/*
 *  Returns dispstate to IDLE and frees allocated buffer.
 */
void spi_block_finished(){
    
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
    DISP_RES_out = level;
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
        IRQG_F = 1;        
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
    DISP_CS_out = 0;                // CS active.    
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