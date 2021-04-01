/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.170.0
        Device            :  dsPIC33CK256MP206
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.61
        MPLAB 	          :  MPLAB X v5.45
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include <stdbool.h>
#include "SSD1306_serial.h"

//  Function prototypes
bool timedinit(void);
void ms_tick(void);

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
/*-----------------------------------------------------------------------------
 *      For testing of the I2C oled module the SA0 line is set Low.
 *      For the module being tested the SA0 line is the D/C pin on oled
 *          module.
 *      So  RD3  -->  DISP_DC --> D/C pin --> SA0  for SSD1306.
 *      With SA0 low the I2C addres is 0xA8.
 *----------------------------------------------------------------------------*/
    LATDbits.LATD3 = 0;            
    TRISDbits.TRISD3 = 0;
    //  SSD1306 I/O and module setup.
    display_pinsetup();
    display_init();
    //  SSD1306 oled timed startup.
    while(timedinit());
    
    while (1) {
        // TMR1  set to 1 mSecond time interval 
        if (IFS0bits.T1IF) {
            IFS0bits.T1IF = false;
            ms_tick();
        }
    }
    return 1;     
}

/*-----------------------------------------------------------------------------
 *      Timed display startup.
 *---------------------------------------------------------------------------*/
bool timedinit(void){
    static uint16_t inittime = 0;
    if(IFS0bits.T1IF)
    {       
        IFS0bits.T1IF = false;
        inittime++;     // 1 ms tick.
       
    }
    switch(inittime){
        
        case 10:
            display_resetpin(false);
            break;
            
        case 12:
            display_resetpin(true);
            break;
            
        case 50:
            display_start();
            break;
            
        case 100:
            display_clearall();
            break;
            
        case 500:
            return false;
            break;        
        default:
            break;            
    }
    return true;
}

/*-----------------------------------------------------------------------------
 *      Test display.  Called every .001 seconds.
 *---------------------------------------------------------------------------*/
uint8_t example[] = {
   'M','e','t','a','l',
   'P','l','a','s','t','i','c',
   'E','l','e','c','t','r','o','n','i','c','s',
   '0','1','2','3','4','5','6','7','8','9','(',')',' '
};
void ms_tick(){
    static uint16_t count = 0;  
    static uint16_t pause = 0;
    count++;

    if (count == 500) {
        count = 0;
        if(pause > 0){
            pause--;
            return;
        }
        //  go through the character set.
        static uint16_t ch = 0;
        static uint8_t col = 4;
        static uint8_t row = 0;
        
#define SHOW_EXAMPLE    // if not defined displays each char in char set.


#ifndef SHOW_EXAMPLE        
        display_charat(ch, col, row);
        ch++;
        if (ch > 133) {
            ch = 0;
        }
        col += 10;
        if (col > 120) {
            col = 4;
            row += 2;
            if (row > 7) {
                row = 0;
            }
        }           
#else
        display_charat(example[ch], col, row);        
        ch++;
        col += 10;
        if(ch == 5){
            col = 4;
            row = 2;
        }
        else if(ch == 12){
            col = 4;
            row = 4;
        }
        else if(ch == 23){
            col = 4;
            row = 6;                        
        }
        else if(ch == 35){
            col = 100;
            row = 0;
            pause = 4;
        }
        else if(ch == 36){
            ch = 0;
            col = 4;
            row = 0;
            display_clearall();
            pause = 1;                    
        }     
#endif 
    }
}
/**
 End of File
*/

