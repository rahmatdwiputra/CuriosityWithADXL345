/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.35
        Device            :  PIC16F1619
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA2 procedures
#define RA2_SetHigh()    do { LATAbits.LATA2 = 1; } while(0)
#define RA2_SetLow()   do { LATAbits.LATA2 = 0; } while(0)
#define RA2_Toggle()   do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define RA2_GetValue()         PORTAbits.RA2
#define RA2_SetDigitalInput()   do { TRISAbits.TRISA2 = 1; } while(0)
#define RA2_SetDigitalOutput()  do { TRISAbits.TRISA2 = 0; } while(0)
#define RA2_SetPullup()     do { WPUAbits.WPUA2 = 1; } while(0)
#define RA2_ResetPullup()   do { WPUAbits.WPUA2 = 0; } while(0)
#define RA2_SetAnalogMode() do { ANSELAbits.ANSA2 = 1; } while(0)
#define RA2_SetDigitalMode()do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set SDA aliases
#define SDA_TRIS               TRISBbits.TRISB4
#define SDA_LAT                LATBbits.LATB4
#define SDA_PORT               PORTBbits.RB4
#define SDA_WPU                WPUBbits.WPUB4
#define SDA_OD                ODCONBbits.ODB4
#define SDA_ANS                ANSELBbits.ANSB4
#define SDA_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define SDA_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define SDA_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define SDA_GetValue()           PORTBbits.RB4
#define SDA_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define SDA_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define SDA_SetPullup()      do { WPUBbits.WPUB4 = 1; } while(0)
#define SDA_ResetPullup()    do { WPUBbits.WPUB4 = 0; } while(0)
#define SDA_SetPushPull()    do { ODCONBbits.ODB4 = 0; } while(0)
#define SDA_SetOpenDrain()   do { ODCONBbits.ODB4 = 1; } while(0)
#define SDA_SetAnalogMode()  do { ANSELBbits.ANSB4 = 1; } while(0)
#define SDA_SetDigitalMode() do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set SCL aliases
#define SCL_TRIS               TRISBbits.TRISB6
#define SCL_LAT                LATBbits.LATB6
#define SCL_PORT               PORTBbits.RB6
#define SCL_WPU                WPUBbits.WPUB6
#define SCL_OD                ODCONBbits.ODB6
#define SCL_ANS                ANSELBbits.ANSB6
#define SCL_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define SCL_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define SCL_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define SCL_GetValue()           PORTBbits.RB6
#define SCL_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define SCL_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define SCL_SetPullup()      do { WPUBbits.WPUB6 = 1; } while(0)
#define SCL_ResetPullup()    do { WPUBbits.WPUB6 = 0; } while(0)
#define SCL_SetPushPull()    do { ODCONBbits.ODB6 = 0; } while(0)
#define SCL_SetOpenDrain()   do { ODCONBbits.ODB6 = 1; } while(0)
#define SCL_SetAnalogMode()  do { ANSELBbits.ANSB6 = 1; } while(0)
#define SCL_SetDigitalMode() do { ANSELBbits.ANSB6 = 0; } while(0)

// get/set LED_D7 aliases
#define LED_D7_TRIS               TRISCbits.TRISC5
#define LED_D7_LAT                LATCbits.LATC5
#define LED_D7_PORT               PORTCbits.RC5
#define LED_D7_WPU                WPUCbits.WPUC5
#define LED_D7_OD                ODCONCbits.ODC5
#define LED_D7_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define LED_D7_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define LED_D7_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define LED_D7_GetValue()           PORTCbits.RC5
#define LED_D7_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define LED_D7_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define LED_D7_SetPullup()      do { WPUCbits.WPUC5 = 1; } while(0)
#define LED_D7_ResetPullup()    do { WPUCbits.WPUC5 = 0; } while(0)
#define LED_D7_SetPushPull()    do { ODCONCbits.ODC5 = 0; } while(0)
#define LED_D7_SetOpenDrain()   do { ODCONCbits.ODC5 = 1; } while(0)

// get/set CS aliases
#define CS_TRIS               TRISCbits.TRISC6
#define CS_LAT                LATCbits.LATC6
#define CS_PORT               PORTCbits.RC6
#define CS_WPU                WPUCbits.WPUC6
#define CS_OD                ODCONCbits.ODC6
#define CS_ANS                ANSELCbits.ANSC6
#define CS_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define CS_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define CS_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define CS_GetValue()           PORTCbits.RC6
#define CS_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define CS_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define CS_SetPullup()      do { WPUCbits.WPUC6 = 1; } while(0)
#define CS_ResetPullup()    do { WPUCbits.WPUC6 = 0; } while(0)
#define CS_SetPushPull()    do { ODCONCbits.ODC6 = 0; } while(0)
#define CS_SetOpenDrain()   do { ODCONCbits.ODC6 = 1; } while(0)
#define CS_SetAnalogMode()  do { ANSELCbits.ANSC6 = 1; } while(0)
#define CS_SetDigitalMode() do { ANSELCbits.ANSC6 = 0; } while(0)


/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/