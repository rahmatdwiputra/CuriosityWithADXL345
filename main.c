/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65
        Device            :  PIC16F1619
        Driver Version    :  2.00
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
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

#include "mcc.h"

void main(void)
{
    unsigned char i2cStatus;
    //ADXL345 data register
    short x;
    short y;
    short z;
    unsigned char DeviceID;
    //MMA8491Q data register
    char        AccelStatus;
    char        TempXMSB;
    char        TempXLSB;
    char        TempYMSB;
    char        TempYLSB;
    char        TempZMSB;
    char        TempZLSB;
    int16_t    AccelX = 0;
    int16_t    AccelY = 0;
    int16_t    AccelZ = 0;
     int16_t   AccelXtemp = 0;
    int16_t    AccelYtemp = 0;
    int16_t    AccelZtemp = 0;
    char counter = 0;
 
     // initialize the device
    SYSTEM_Initialize();
    i2cStatus = ADXL345_Init(ADXL345_I2C_COMM);
    
    //I2C GPIO init state
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
//    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
//    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    
    while (1)
    {
//    ADXL345_GetXyz(&x,&y,&z);
//    __delay_ms(5);
        
//        LED_D4_SetHigh();
//        LED_D7_SetLow();
//        __delay_ms(500);
//        DeviceID = ADXL345_GetRegisterValue(0x00);
        AccelStatus = AccelReadStatus(0x1D,0x00);
        __delay_ms(5);
//        if(AccelStatus & 0b00001000)
//        {
////        AccelReadData(MMA8491Q_ADDRESS,MMA8491Q_OUTZ_MSB,&TempXMSB,&TempXLSB);
//        AccelReadDataAll(MMA8491Q_ADDRESS,MMA8491Q_OUTX_MSB,&TempXMSB,&TempXLSB,&TempYMSB,&TempYLSB,&TempZMSB,&TempZLSB);
//        AccelXtemp = AccelConversion(TempXMSB,TempXLSB);
//        AccelYtemp = AccelConversion(TempYMSB,TempYLSB);
//        AccelZtemp = AccelConversion(TempZMSB,TempZLSB);
//        }
//        AccelX += AccelXtemp; 
//        AccelY += AccelYtemp;
//        AccelZ += AccelZtemp;
//        counter++;
//        if(counter == AVERAGE){
//            AccelX/= AVERAGE;
//            AccelY/= AVERAGE;
//            AccelZ/= AVERAGE;
//            counter = 0;
//        }
        
        
//        __delay_ms(500);
//        LED_D4_SetLow();
//        LED_D7_SetHigh();
//        __delay_ms(500);
//        
        
    }
}



char AccelReadDataAll(char SlaveAdd, char DataAdd, char *DataXMSB, char *DataXLSB,char *DataYMSB, char *DataYLSB,char *DataZMSB, char *DataZLSB)
{
    char TempData;
   SSP1CON2bits.SEN = 1;        //initiate start condition
   while(SSP1CON2bits.SEN);     //Wait for the start condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = SlaveAdd << 1;          //Send the slave address and R/W bit
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1BUF = DataAdd;          //Send the register address
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1CON2bits.RSEN = 1;        //initiate restart condition
   while(SSP1CON2bits.RSEN);     //Wait for the restart condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = ((SlaveAdd << 1) + 1);          //Send the slave address and R/W bit = 1
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataXMSB = SSPBUF;           //save the data byte
   
   SSP1CON2bits.ACKDT = 0;     //prepare to send the ACK
   SSP1CON2bits.ACKEN = 1;     //initiate ACK to accel
   while(SSP1CON2bits.ACKEN);  //wait for the ACK to complete
    
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataXLSB = SSPBUF;           //save the data byte
   
   SSP1CON2bits.ACKDT = 0;     //prepare to send the ACK
   SSP1CON2bits.ACKEN = 1;     //initiate ACK to accel
   while(SSP1CON2bits.ACKEN);  //wait for the ACK to complete
    
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataYMSB = SSPBUF;           //save the data byte
   
   SSP1CON2bits.ACKDT = 0;     //prepare to send the ACK
   SSP1CON2bits.ACKEN = 1;     //initiate ACK to accel
   while(SSP1CON2bits.ACKEN);  //wait for the ACK to complete
    
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataYLSB = SSPBUF;           //save the data byte
   
   SSP1CON2bits.ACKDT = 0;     //prepare to send the ACK
   SSP1CON2bits.ACKEN = 1;     //initiate ACK to accel
   while(SSP1CON2bits.ACKEN);  //wait for the ACK to complete
    
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataZMSB = SSPBUF;           //save the data byte
   
   SSP1CON2bits.ACKDT = 0;     //prepare to send the ACK
   SSP1CON2bits.ACKEN = 1;     //initiate ACK to accel
   while(SSP1CON2bits.ACKEN);  //wait for the ACK to complete
    
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataZLSB = SSPBUF;           //save the data byte
   
    SSP1CON2bits.ACKDT = 1;     //prepare to send the NACK
    SSP1CON2bits.ACKEN = 1;     //initiate NACK to accel
    while(SSP1CON2bits.ACKEN);  //wait for the NACK to complete
    
    SSP1CON2bits.PEN = 1;       //initiate Stop condition
    while(SSP1CON2bits.PEN);    //wait for the stop condition to complete
    CS_SetLow();
    return TempData;            //return the Data Byte value
}

char AccelReadData(char SlaveAdd, char DataAdd, char *DataMSB, char *DataLSB){
    char TempData;
   SSP1CON2bits.SEN = 1;        //initiate start condition
   while(SSP1CON2bits.SEN);     //Wait for the start condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = SlaveAdd << 1;          //Send the slave address and R/W bit
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1BUF = DataAdd;          //Send the register address
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1CON2bits.RSEN = 1;        //initiate restart condition
   while(SSP1CON2bits.RSEN);     //Wait for the restart condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = ((SlaveAdd << 1) + 1);          //Send the slave address and R/W bit = 1
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataMSB = SSPBUF;           //save the data byte
   
   SSP1CON2bits.ACKDT = 0;     //prepare to send the ACK
   SSP1CON2bits.ACKEN = 1;     //initiate ACK to accel
   while(SSP1CON2bits.ACKEN);  //wait for the ACK to complete
    
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   *DataLSB = SSPBUF;           //save the data byte
   
    SSP1CON2bits.ACKDT = 1;     //prepare to send the NACK
    SSP1CON2bits.ACKEN = 1;     //initiate NACK to accel
    while(SSP1CON2bits.ACKEN);  //wait for the NACK to complete
    
    SSP1CON2bits.PEN = 1;       //initiate Stop condition
    while(SSP1CON2bits.PEN);    //wait for the stop condition to complete
    
    CS_SetLow();
    return TempData;            //return the Data Byte value
}



char AccelReadStatus(char SlaveAdd, char DataAdd)
{
    char TempData;
    CS_SetHigh();
   __delay_ms(1);
   SSP1CON2bits.SEN = 1;        //initiate start condition
   while(SSP1CON2bits.SEN);     //Wait for the start condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = SlaveAdd << 1;          //Send the slave address and R/W bit
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1BUF = DataAdd;          //Send the register address
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1CON2bits.RSEN = 1;        //initiate restart condition
   while(SSP1CON2bits.RSEN);     //Wait for the restart condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = ((SlaveAdd << 1) + 1);          //Send the slave address and R/W bit = 1
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   if (SSP1CON2bits.ACKSTAT)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
       return (0xFF);           //exit Read (NACK)
   }
   
   SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
   while(!SSP1STATbits.BF);     //wait for the receive to complete
   TempData = SSPBUF;           //save the data byte
   
    SSP1CON2bits.ACKDT = 1;     //prepare to send the NACK
    SSP1CON2bits.ACKEN = 1;     //initiate NACK to accel
    while(SSP1CON2bits.ACKEN);  //wait for the NACK to complete
    
    SSP1CON2bits.PEN = 1;       //initiate Stop condition
    while(SSP1CON2bits.PEN);    //wait for the stop condition to complete
    return TempData;            //return the Data Byte value
        
}