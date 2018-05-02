/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012-2015(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <xc.h>
#include "Communication.h"
#include "pin_manager.h"

/***************************************************************************//**
 * @brief Initializes the I2C communication peripheral.
 *
 * @param clockFreq - I2C clock frequency (Hz).
 *                    Example: 100000 - I2C clock frequency is 100 kHz.
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char I2C_Init(unsigned long clockFreq)
{
    unsigned char   status      = 0;
    unsigned long   pbFrequency = 4000000;
    unsigned short  brgValue    = 0;
    
    // R_nW write_noTX; P stopbit_notdetected; S startbit_notdetected; BF RCinprocess_TXcomplete; SMP High Speed; UA dontupdate; CKE disabled; D_nA lastbyte_address; 
    SSP1STAT = 0x00;
    // SSPEN enabled; WCOL no_collision; CKP Idle:Low, Active:High; SSPM FOSC/4_SSPxADD_I2C; SSPOV no_overflow; 
    SSP1CON1 = 0x28;
    // ACKTIM ackseq; SBCDE disabled; BOEN disabled; SCIE disabled; PCIE disabled; DHEN disabled; SDAHT 100ns; AHEN disabled; 
    SSP1CON3 = 0x00;
    // SSP1ADD 3;
//    brgValue = pbFrequency/(4*clockFreq) - 1;
//    SSP1ADD = brgValue;
    SSP1ADD = 0x03;
    
    // clear the interrupt flags
    PIR1bits.SSP1IF = 0;
    PIR2bits.BCL1IF = 0;
	
    // enable the interrupts
    PIE1bits.SSP1IE = 1;
    PIE2bits.BCL1IE = 1;
    
    CS_SetHigh();
    status = 1;
    
    return status;
}

/***************************************************************************//**
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Address of the slave device.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes to write.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
 *
 * @return status - Number of written bytes.
*******************************************************************************/
unsigned char I2C_Write(unsigned char slaveAddress,
                        unsigned char* dataBuffer,
                        unsigned char bytesNumber,
                        unsigned char stopBit)
{
    unsigned char status = 0;
    unsigned char acknowledge = 0;
    unsigned char byte = 0;
    
    
   SSP1CON2bits.SEN = 1;        //initiate start condition
   while(SSP1CON2bits.SEN);     //Wait for the start condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = slaveAddress << 1;          //Send the slave address and R/W bit
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag  
   acknowledge = SSP1CON2bits.ACKSTAT;
   if (acknowledge == 0)
   {
       for(byte = 0; byte < bytesNumber; byte++)
        {
            SSP1BUF = dataBuffer[byte];
            while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
            PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
        }
   }
   else
   {
       status = 0xFF;           //exit Read (NACK)
   }     
   if(stopBit)
   {
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete
   }
   
   return status;
}

/***************************************************************************//**
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Address of the slave device.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes to read.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
 *
 * @return status - Number of read bytes.
*******************************************************************************/
unsigned char I2C_Read(unsigned char slaveAddress,
                       unsigned char* dataBuffer,
                       unsigned char bytesNumber,
                       unsigned char stopBit)
{
    unsigned char status = bytesNumber;
    unsigned char acknowledge = 0;
    unsigned char byte = 0;
    
   SSP1CON2bits.RSEN = 1;        //initiate restart condition
   while(SSP1CON2bits.RSEN);     //Wait for the restart condition to complete
   PIR1bits.SSP1IF = 0;         //clear SSP interrupt flag
   
   SSP1BUF = (slaveAddress << 1) + 1;          //Send the slave address and R/W bit
   while(!PIR1bits.SSP1IF);     //wait for ACK. SSPIF is set every 9th clock cycle
   PIR1bits.SSP1IF = 0;          //Clear SSP Interrupt Flag
   acknowledge = SSP1CON2bits.ACKSTAT;
   if (acknowledge == 0)
   {
       for(byte = 0; byte < bytesNumber; byte++)
        {
           SSP1CON2bits.RCEN = 1;       //receive the data byte from the slave
           while(!SSP1STATbits.BF);     //wait for the receive to complete
           dataBuffer[byte] = SSPBUF;
           if(byte == (bytesNumber - 1))
            {
               SSP1CON2bits.ACKDT = 1;  //prepare to send the NACK
            }
            else
            {
               SSP1CON2bits.ACKDT = 0;
            }
            SSP1CON2bits.ACKEN = 1;     //initiate NACK to accel
            while(SSP1CON2bits.ACKEN);  //wait for the NACK to complete
        }
   }
   else
   {
       status = 0xFF;           //exit Read (NACK)
   }
       SSP1CON2bits.PEN = 1;    //initiate Stop condition
       while(SSP1CON2bits.PEN); //wait for start condition to complete

    return status;
}

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{
    unsigned char   status      = 0;
    unsigned long   pbFrequency = 40000000;
    unsigned short  brgValue    = 0;
    
    CS_SetHigh();
    
    // Set the SPI module to the options selected in the User Interface
    
    // R_nW write_noTX; P stopbit_notdetected; S startbit_notdetected; BF RCinprocess_TXcomplete; SMP Middle; UA dontupdate; CKE Idle to Active; D_nA lastbyte_address; 
    SSP1STAT = 0x00;
    SSP1STATbits.CKE = clockEdg;
    
    // SSPEN enabled; WCOL no_collision; CKP Idle:Low, Active:High; SSPM FOSC/4; SSPOV no_overflow; 
    SSP1CON1 = 0x20;
    SSP1CON1bits.CKP = clockPol;
    
    // SSP1ADD 0;
//    brgValue = pbFrequency / (4 * clockFreq) - 1;
//    SSP1ADD = brgValue;
    SSP1ADD = 0x00;
    
    status = 1;
    return status;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
    unsigned char   byte            = 0;
    unsigned char   writeBuffer[4]  = {0, 0, 0, 0};
    
    
    SSP1CON1bits.WCOL = 0;  // Clear the Write Collision flag, to allow writing
    
    for(byte = 0; byte < bytesNumber; byte++)
    {
        writeBuffer[byte] = data[byte];
    }
    if(slaveDeviceId == 1)
    {
        CS_SetLow();
    }
    for(byte = 0; byte < bytesNumber; byte++)
    {
        SSP1BUF = writeBuffer[byte];
        while(SSP1STATbits.BF == 0);
        data[byte] = SSP1BUF;
    }
//    while(SPI2STATbits.SPIBUSY == 1);
    if(slaveDeviceId == 1)
    {
        CS_SetHigh();
    }

    return bytesNumber;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
    unsigned char byte     = 0;
    unsigned char tempByte = 0;

    if(slaveDeviceId == 1)
    {
        CS_SetLow();
    }
    for(byte = 0; byte < bytesNumber; byte++)
    {
        SSP1BUF = data[byte];
        while(SSP1STATbits.BF == 0);
        tempByte = SSP1BUF;
    }
//    while(SPI2STATbits.SPIBUSY == 1);
    if(slaveDeviceId == 1)
    {
        CS_SetHigh();
    }

    return bytesNumber;
}
/***************************************************************************//**
 * @brief Initializes the UART communication peripheral.
 *
 * @param baudRate - Baud rate value.
 *                   Example: 9600 - 9600 bps.
 *
 * @return status  - Result of the initialization procedure.
 *                   Example: -1 - if initialization was unsuccessful;
 *                             0 - if initialization was successful.
*******************************************************************************/
void UART_Init(void)
{
    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE disabled; ABDEN disabled; 
    BAUD1CON = 0x08;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RC1STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    TX1STA = 0x24;

    // SP1BRGL 103; 
    SPBRGL = 0x67;

    // SP1BRGH 0; 
    SPBRGH = 0x00;

}

/***************************************************************************//**
 * @brief Writes one character to UART.
 *
 * @param data - Character to write.
 *
 * @return None.
*******************************************************************************/
void UART_WriteChar(char data)
{
    while(0 == PIR1bits.TXIF){}
    TX1REG = data;    // Write the data byte to the USART.
}

/***************************************************************************//**
 * @brief Reads one character from UART.
 *
 * @param data - Read character.
 *
 * @return None.
*******************************************************************************/
void UART_ReadChar(char* data)
{
    while(!PIR1bits.RCIF){}
     if(1 == RC1STAbits.OERR)
    {
        // EUSART error - restart
        RC1STAbits.CREN = 0; 
        RC1STAbits.CREN = 1; 
    }
    *data = RC1REG;
}

/***************************************************************************//**
 * @brief Writes one string of characters to UART.
 *
 * @param string - String of characters to write.
 *
 * @return None.
*******************************************************************************/
void UART_WriteString(const char* string)
{
    while(*string)
    {
        UART_WriteChar(*string++);
    }
}