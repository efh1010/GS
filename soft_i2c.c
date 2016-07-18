/*
 * File Name: soft_i2c.c
 * Author: Siddharth Chandrasekaran
 * Website	: http://embedjournnal.com/
 * Created on July 27, 2012, 12:12 PM
*/

#include "soft_i2c.h"
#include "mcc_generated_files/mcc.h"  
#include <xc.h>  

unsigned char i2cRead(void)
{
   unsigned char inByte, n;
   i2cHighSda();
   for (n=0; n<8; n++)
   {
      i2cHighScl();

      if (SDA_PIN == 1)
         inByte = (inByte << 1) | 0x01; // msbit first
      else
         inByte = inByte << 1;
      i2cLowScl();
   }
   return(inByte);
}

void i2cWrite(unsigned char outByte)
{
   unsigned char n;
   for(n=0; n<8; n++)
   {
      if(outByte&0x80)
         i2cHighSda();
      else
         i2cLowSda();
      i2cHighScl();
      i2cLowScl();
      outByte = outByte << 1;
   }
   i2cHighSda();
   
    i2cHighScl();
    i2cLowScl();
    __delay_us(3);
}

void i2cNack(void)
{
   i2cHighSda();
   i2cHighScl();
   __delay_us(3);
   i2cLowScl();		// bring data high and clock
   __delay_us(3);
}

void i2cAck(void)
{
   i2cLowSda();
   __delay_us(3);
   i2cHighScl();
   __delay_us(3);
   i2cLowScl();
   __delay_us(3);
   i2cHighSda();		// bring data low and clock
   __delay_us(3);
}

void i2cWaitAck()
{
    i2cHighSda();
    i2cLowScl();
    __delay_us(3);
    i2cHighScl();
    __delay_us(3);
    i2cLowScl();
}

void i2cStart(void)
{
    i2cHighSda();
    i2cHighScl();
    __delay_us(3);
    i2cLowSda();
    __delay_us(3);
    i2cLowScl();
    __delay_us(3);
}

void i2cStop(void)
{
    i2cLowSda();
    __delay_us(3);
    i2cHighScl();
    __delay_us(3);
    i2cHighSda();
    __delay_us(3);
}

void i2cHighSda(void)
{
   SDA_DIR = 1;		// bring SDA to high impedance
   __delay_us(5);
}

void i2cLowSda(void)
{
   SDA_PIN = 0;
   SDA_DIR = 0;		// output a logic zero
   __delay_us(5);
}

void i2cHighScl(void)
{
   SCL_DIR = 1;		// bring SCL to high impedance
   __delay_us(5);
}

void i2cLowScl(void)
{
   SCL_PIN = 0;		
   SCL_DIR = 0;
   __delay_us(5);
}

uint8_t MasterRead(uint8_t address,uint8_t mem_address,uint8_t data[],uint8_t counterMax)
{
    address = address << 1;

    int i;
      
        for (i=0;i<counterMax;i++){
            i2cStart();
            i2cWrite(address);
            i2cWrite(mem_address+i);
            i2cStart();
            i2cWrite(address+1);
            data[i] = i2cRead();
            i2cNack();
            i2cStop();         
        }
    return 1;
}


uint8_t MasterWrite(uint8_t address,uint8_t mem_address,uint8_t data[],uint8_t counterMax)
{
address = address << 1;

    int i;
    
        for (i=0;i<counterMax;i++){
            i2cStart();
            i2cWrite(address);
            i2cWrite(mem_address+i);
            i2cWrite(data[i]);
            i2cStop();         
        }
    return 1;
}

void WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t value)
{
    i2cStart();
    i2cWrite(devAddr);
    i2cWrite(regAddr);
    i2cWrite(value);
    i2cStop(); 
}