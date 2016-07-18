/*
 * File Name: soft_i2c.h
 * Author: Siddharth Chandrasekaran
 * Website	: http://embedjournnal.com/
 * Created on July 27, 2012, 12:12 PM
*/

#include<xc.h>
#include<stdint.h>




#define SDA_PIN PORTCbits.RC4
#define SCL_PIN PORTCbits.RC3	
#define SDA_DIR TRISCbits.RC4
#define SCL_DIR TRISCbits.RC3

unsigned char i2cRead();
void i2cWrite(unsigned char outByte);
void i2cNack();
void i2cAck();
void i2cStart();
void i2cStop();
void i2cWaitAck();


void i2cHighSda();
void i2cLowSda();
void i2cHighScl();
void i2cLowScl();

void WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t value);
uint8_t MasterRead(uint8_t address,uint8_t mem_address,uint8_t data[],uint8_t counterMax);
uint8_t MasterWrite(uint8_t address,uint8_t mem_address,uint8_t data[],uint8_t counterMax);

