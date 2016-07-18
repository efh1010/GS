/* 
 * File:   BMP180.c
 * Author: francescotesolin
 *
 * Created on February 29, 2016, 9:03 PM
 */
 
 
 
 
#include <stdint.h>                     /* For uint8_t definition */
#include <xc.h>                         /* XC8 General Include File */
#include <stdbool.h>                    /* For true/false definition */
#include "soft_i2c.h"
#include "BMP180.h"
#include ""





//DEFINE i2c address
uint8_t BMP180_address = 0xEE;
uint8_t AC1_address = 0xAA;
uint8_t reg_num = 22;




//DEFINE CALIBRATION REGISTER
	uint8_t calibration_reg[22];
	uint8_t ut_read[2];
	uint8_t oss=0;
	uint8_t up_read[3];
	
	
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
	
	int32_t ut;
	int32_t up;
	
	int32_t x1;
	int32_t x2;
	int32_t b5;
	int32_t t;
	
	int32_t b6;
	int32_t x3;
	int32_t b3;
	uint32_t b4;
	uint32_t b7;
	int32_t p;








/*      READ INTERNAL CALIBRATION REGISTER  */

void readCalibrationRegister(void){


	MasterRead(BMP180_address,AC1_address,calibration_reg,reg_num);
    
    ac1=(int16_t)(calibration_reg[0]<<8) + calibration_reg[1];
	ac2=(int16_t)(calibration_reg[2]<<8) + calibration_reg[3];
	ac3=(int16_t)(calibration_reg[4]<<8) + calibration_reg[5];

	ac4=(uint16_t)(calibration_reg[6]<<8) + calibration_reg[7];
	ac5=(uint16_t)(calibration_reg[8]<<8) + calibration_reg[9];
	ac6=(uint16_t)(calibration_reg[10]<<8) + calibration_reg[11];

	b1=(int16_t)(calibration_reg[12]<<8) + calibration_reg[13];
	b2=(int16_t)(calibration_reg[14]<<8) + calibration_reg[15];
	mb=(int16_t)(calibration_reg[16]<<8) + calibration_reg[17];
	mc=(int16_t)(calibration_reg[18]<<8) + calibration_reg[19];
	md=(int16_t)(calibration_reg[20]<<8) + calibration_reg[21];
	
}







int32_t readUncompensatedTemperature(void){

	WriteReg(BMP180_address,0xF4,0x2E);
	__delay_ms(5);  //4.5 valore corretto
    
	MasterRead(BMP180_address,0xF6,ut_read,2);
	ut = (int32_t) (ut_read[0]<<8) + ut_read[1];
    
    return ut;
}








int32_t readUncompensatedPressure(void){
    
	WriteReg(BMP180_address,0xF4,(0x34 /*+ (oss<<6)*/));
	__delay_ms(10); //4.5ms valore corretto
    
	MasterRead(BMP180_address,0xF6,up_read,3);
	//up = (int32_t)((((int32_t)up_read[0] << 16) + ((int32_t)up_read[1] <<8) + ((int32_t)up_read[2])) >> (8 - oss));
    up = (int32_t) ((int32_t)up_read[0]<<8) + up_read[1];
    return up;
}









int32_t calculateTrueTemperature(void){
            
    x1 = (int32_t)(((int32_t)ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
	x2 = (int32_t)((int32_t)mc << 11)/((int32_t)x1 + (int32_t)md);
	b5 = (int32_t)x1 + (int32_t)x2;
	t = (((int32_t)b5 + 8)>>4);
    
    return t;
}








int32_t calculateTruePressure(void){
    
	b6 = (int32_t)((int32_t)b5 - 4000);
	x1 =(int32_t)(((int32_t)b2 * ( ((int32_t)b6 * (int32_t)b6) >> 12) ) >> 11 );
	x2 = (int32_t)(((int32_t)ac2 * (int32_t)b6) >> 11);
	x3 = (int32_t)((int32_t)x1 + (int32_t)x2) ;
	b3 =(int32_t)(((((int32_t)ac1 * 4 + (int32_t)x3 ) << (int32_t)oss )+2)/4);
	
	x1 = (int32_t)(((int32_t)ac3 * (int32_t)b6) >> 13);
	x2 =(int32_t)(((int32_t)b1 * ( ((int32_t)b6 * (int32_t)b6) >> 12)) >> 16);
	x3 =(int32_t)(((( (int32_t)x1 + (int32_t)x2 ) + 2 )) >> 2);
	b4 = (uint32_t)((uint32_t)ac4 * ((uint32_t)x3 + 32768) >> 15);
	b7 = (uint32_t)((( uint32_t ) up - (uint32_t)b3 ) * ( 50000 >> (uint32_t)oss ));
	if ( b7 < 0x80000000 ) {
		p = (int32_t)(( (int32_t)b7 * 2 ) / (int32_t)b4);
	}
	else{
		p = (int32_t)(( (int32_t)b7 / (int32_t)b4 ) * 2);
	}
	
	x1 = (int32_t)(((int32_t)p >> 8) * ((int32_t)p >> 8));
	
	x1 = (int32_t)(( (int32_t)x1 * 3038) >> 16);
	x2 = (int32_t)((-7357 * (int32_t)p) >> 16);
	p  = (int32_t)((int32_t)p + (( (int32_t)x1 + (int32_t)x2 + 3791) >> 4));
}








uint8_t readBMP180(int32_t data_read[]){
    
    readCalibrationRegister();
    
    readUncompensatedTemperature();
    readUncompensatedPressure();
    
    data_read[0] = calculateTrueTemperature();
    data_read[1] = calculateTruePressure();
    
    
	return 1;
}





void serialPrint_BMP180(void){
	uint8_t i=0;
	int32_t temp;			
	uint8_t display;		
	
	
	EUSART1_Write(0xAA);  	// START MARKER
	
	for (i=0;i<22;i++){
		EUSART1_Write(calibration_reg[i]);
		}
	
	EUSART1_Write(0xAA); 		//MARKER
	
	for (i=0;i<2;i++){
		EUSART1_Write(ut_read[i]);
		}	
		
	for (i=0;i<3;i++){
		EUSART1_Write(up_read[i]);
		}
		
	EUSART1_Write(0xAA); 		//MARKER
	
	temp=t;
	temp=temp/10;
	display=(uint8_t) temp;
	EUSART1_Write(display);
	
	temp=p;
	temp=temp/1000;
	display=(uint8_t) temp;
	EUSART1_Write(display);
	
}


void print_8bit(int32_t data){
    uint8_t print;
    
    
}