/*
TITLE: driver GPS-610F for PIC18F26K22 
AUTHOR: francescotesolin SKYWARD - TEAM GS
DESCRIPTION: 

*/

#include "gps610f.h"
#include <xc.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "serialprint.h"


//$GPGGA,141829.000,2400.00004,N,12100.00004,E,0,00,0.0,0.00,M,0.00,M,,0000*6E


uint8_t readGPS610F(gpsgga *gps){

	uint8_t data[DIMDATA]={',','1','4','1','8','2','9','.','0','0','0',',','2','4','0','0','.','2','3','0','0','4',',','N',',','1','2','1','0','0','.','4','3','0','0','4',',','E',',','0',',','1','1',',','6','.','8',',','5','.','8','6',',','M',',','0','.','0','0',',','M',',',',','0','0','0','0','*','6','E',};
	uint8_t marker[DIMMARKER]={'$','G','P','G','G','A'};
	uint8_t temp[DIMDATA];
	uint8_t dim = 0;
	uint8_t i = 0;	
	uint8_t status = 0;
	uint8_t counter = 0;
	

/* lettura MARKER */
	
	do{ 
		if(readMarker(marker) == 1){
			status = controlMarker(marker);
		}
		counter ++;
	} while(status != 1 && counter <= 30);
		
	if (counter >30) return 0; //errore





/* lettura DATA */

	status = readData(data);
	if (status != 1) return 0; // errore
	
	
	
	
/* DECODIFICA */
	
// 1) TIME
	dim = popCommaToComma(data,temp);
	for(i = 0; i < dim && i < DIMTIME; i++){
		(gps -> time[i]) = temp[i];
	}
	
// 2) LATITUDE
	dim = popCommaToComma(data,temp);
	(gps -> latitude) = stringToFloat(temp,dim);
	
// 3) N/S
	dim = popCommaToComma(data,temp);
	(gps -> ns) = temp[0];

// 4) LONGITUDE
	dim = popCommaToComma(data,temp);
	(gps -> longitude) = stringToFloat(temp,dim);

// 5) E/W
	dim = popCommaToComma(data,temp);
	(gps -> ew) = temp[0];

// 6) GPS Quality
	dim = popCommaToComma(data,temp);
	(gps -> quality) = (temp[0]-48); 

// 7) n satellites
	dim = popCommaToComma(data,temp);
	(gps -> n_satellites) =  stringToUint8_t(temp, dim);

// 8) Horizontal Dilution of precision
	dim = popCommaToComma(data,temp);
	(gps -> horizontal_dilution) = stringToFloat(temp,dim);


// 9) Antenna Altitude above/below mean-sea-level (geoid)
	dim = popCommaToComma(data,temp);
	(gps -> altitude) = stringToFloat(temp,dim);
 	
// 	10) Units of antenna altitude, meters
	dim = popCommaToComma(data,temp);
	(gps -> units_altitude) = temp[0];

// 	11) Geoidal separation
	dim = popCommaToComma(data,temp);
	(gps -> geoidal_separation) = stringToFloat(temp,dim);
	
// 	12) Units of geoidal separation, meters
	dim = popCommaToComma(data,temp);
	(gps -> units_geoidal_separation) = temp[0];

// 	13) Age of differential GPS data
// 	printf("13) Age of differential GPS data\n");
// 	dim=popCommaToComma(data,temp);
// 	printf("%d\n",dim);
// 	printString(temp,DIMDATA);
// 	printString(data,DIMDATA);
// 	printf("\n");
// 	
// 	
// 	14) Differential reference station ID, 0000-1023
// 	printf("14) Differential reference station ID, 0000-1023\n");
// 	dim=popCommaToComma(data,temp);
// 	printf("%d\n",dim);
// 	printString(temp,DIMDATA);
// 	printString(data,DIMDATA);
// 	printf("\n");
// 	
// 	
// 	15) Checksum
// 	printf("15) Checksum\n");
// 	dim=popCommaToComma(data,temp);
// 	printf("%d\n",dim);
// 	printString(temp,DIMDATA);
// 	printString(data,DIMDATA);
// 	printf("\n");
// 	


}






void printString(uint8_t string[], uint8_t dim){
	uint8_t i = 0;
	
	for(i=0; i<dim; i++){
		EUSART1_Write(string[i]);
	}
	//printf("\n");

}






uint8_t controlMarker(uint8_t string[]){
    if(string[1]=='G' && string[2]=='P' && string[3]=='G' && string[4]=='G' && string[5]=='A') {
        return 1;
    }
    
    return 0;
            
}







uint8_t popCommaToComma(uint8_t string_in[], uint8_t string_out[]){
	uint8_t i = 1;
	uint8_t dim = 0;
	
	
	if(string_in[0] != ',') return 0xFF; //errore

	while(string_in[i] != ','){
		
		if(i >= DIMDATA) return 0xFF; //errore
		string_out[i-1] = string_in[i];
		i++;
	}
	
	dim = i;
	
	while(i<DIMDATA){
		
		string_in[i-dim] = string_in[i];
		i++;
		
	}
	
	dim--;
	return dim;
	
}








uint8_t readMarker(uint8_t string[]){
	uint8_t i = 0;
	uint16_t counter = 0;
	
	while(string[0] != '$'){
	string[0]=EUSART2_Read();
	if (counter >= MAXTRYALS) return 0; //errore
	counter++;
	}
	
	for(i = 1; i < DIMMARKER; i++){
	string[i]=EUSART2_Read();
	}
	
	return 1;	//lettura con successo
}








uint8_t readData(uint8_t string[]){
	uint8_t i = 0;
	
	while(i < DIMDATA){
	
	string[i]=EUSART2_Read();
	if(string[i] == '*') return 1; 	//successo
	
	i++;
	}
	
	return 0;	//errore
	
	
}







float stringToFloat(uint8_t string[], uint8_t dim){
	uint8_t comma = 0;
	uint8_t i = 0;
	float ret = 0;
	
	while(string[i] != '.'){
	i++;
	}
	comma = i;
	//printf("comma: %d\n", comma);
	
	for(i = 0; i < comma; i++){
		if((string[i] > 57) || (string[i] < 48)) return 0; //errore
		//printf("%c\n", string[i]);
		//printf("%f\n", (float)(string[i]-48));
		ret = ret + (float)((float)(string[i]-48) * pow(10,(comma - i -1)));
		//printf("%f\n", ret);
	}
	
	for(i = comma + 1 ; i < dim; i++){
		if((string[i] > 57) || (string[i] < 48)) return 0; //errore
		//printf("%c\n", string[i]);
		ret = ret + (float)((float)(string[i]-48) * pow(10,(comma - i)));
		//printf("%f\n", ret);
	}
	
	return ret;
}









uint8_t stringToUint8_t(uint8_t string[], uint8_t dim){
	uint8_t i = 0;
	uint8_t ret = 0;

	for(i = 0; i < dim; i++){
		if((string[i] > 57) || (string[i] < 48)) return 0; //errore
		ret = ret + ((string[i]-48) * pow(10,(dim - i -1)));
	}
	
	
	return ret;

}

