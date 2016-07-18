/*
TITLE: driver GPS-610F for PIC18F26K22 
AUTHOR: francescotesolin SKYWARD - TEAM GS
DESCRIPTION: 

*/

#include <stdio.h>
#include <stdlib.h>

#define DIMDATA 100
#define DIMMARKER 6
#define DIMTIME 10
#define MAXTRYALS 400

typedef struct {
		uint8_t time[DIMTIME];
		float latitude;
		uint8_t ns;
		float longitude;
		uint8_t ew;
		uint8_t quality;
		uint8_t n_satellites;
		float horizontal_dilution;
		float altitude;
		uint8_t units_altitude;
		float  geoidal_separation;
		uint8_t units_geoidal_separation;
	} gpsgga;
	
	
	

void printString(uint8_t string[], uint8_t dim);
uint8_t controlMarker(uint8_t string[]);
uint8_t readMarker(uint8_t string[]);
uint8_t readData(uint8_t string[]); 
uint8_t popCommaToComma(uint8_t string_in[], uint8_t string_out[]);
float stringToFloat(uint8_t string[], uint8_t dim);
uint8_t stringToUint8_t(uint8_t string[], uint8_t dim);

uint8_t readGPS610F(gpsgga *gps);