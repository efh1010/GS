void readCalibrationRegister(void);
int32_t readUncompensatedTemperature(void);
int32_t readUncompensatedPressure(void);
int32_t calculateTrueTemperature(void);
int32_t calculateTruePressure(void);


uint8_t readBMP180(int32_t data_read[]);


/*
data_read[0]=temperature;
data_read[1]=pressure;
*/


void serialPrint_BMP180(void);

/*
print:

MARKER 					0xAA
internal register(22)	0xAA-0xBF


MARKER 					0xAA
UT(2)					MSB-LSB
UP(3)					MSB-LSB-XLSB

MARKER 					0xAA
T(1) 					t/10
P(1) 					p/1000

*/


void print_8bit(int32_t data);