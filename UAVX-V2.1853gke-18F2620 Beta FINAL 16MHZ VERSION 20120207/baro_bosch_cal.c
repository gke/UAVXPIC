#include "uavx.h"

//From Bosch Sensortec BMP085_SMD500_API
int16 ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16 ac4, ac5, ac6;

void BMP085Calibrate() 
{
	ac1 = 408;
	ac2 = -72;
	ac3 = -14383;
	ac4 = 32741;
	ac5 = 32757;
	ac6 = 23153;
	b1 = 6190;
	b2 = 4;
	mb = -32767;
	mc = -8711;
	md = 2868;
}

void BMP085(int32 up, int32 ut) 
{
	int32  tval, pval;
	int32  x1, x2, x3, b3, b5, b6, p;
	uint32  b4, b7;
	uint8 oss = 3;

	x1 = (ut - ac6) * ac5 >> 15;
	x2 = ((int32 ) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	tval = (b5 + 8) >> 4;
	
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((int32 ) ac1 * 4 + x3)<<oss + 2) >> 2;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32 ) (x3 + 32768)) >> 15;
	b7 = ((uint32 ) up - b3) * (50000 >> oss);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	pval = p + ((x1 + x2 + 3791) >> 4);
}


void SMD500Calibrate() 
{
	ac1 = 408;
	ac2 = -72;
	ac3 = -14383;
	ac4 = 32741;
	ac5 = 32757;
	ac6 = 23153;
	b1 = 6190;
	b2 = 4;
	mb = -32767;
	mc = -8711;
	md = 2868;
}
/*
void SMD500(int32 up, int32 ut) 
{
	int32  tval, pval;
	int32  x1, x2, x3, b3, b5, b6, p;
	uint32  b4, b7;
	uint8 oss = 3;

	x1 = (ut - ac6) * ac5 >> 15;
	x2 = ((int32 ) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	tval = (b5 + 8) >> 4;
	
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((int32 ) ac1 * 4 + x3)<<oss + 2) >> 2;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32 ) (x3 + 32768)) >> 15;
	b7 = ((uint32 ) up - b3) * (50000 >> oss);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	pval = p + ((x1 + x2 + 3791) >> 4);
}
*/