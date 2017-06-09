#include "miscFunctions.h"

unsigned short int sum1Helper(unsigned short int sum1, unsigned char data)
{
	return ((sum1+data) % 255);
}

unsigned short int sum2Helper(unsigned short int sum1, unsigned short int sum2)
{
	return ((sum1+sum2) % 255);
}

unsigned char check1Helper(unsigned short int sum1Tx, unsigned short int sum2Tx)
{
	return (255 - ((sum1Tx + sum2Tx) % 255));
}

unsigned char check2Helper(unsigned short int sum1Tx, unsigned char check1Tx)
{
	return (255 - ((sum1Tx + check1Tx) % 255));
}

unsigned short int combineValues(unsigned char upper, unsigned char lower)
{
	return (((unsigned short int)(upper << 8)) | ((unsigned short int)(lower)));
}
