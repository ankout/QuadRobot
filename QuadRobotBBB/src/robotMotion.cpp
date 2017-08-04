/*
 * robotMotion.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: acardi
 */

#include "robotMotion.h"
#include "typeDefs.h"
#include "miscFunctions.h"

void getMotorCommands(unsigned char *p_motorCommand)
{
	unsigned char check1Tx, check2Tx;
	unsigned short int i;
	unsigned short int sum1Tx, sum2Tx;

	unsigned char high = 20, low = 40;

	// The first four bytes are preamble bytes
	for (i = 0; i < SPI_PREAMBLE_BYTES; i++)
	{
		p_motorCommand[i] = 0xFF;
	}

	sum1Tx = 0;
	sum2Tx = 0;

	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i++)
	{
		p_motorCommand[i] = i-SPI_PREAMBLE_BYTES;

		if ((i == 4) | (i == 10) | (i == 16) | (i == 21) | (i == 26))
		{
			p_motorCommand[i] = high;
		}

		else if ((i == 5) | (i == 11) | (i == 17) | (i == 22) | (i == 27))
		{
			p_motorCommand[i] = high;
		}

		else if ((i == 6) | (i == 12) | (i == 18) | (i == 23) | (i == 28))
		{
			p_motorCommand[i] = high;
		}

		else if ((i == 7) | (i == 13) | (i == 19) | (i == 24) | (i == 29))
		{
			p_motorCommand[i] = low;
		}

		else if ((i == 8) | (i == 14) | (i == 20) | (i == 25) | (i == 30))
		{
			p_motorCommand[i] = low;
		}

		else if ((i == 9) | (i == 15) | (i == 21) | (i == 26) | (i == 31))
		{
			p_motorCommand[i] = 0b10110;
		}


		sum1Tx = sum1Helper(sum1Tx, p_motorCommand[i]);
		sum2Tx = sum2Helper(sum1Tx, sum1Tx);
	}

	check1Tx = check1Helper(sum1Tx, sum2Tx);
	check2Tx = check2Helper(sum1Tx, check1Tx);

	p_motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE] = check1Tx;
	p_motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+1] = check2Tx;

	// Fill the rest with 0s...Tx and Rx data have to be the same length for SPI
	for (i = (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i < SPI_TRANSMISSION_SIZE; i++)
	{
		p_motorCommand[i] = 0x0;
	}
}


