/*
 * robotMotion.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: acardi
 */

#include "robotMotion.h"
#include "typeDefs.h"
#include "miscFunctions.h"
#include "jointConstants.h"

//#include <iostream>
#include <stdio.h>

// This function calculates the requires motor commands given desired joint angles
/*void getMotorCommands(unsigned char *p_motorCommand, unsigned char *p_desiredAngle)
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
			p_motorCommand[i] = 0b10100;
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
}*/

void getJointAngles(unsigned char *forwardV, unsigned char *rotationV, float *p_desiredAngle)
{
	// Angle Convention for p_desiredAngle[20]:
	// Leg 1: [0]-[4]
	// Leg 2: [5]-[9]
	// Leg 3: [10]-[14]
	// Leg 4: [15]-[19]
	p_desiredAngle[3] = 160.1f;
	p_desiredAngle[4] = 90.72f;
}

void getMotorCommands(unsigned char *p_motorCommand, float *p_desiredAngle, struct LEG_PCB *p_LEGdata)
{
	/*float currentJointAngle[NUM_ENCODERS*NUM_LEGS];

	unsigned char OL, IL;

	for (OL = 0; OL < NUM_LEGS; OL++)
	{
		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			currentJointAngle[OL*NUM_ENCODERS+IL] =
					(float)(p_LEGdata[OL].encoder[IL] - minEncRead[OL*NUM_ENCODERS+IL])*DEG_PER_BIT*encDir[OL*NUM_ENCODERS+IL] + angleAtMinEncoderReading[OL*NUM_ENCODERS+IL];

			//cout << (int)(OL*NUM_ENCODERS+IL) << ")" << (float)currentJointAngle[OL*NUM_ENCODERS+IL] << endl;
			printf("Angle %d) %f\n",OL*NUM_ENCODERS+IL,currentJointAngle[OL*NUM_ENCODERS+IL]);
		}
	}*/
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
				p_motorCommand[i] = 0b10100;
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


