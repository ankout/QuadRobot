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
	p_desiredAngle[3] = 25.0f;
	p_desiredAngle[4] = 13.44f;
}

void getMotorCommands(unsigned char *p_motorCommand, float *p_desiredAngle, struct LEG_PCB *p_LEGdata)
{
	float currentJointAngle[NUM_ENCODERS*NUM_LEGS];
	static float errorHist[NUM_ENCODERS*NUM_LEGS*3]; // This will contain the last three errors for each joint
	static float uHist[NUM_ENCODERS*NUM_LEGS]; // contains previous control input
	static float u[NUM_ENCODERS*NUM_LEGS]; // contains current control input

	unsigned char OL, IL, curJoint, IIL, switchWith;

	unsigned char check1Tx, check2Tx;
	unsigned short int i;
	unsigned short int sum1Tx, sum2Tx;

	unsigned char high = 20, low = 40;

	sum1Tx = 0;
	sum2Tx = 0;

	// The first four bytes are preamble bytes
	for (i = 0; i < SPI_PREAMBLE_BYTES; i++)
	{
		p_motorCommand[i] = 0xFF;
	}

	for (OL = 0; OL < NUM_LEGS; OL++)
	{
		// These bits are assigned by ORing, so they need to be zeriod out beforehand
		p_motorCommand[(OL+1)*NUM_ENCODERS+SPI_PREAMBLE_BYTES] = 0;

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			curJoint = OL*NUM_ENCODERS+IL;

			currentJointAngle[curJoint] =
					(float)(p_LEGdata[OL].encoder[IL] - minEncRead[curJoint])*DEG_PER_BIT*encDir[curJoint] + angleAtMinEncoderReading[curJoint];
			errorHist[curJoint] = p_desiredAngle[curJoint] - currentJointAngle[curJoint];

			// Formula for control law from http://portal.ku.edu.tr/~cbasdogan/Courses/Robotics/projects/Discrete_PID.pdf
			u[curJoint] = uHist[curJoint] +
					(P_GAIN[curJoint] + I_GAIN[curJoint]*TS/2.0f + D_GAIN[curJoint]/TS)*errorHist[curJoint] +
					(-P_GAIN[curJoint] + I_GAIN[curJoint]*TS/2.0f - 2*D_GAIN[curJoint]/TS)*errorHist[curJoint+NUM_ENCODERS*NUM_LEGS] +
					(D_GAIN[curJoint]/TS)*errorHist[curJoint+NUM_ENCODERS*NUM_LEGS*2];

			if (u[curJoint] > U_MAX)
			{
				u[curJoint] = U_MAX;
			}

			else if (u[curJoint] < -U_MAX)
			{
				u[curJoint] = -U_MAX;
			}

			uHist[curJoint] = u[curJoint];

			errorHist[curJoint+NUM_ENCODERS*NUM_LEGS*2] = errorHist[curJoint+NUM_ENCODERS*NUM_LEGS]; // error two samples ago (for next time gains are calculated)
			errorHist[curJoint+NUM_ENCODERS*NUM_LEGS] = errorHist[curJoint]; // error one sample ago (for next time gains are calculated)
		}
	}

	for (OL = 0; OL < (NUM_ENCODERS*NUM_LEG_PCBS); OL++)
	{
		if (u[jointMapping[OL]] >= 0.0f)
		{
			// The math inside the indexer takes care of the preamble offset and the fact that every fifth entry is motor direction information
			p_motorCommand[OL+(OL/NUM_ENCODERS)+SPI_PREAMBLE_BYTES] = (unsigned char)(uHist[curJoint] + 0.5f);

			// Direction bit is "1" if positive
			p_motorCommand[NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES] = //left of here



					//p_motorCommand[(OL+1)*NUM_ENCODERS+SPI_PREAMBLE_BYTES] | (0b1 << IL);

		}

		else
		{
			p_motorCommand[OL+(OL/NUM_ENCODERS)+SPI_PREAMBLE_BYTES] = (unsigned char)(-uHist[curJoint] + 0.5f);
		}
	}

	// This byte is built up by ORing, so it needs to be zeroed out
	p_motorCommand[31] = 0;

	for (i = 26; i < 31; i++)
	{
		switch(i)
		{
			// "switchWith" is the motor number that needs to be swapped (0-based)
			// "i" is the byte index (0-based)
			// motor number 0 starts after SPI_PREAMBLE_BYTES

			case 26:
				switchWith = 12;
			break;
			case 27:
				switchWith = 12;
				break;
			case 28:
				switchWith = 12;
				break;
			case 29:
				switchWith = 12;
				break;
			case 30:
				switchWith = 12;
				break;

		}

		p_motorCommand[i] = p_motorCommand[switchWith];
		p_motorCommand[31] = p_motorCommand[31] | ((0b1 << (switchWith % NUM_ENCODERS)) & p_motorCommand

		// Bytes 26-31 (0-based counting) pertain to PCB 5

		p_motorCommand[27] = 0;
		p_motorCommand[28] = 0;
		p_motorCommand[29] = 0;
		p_motorCommand[30] = 0;
		p_motorCommand[31] = 0b00000;

	////////////////////////////////

	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i++)
	{
		sum1Tx = sum1Helper(sum1Tx, p_motorCommand[curJoint+SPI_PREAMBLE_BYTES]);
		sum2Tx = sum2Helper(sum1Tx, sum1Tx);
	}

	check1Tx = check1Helper(sum1Tx, sum2Tx);
	check2Tx = check2Helper(sum1Tx, check1Tx);

	//printf("Angle %d) %f\n",4,currentJointAngle[3]);
	//printf("Angle %d) %f\n",5,currentJointAngle[4]);

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


