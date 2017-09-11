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

	unsigned char OL, IL, curJoint;
	//unsigned char IIL, switchWith;

	unsigned char check1Tx, check2Tx;
	unsigned short int i;
	unsigned short int sum1Tx, sum2Tx;

	//unsigned char high = 20, low = 40;

	sum1Tx = 0;
	sum2Tx = 0;

	// The first four bytes are preamble bytes
	for (i = 0; i < SPI_PREAMBLE_BYTES; i++)
	{
		p_motorCommand[i] = 0xFF;
	}

	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i++)
	{
		p_motorCommand[i] = 0;
	}

	for (OL = 0; OL < NUM_LEGS; OL++)
	{
		// These bits are assigned by ORing, so they need to be zeroed out beforehand
		p_motorCommand[(OL+1)*NUM_ENCODERS+SPI_PREAMBLE_BYTES] = 0;

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			curJoint = OL*NUM_ENCODERS+IL;

			currentJointAngle[curJoint] =
					(float)(p_LEGdata[OL].encoder[IL] - minEncRead[curJoint])*DEG_PER_BIT*encDir[curJoint] + angleAtMinEncoderReading[curJoint];
			errorHist[curJoint] = p_desiredAngle[curJoint] - currentJointAngle[curJoint];

			// Formula for control law from http://portal.ku.edu.tr/~cbasdogan/Courses/Robotics/projects/Discrete_PID.pdf
//			u[curJoint] = u[curJoint] +
//					(P_GAIN[curJoint] + I_GAIN[curJoint]*TS/2.0f + D_GAIN[curJoint]/TS)*errorHist[curJoint] +
//					(-P_GAIN[curJoint] + I_GAIN[curJoint]*TS/2.0f - 2*D_GAIN[curJoint]/TS)*errorHist[curJoint+NUM_ENCODERS*NUM_LEGS] +
//					(D_GAIN[curJoint]/TS)*errorHist[curJoint+NUM_ENCODERS*NUM_LEGS*2];
			u[curJoint] = (P_GAIN[curJoint])*errorHist[curJoint];

			if (u[curJoint] > U_MAX)
			{
				u[curJoint] = U_MAX;
			}

			else if (u[curJoint] < -U_MAX)
			{
				u[curJoint] = -U_MAX;
			}

			//uHist[curJoint] = u[curJoint];

			errorHist[curJoint+NUM_ENCODERS*NUM_LEGS*2] = errorHist[curJoint+NUM_ENCODERS*NUM_LEGS]; // error two samples ago (for next time gains are calculated)
			errorHist[curJoint+NUM_ENCODERS*NUM_LEGS] = errorHist[curJoint]; // error one sample ago (for next time gains are calculated)
		}
	}

	//printf("Angle %d) Current: %f\tDesired: %f\tu=%f\n",4,currentJointAngle[3],p_desiredAngle[3],u[3]);


	for (OL = 0; OL < (NUM_ENCODERS*NUM_LEG_PCBS); OL++)
	{
		if (u[jointMapping[OL]] >= 0.0f)
		{
			// The math inside the indexer takes care of the preamble offset and the fact that every fifth entry is motor direction information
			p_motorCommand[OL+(OL/NUM_ENCODERS)+SPI_PREAMBLE_BYTES] = (unsigned char)(u[OL] + 0.5f);

			// Set direction bit
			// Direction bit is "1" if positive
			p_motorCommand[NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES] =
					p_motorCommand[NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES] | (0b1 << (OL % NUM_ENCODERS));

			//printf("(+)%d-%d-%d-%d\n",
			//		OL+(OL/NUM_ENCODERS)+SPI_PREAMBLE_BYTES,
			//		NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES,
			//		(0b1 << (OL % NUM_ENCODERS)),
			//		p_motorCommand[NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES]);

		}

		else
		{
			p_motorCommand[OL+(OL/NUM_ENCODERS)+SPI_PREAMBLE_BYTES] = (unsigned char)(-u[OL] + 0.5f);
			//printf("(-)%d-%d-%d-%d\n",
			//		OL+(OL/NUM_ENCODERS)+SPI_PREAMBLE_BYTES,
			//		NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES,
			//		0,
			//		p_motorCommand[NUM_ENCODERS + (1+NUM_ENCODERS)*(OL/NUM_ENCODERS) + SPI_PREAMBLE_BYTES]);
		}


	}

	printf("Angle %d) Desired: %f Current: %f u=%d  Direction: %d\n",1,p_desiredAngle[0],currentJointAngle[0],p_motorCommand[4],p_motorCommand[9]);
	//printf("\tAngle %d) Desired: %f Current: %f u=%f  Direction: %d\n",5,p_desiredAngle[4],currentJointAngle[4],u[4],p_motorCommand[9]);

	//for (OL = SPI_PREAMBLE_BYTES; OL < (SPI_PREAMBLE_BYTES+6); OL++)
	//{
	//	printf("p_motorCommand[%d]: %d \n", OL, p_motorCommand[OL]);
	//}


	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i++)
	{
		sum1Tx = sum1Helper(sum1Tx, p_motorCommand[i]);
		sum2Tx = sum2Helper(sum1Tx, sum1Tx);
	}

	check1Tx = check1Helper(sum1Tx, sum2Tx);
	check2Tx = check2Helper(sum1Tx, check1Tx);


//	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i++)
//	{
//		p_motorCommand[i] = i-SPI_PREAMBLE_BYTES;
//
//		if ((i == 4) | (i == 10) | (i == 16) | (i == 22) | (i == 28))
//		{
//			p_motorCommand[i] = high;
//		}
//
//		else if ((i == 5) | (i == 11) | (i == 17) | (i == 23) | (i == 29))
//		{
//			p_motorCommand[i] = high;
//		}
//
//		else if ((i == 6) | (i == 12) | (i == 18) | (i == 24) | (i == 30))
//		{
//			p_motorCommand[i] = high;
//		}
//
//		else if ((i == 7) | (i == 13) | (i == 19) | (i == 25) | (i == 31))
//		{
//			p_motorCommand[i] = low;
//		}
//
//		else if ((i == 8) | (i == 14) | (i == 20) | (i == 26) | (i == 32))
//		{
//			p_motorCommand[i] = low;
//		}
//
//		else if ((i == 9) | (i == 15) | (i == 21) | (i == 27) | (i == 33))
//		{
//			p_motorCommand[i] = 0b10100;
//		}
//
//
//		sum1Tx = sum1Helper(sum1Tx, p_motorCommand[i]);
//		sum2Tx = sum2Helper(sum1Tx, sum1Tx);
//	}

//	check1Tx = check1Helper(sum1Tx, sum2Tx);
//	check2Tx = check2Helper(sum1Tx, check1Tx);

	p_motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE] = check1Tx;
	p_motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+1] = check2Tx;

	// Fill the rest with 0s...Tx and Rx data have to be the same length for SPI
	for (i = (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i < SPI_TRANSMISSION_SIZE; i++)
	{
		p_motorCommand[i] = 0x0;
	}

}


