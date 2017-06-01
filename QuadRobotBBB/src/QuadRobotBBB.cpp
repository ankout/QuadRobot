/* Using an SPI ADC (e.g., the MCP3008)
 * Written by Derek Molloy for the book "Exploring BeagleBone: Tools and
 * Techniques for Building with Embedded Linux" by John Wiley & Sons, 2014
 * ISBN 9781118935125. Please see the file README.md in the repository root
 * directory for copyright and GNU GPLv3 license information.            */


#include <iostream>
#include <sstream>
#include "bus/SPIDevice.h"
#include "typeDefs.h"
#include "miscFunctions.h"
#include <unistd.h>

using namespace std;
using namespace exploringBB;

short combineValues(unsigned char upper, unsigned char lower){
	return ((short)upper<<8)|(short)lower;
}

int main()
{
	unsigned char motorCommand[SPI_TRANSMISSION_SIZE], receive[SPI_TRANSMISSION_SIZE], counter = 0;
	unsigned char check1Tx, check2Tx;
	unsigned short int sum1Tx, sum2Tx;
	unsigned short int OL, IL, IIL;
	unsigned short int posInData;
	unsigned short int i;

	struct FSR_PCBA {
		unsigned char firmwareVersion;
		unsigned short data[24];
		unsigned char error;
		unsigned char chksum1;
		unsigned char chksum2;
	};

	struct LEG_PCBA {
			unsigned char firmwareVersion;
			unsigned short encoder[5];
			unsigned short motCurrent[5];
			unsigned char error;
			unsigned char chksum1;
			unsigned char chksum2;
	};

	struct MAIN_PCBA {
		unsigned char firmwareVersion;
		unsigned char error;
	};

	FSR_PCBA FSR[5];
	LEG_PCBA LEG[5];
	MAIN_PCBA MAIN;

	cout << "Starting_11..." << endl;
	SPIDevice *busDevice = new SPIDevice(1,0); //Using second SPI bus (both loaded)
	busDevice->setSpeed(400000);      // Have access to SPI Device object
	busDevice->setMode(SPIDevice::MODE0);

	for (i = 0; i < SPI_PREAMBLE_BYTES; i++)
	{
		motorCommand[i] = 0xFF;
	}

	sum1Tx = 0;
	sum2Tx = 0;

	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i++)
	{
		motorCommand[i] = i-SPI_PREAMBLE_BYTES;
		sum1Tx = sum1Helper(sum1Tx,motorCommand[i]);
		sum2Tx = sum2Helper(sum1Tx,sum1Tx);
	}

	check1Tx = check1Helper(sum1Tx, sum2Tx);
	check2Tx = check2Helper(sum1Tx, check1Tx);

	motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE] = check1Tx;
	motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+1] = check2Tx;

	// Fill the rest with 0s
	for (i = (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i < SPI_TRANSMISSION_SIZE; i++)
	{
		motorCommand[i] = 0xBB;
	}

	while (1)
	{
		posInData = 0;
		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);

		posInData = 5;
		for (OL = 0; OL < 5; OL++)
		{
			FSR[OL].firmwareVersion = receive[posInData];
			posInData++;

			for (IL = 0; IL < 2*NUM_FSRS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				FSR[OL].data[IL] = combineValues(receive[2*IL+posInData]&0b00000011, receive[2*IL+1+posInData]);
			}

			posInData = posInData+48;
			FSR[OL].error = receive[posInData];
			posInData++;
			FSR[OL].chksum1 = receive[posInData];
			posInData++;
			FSR[OL].chksum2 = receive[posInData];
			posInData++;

			LEG[OL].firmwareVersion = receive[posInData];
			posInData++;

			for (IL = 0; IL < 2*NUM_ENCODERS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				LEG[OL].encoder[IL] = combineValues(receive[2*IL+posInData]&0b00000011, receive[2*IL+1+posInData]);
			}

			posInData = posInData+10;

			for (IL = 0; IL < 2*NUM_ENCODERS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				LEG[OL].motCurrent[IL] = combineValues(receive[2*IL+posInData]&0b00000011, receive[2*IL+1+posInData]);
			}

			posInData = posInData+10;
			LEG[OL].error = receive[posInData];
			posInData++;
			LEG[OL].chksum1 = receive[posInData];
			posInData++;
			LEG[OL].chksum2 = receive[posInData];
			posInData++;
		}

		MAIN.firmwareVersion = receive[posInData];
		posInData++;
		MAIN.error = receive[posInData];
		posInData++;

		// Need to add checksum calculation to main loop eventually
		cout << "Preamble bytes are: " << (int)receive[1] << " " << (int)receive[2] << " " << (int)receive[3] << " " << (int)receive[4] << endl;

		for (OL = 0; OL < 5; OL++)
		{
			cout << "--------- [FSR " << (int)(OL+1) << "]------------" << endl;
			cout << "Firmware Version: " << (int)FSR[OL].firmwareVersion << endl;
			cout << "Error Flag: " << (int)FSR[OL].error << endl;
			cout << "Checksum 1: " << (int)FSR[OL].chksum1 << endl;
			cout << "Checksum 2: " << (int)FSR[OL].chksum2 << endl;

			for (IL = 0; IL < 2; IL++)
			{
				cout << "FSR Data "<< (int)(IL*12+1) << "-" << (int)((IL+1)*12) << ": ";

				for (IIL = 0; IIL < 12; IIL++)
				{
					cout << (int)FSR[OL].data[IL*12+IIL] << " ";
				}

				cout << endl;
			}

			cout << "--------- [LEG " << (int)(OL+1) << "]------------" << endl;
			cout << "Firmware Version: " << (int)LEG[OL].firmwareVersion << endl;
			cout << "Error Flag: " << (int)LEG[OL].error << endl;
			cout << "Checksum 1: " << (int)LEG[OL].chksum1 << endl;
			cout << "Checksum 2: " << (int)LEG[OL].chksum2 << endl;

			cout << "Encoder Data: ";

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				cout << (int)LEG[OL].encoder[IL] << " ";
			}

			cout << endl;

			cout << "Motor Current Data: ";

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				cout << (int)LEG[OL].motCurrent[IL] << " ";
			}

			cout << endl;
		}

		cout << "---------- [MAIN]------------" << endl;
		cout << "Firmware Version: " << (int)MAIN.firmwareVersion << endl;
		cout << "Error Flag: " << (int)MAIN.error << endl;

		counter = 0;

		for (OL = 0; OL < SPI_TRANSMISSION_SIZE; OL++)
		{
			if (counter == 0)
			{
				cout << endl << (unsigned int)OL << ") ";
				counter = 10;
			}

			cout << (unsigned int)receive[(unsigned int)OL] << "  ";
			counter--;
		}

		cout << endl;


		// NUM_LEG_PCBS*LEG_DATA_SIZE_RX <--- This does not include the error check


		//cout << (int)counter << ") Response bytes are " << (int)receive[1] << "," << (int)receive[2] << endl;

		// Use the 8-bits of the second value and the two LSBs of the first value
		//int value = combineValues(receive[1]&0b00000011, receive[2]);

		usleep(10000000);
		//counter++;
	}
}
