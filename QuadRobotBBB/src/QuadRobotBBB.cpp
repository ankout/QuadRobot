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

unsigned short int combineValues(unsigned char upper, unsigned char lower)
{
	return (((unsigned short int)(upper << 8)) | ((unsigned short int)(lower)));
}



int main()
{
	unsigned char motorCommand[SPI_TRANSMISSION_SIZE], receive[SPI_TRANSMISSION_SIZE], counter = 0;
	unsigned char check1Tx, check2Tx;
	unsigned short int sum1Tx, sum2Tx;
	unsigned short int OL, IL, IIL;
	unsigned short int posInData;
	unsigned short int i, temp;

	struct LEG LEGdata[5];
	struct FSR_PCBA FSRdata[5];
	struct MAIN_PCBA MAINdata;

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
		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);
		usleep(100000);
		posInData = 5;

		for (OL = 0; OL < 5; OL++)
		{
			//FSR[OL].firmwareVersion = receive[(unsigned char)posInData];
			cout << "receive[" << (int)(posInData) << "] = " << (int)receive[posInData] << endl;
			FSRdata[0L].firmwareVersion = receive[posInData];
			//temp = (unsigned short int)receive[posInData];
			//FSR[0L].firmwareVersion = (unsigned char)temp;
			//cout << "temp = " << (int)(temp) << endl;

			//FSR[OL].firmwareVersion = 67;
			posInData++;

			for (IL = 0; IL < 2*NUM_FSRS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				FSRdata[OL].data[IL] = combineValues(receive[2*IL+posInData]&0b00000011, receive[2*IL+1+posInData]);
			}

			posInData = posInData+48;
			FSRdata[OL].error = receive[posInData];
			posInData++;
			FSRdata[OL].chksum1 = receive[posInData];
			posInData++;
			FSRdata[OL].chksum2 = receive[posInData];
			posInData++;

			LEGdata[OL].firmwareVersion_ = receive[posInData];
			posInData++;

			for (IL = 0; IL < 2*NUM_ENCODERS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				LEGdata[OL].encoder[IL] = combineValues(receive[2*IL+posInData]&0b00000011, receive[2*IL+1+posInData]);
			}


			posInData = posInData+10;
			cout << "1) FSR[0L].firmwareVersion = " << (int)(FSRdata[0L].firmwareVersion) << endl;

			for (IL = 0; IL < 2*NUM_ENCODERS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				//LEG[OL].motCurrent[IL] = combineValues((receive[2*IL+posInData] & 0b00000011), receive[2*IL+1+posInData]);
				LEGdata[OL].motCurrent[IL] = 88;
			}
			cout << "2) FSR[0L].firmwareVersion = " << (int)(FSRdata[0L].firmwareVersion) << endl;

			posInData = posInData+10;
			LEGdata[OL].error_ = receive[posInData];
			posInData++;
			LEGdata[OL].chksum1_ = receive[posInData];
			posInData++;
			LEGdata[OL].chksum2_ = receive[posInData];
			posInData++;

			cout << "5) FSR[0L].firmwareVersion = " << (int)(FSRdata[0L].firmwareVersion) << endl;
		}

		MAINdata.firmwareVersion = receive[posInData];
		posInData++;
		MAINdata.error = receive[posInData];
		posInData++;

		// Need to add checksum calculation to main loop eventually
		cout << "Preamble bytes are: " << (int)receive[1] << " " << (int)receive[2] << " " << (int)receive[3] << " " << (int)receive[4] << endl;

		for (OL = 0; OL < 5; OL++)
		{
			cout << "--------- [FSR " << (int)(OL+1) << "]------------" << endl;
			cout << "Firmware Version: " << (int)(FSRdata[OL].firmwareVersion) << endl;
			cout << "Error Flag: " << (int)FSRdata[OL].error << endl;
			cout << "Checksum 1: " << (int)FSRdata[OL].chksum1 << endl;
			cout << "Checksum 2: " << (int)FSRdata[OL].chksum2 << endl;

			for (IL = 0; IL < 2; IL++)
			{
				cout << "FSR Data "<< (int)(IL*12+1) << "-" << (int)((IL+1)*12) << ": ";

				for (IIL = 0; IIL < 12; IIL++)
				{
					cout << (int)FSRdata[OL].data[IL*12+IIL] << " ";
				}

				cout << endl;
			}

//			cout << "--------- [LEG " << (int)(OL+1) << "]------------" << endl;
//			cout << "Firmware Version: " << (int)LEG[OL].firmwareVersion << endl;
//			cout << "Error Flag: " << (int)LEG[OL].error << endl;
//			cout << "Checksum 1: " << (int)LEG[OL].chksum1 << endl;
//			cout << "Checksum 2: " << (int)LEG[OL].chksum2 << endl;
//
//			cout << "Encoder Data: ";
//
//			for (IL = 0; IL < NUM_ENCODERS; IL++)
//			{
//				cout << (int)LEG[OL].encoder[IL] << " ";
//			}
//
//			cout << endl;
//
//			cout << "Motor Current Data: ";
//
//			for (IL = 0; IL < NUM_ENCODERS; IL++)
//			{
//				cout << (int)LEG[OL].motCurrent[IL] << " ";
//			}
//
//			cout << endl;
		}

		cout << "---------- [MAIN]------------" << endl;
		cout << "Firmware Version: " << (int)MAINdata.firmwareVersion << endl;
		cout << "Error Flag: " << (int)MAINdata.error << endl;

		counter = 0;

		for (OL = 0; OL < SPI_TRANSMISSION_SIZE; OL++)
		{
			if (counter == 0)
			{
				cout << endl << (unsigned int)OL << ") ";
				counter = 10;
			}

			cout << (unsigned int)receive[OL] << "  ";
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
