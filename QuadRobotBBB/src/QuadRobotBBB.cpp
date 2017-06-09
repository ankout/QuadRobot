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

void parseSPIfromMAIN(struct LEG_PCB *L, struct FSR_PCBA *F, struct MAIN_PCBA *M, unsigned char *r);

int main()
{
	unsigned char motorCommand[SPI_TRANSMISSION_SIZE], receive[SPI_TRANSMISSION_SIZE], counter = 0;
	unsigned char check1Tx, check2Tx;
	unsigned short int sum1Tx, sum2Tx;
	unsigned char OL, IL, IIL, counter2;
	unsigned short int posInData;
	unsigned short int i;

	struct LEG_PCB LEGdata[5];
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
		counter2 = 0;

		for (OL = 0; OL < 5; OL++)
		{
			cout << endl << "OL = " << (int)OL<< endl;

			//FSR[OL].firmwareVersion = receive[(unsigned char)posInData];
			cout << "receive[" << (int)(posInData) << "] = " << (int)receive[posInData] << endl;

			for (IL = 0; IL < 5; IL++)
			{
				cout << "FSR[" << (int)IL << "].firmwareVersion = " << (int)FSRdata[IL].firmwareVersion << endl;
			}

			cout << endl;

			//FSRdata[0L].firmwareVersion = receive[posInData];
			FSRdata[counter2].firmwareVersion = receive[posInData];
			counter2++;

			for (IL = 0; IL < 5; IL++)
			{
				cout << "FSR[" << (int)IL << "].firmwareVersion = " << (int)FSRdata[IL].firmwareVersion << endl;
			}

			posInData++;
			//cout << "OL = " << (int)OL<< " - 3) FSR[0].firmwareVersion = " << (int)FSRdata[0].firmwareVersion << endl;
			for (IL = 0; IL < NUM_FSRS; IL++)
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

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				LEGdata[OL].encoder[IL] = combineValues(receive[2*IL+posInData]&0b00000011, receive[2*IL+1+posInData]);
			}


			posInData = posInData+10;


			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				// These are 10 bit values sent with the upper byte first
				LEGdata[OL].motCurrent[IL] = combineValues((receive[2*IL+posInData] & 0b00000011), receive[2*IL+1+posInData]);
			}

			posInData = posInData+10;
			LEGdata[OL].error_ = receive[posInData];
			posInData++;
			LEGdata[OL].chksum1_ = receive[posInData];
			posInData++;
			LEGdata[OL].chksum2_ = receive[posInData];
			posInData++;
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
			cout << "Firmware Version: " << (int)FSRdata[OL].firmwareVersion << endl;
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

			cout << "--------- [LEG " << (int)(OL+1) << "]------------" << endl;
			cout << "Firmware Version: " << (int)LEGdata[OL].firmwareVersion_ << endl;
			cout << "Error Flag: " << (int)LEGdata[OL].error_ << endl;
			cout << "Checksum 1: " << (int)LEGdata[OL].chksum1_ << endl;
			cout << "Checksum 2: " << (int)LEGdata[OL].chksum2_ << endl;

			cout << "Encoder Data: ";

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				cout << (int)LEGdata[OL].encoder[IL] << " ";
			}

			cout << endl;

			cout << "Motor Current Data: ";

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				cout << (int)LEGdata[OL].motCurrent[IL] << " ";
			}

			cout << endl;
		}

		cout << "---------- [MAIN]------------" << endl;
		cout << "Firmware Version: " << (int)MAINdata.firmwareVersion << endl;
		cout << "Error Flag: " << (int)MAINdata.error << endl;

		counter = 0;

		for (i = 0; i < SPI_TRANSMISSION_SIZE; i++)
		{
			if (counter == 0)
			{
				cout << endl << (unsigned int)i << ") ";
				counter = 10;
			}

			cout << (unsigned int)receive[i] << "  ";
			counter--;
		}

		cout << endl;


		usleep(10000000);
		//counter++;
	}
}

void parseSPIfromMAIN(struct LEG_PCB *L, struct FSR_PCBA *F, struct MAIN_PCBA *M, unsigned char *r)
{

}
