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

	unsigned char i;

	cout << "Starting_111..." << endl;
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

	//for (i = (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i++)
	//{
	//	motorCommand[i] = 0xAA;
	//}

	//send[0] = 0b00000001; // The Start Bit followed
	// Set the SGL/Diff and D mode -- e.g., 1000 means single ended CH0 value
	//send[1] = 0b10000000; // The MSB is the Single/Diff bit and it is followed by 000 for CH0
	//send[2] = 0;          // This byte doesn't need to be set, just for a clear display
	while (1)
	{
		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);
		cout << (int)counter << ") Response bytes are " << (int)receive[1] << "," << (int)receive[2] << endl;

		// Use the 8-bits of the second value and the two LSBs of the first value
		int value = combineValues(receive[1]&0b00000011, receive[2]);
		cout << "This is the value " << value << " out of 1024." << endl;
		usleep(1000000);
		counter++;
	}
}
