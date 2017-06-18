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


int main()
{
	unsigned char motorCommand[SPI_TRANSMISSION_SIZE], receive[SPI_TRANSMISSION_SIZE];
	unsigned char check1Tx, check2Tx;
	unsigned short int sum1Tx, sum2Tx;
	unsigned short int i;

	struct LEG_PCB LEGdata[NUM_LEG_PCBS];
	struct FSR_PCBA FSRdata[NUM_LEG_PCBS];
	struct MAIN_PCBA MAINdata;
	struct QUAD_ROBOT QUADdata;

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

	// Fill the rest with 0s
	for (i = (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i < SPI_TRANSMISSION_SIZE; i++)
	{
		motorCommand[i] = 0xBB;
	}

	while (1)
	{
		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);
		usleep(100000);

		parseSPIfromMAIN(LEGdata, FSRdata, &MAINdata, &QUADdata, receive);
		printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata);
		//printSPIstream(receive);

		usleep(10000000);
		//counter++;
	}
}


