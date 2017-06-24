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
	unsigned long int counter = 0;

	struct LEG_PCB LEGdata[NUM_LEG_PCBS];
	struct FSR_PCBA FSRdata[NUM_LEG_PCBS];
	struct MAIN_PCBA MAINdata;
	struct QUAD_ROBOT QUADdata;


	SPIDevice *busDevice = new SPIDevice(1,0); //Using second SPI bus (both loaded)
	busDevice->setSpeed(400000);      // Have access to SPI Device object
	busDevice->setMode(SPIDevice::MODE0);

	getMotorCommands(motorCommand);

	//GPIO outGPIO(49);
	//outGPIO.setDirection(OUTPUT);

	while (1)
	{
		cout << endl << "        -----[" << (unsigned long int)counter << "]-----" << endl;
		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);
		usleep(100000);

		parseSPIfromMAIN(LEGdata, FSRdata, &MAINdata, &QUADdata, receive);
		printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata, 0b00001);
		//printSPIstream(receive);

		usleep(3000000);
		counter++;
	}
}


