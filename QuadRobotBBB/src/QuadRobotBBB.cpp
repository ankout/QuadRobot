#include <iostream>
#include <sstream>
#include "bus/SPIDevice.h"
#include "typeDefs.h"
#include "miscFunctions.h"
#include <unistd.h>
#include "GPIO.h"

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
	busDevice->setSpeed(800000);      // Have access to SPI Device object
	busDevice->setMode(SPIDevice::MODE0);

	getMotorCommands(motorCommand);

	GPIO GPIO_1(61);
	GPIO_1.setDirection(OUTPUT);

	GPIO GPIO_2(22);
	GPIO_2.setDirection(OUTPUT);

	while (1)
	{
		//cout << endl << "        -----[" << (unsigned long int)counter << "]-----" << endl;
		GPIO_1.setValue(HIGH);
		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);
		GPIO_1.setValue(LOW);
		GPIO_2.setValue(HIGH);
		parseSPIfromMAIN(LEGdata, FSRdata, &MAINdata, &QUADdata, receive);
		GPIO_2.setValue(LOW);
		//printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata, 0b00001);

		//GPIO_1.toggleOutput();

		counter++;
		//GPIO_1.toggleOutput();
		//usleep(50);
	}
}


