#include <iostream>
#include <sstream>
#include "bus/SPIDevice.h"
#include "typeDefs.h"
#include "miscFunctions.h"
#include "robotMotion.h"
#include <unistd.h>
#include "GPIO.h"

//GPIO_1.setValue(HIGH);

using namespace std;
using namespace exploringBB;

int main()
{
	unsigned char motorCommand[SPI_TRANSMISSION_SIZE], receive[SPI_TRANSMISSION_SIZE];
	float desiredAngle[NUM_ENCODERS*4]; // Unsigned integer representing desired angle of each joint (0-4095)
	unsigned char forwardV, rotationV;

	unsigned long int counter = 0;

	struct LEG_PCB LEGdata[NUM_LEG_PCBS];
	struct FSR_PCBA FSRdata[NUM_LEG_PCBS];
	struct MAIN_PCBA MAINdata;
	struct QUAD_ROBOT QUADdata;

	SPIDevice *busDevice = new SPIDevice(1,0); //Using second SPI bus (both loaded)
	busDevice->setSpeed(400000);      // If checksums on MAIN or QUAD are bad, try lowering this number
	busDevice->setMode(SPIDevice::MODE0);

	GPIO GPIO_1(61);
	GPIO_1.setDirection(OUTPUT);

	GPIO GPIO_2(22);
	GPIO_2.setDirection(OUTPUT);

	while (1)
	{

		GPIO_1.toggleOutput();
		//getRobotCommand(forwardV,rotationV);
		getJointAngles(&forwardV, &rotationV, desiredAngle); // Input
		getMotorCommands(motorCommand, desiredAngle);

		busDevice->transfer(motorCommand, receive, SPI_TRANSMISSION_SIZE);
		parseSPIfromMAIN(LEGdata, FSRdata, &MAINdata, &QUADdata, receive);
		cout << endl << "        ------[[" << (unsigned long int)counter << "]]------" << endl;
		//printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata, 0b00001);
		//printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata, 0b00011);
		printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata, 0b10001);
		usleep(1000001);


		//if (MAINdata.dataError > 0)
		//{
		//	printSensorData(LEGdata, FSRdata, &MAINdata, &QUADdata, 0b00001);
		//	usleep(10000000);
		//}

		counter++;
		//GPIO_1.toggleOutput();

	}
}


