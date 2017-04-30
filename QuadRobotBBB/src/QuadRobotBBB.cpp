//============================================================================
// Name        : QuadRobotBBB.cpp
// Author      : Adam Cardi
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h> // used for microsecond sleeping
#include "GPIO.h"

#include<stdio.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<stdint.h>
#include<linux/spi/spidev.h>

#define SPI_PATH "/dev/spidev1.0"

using namespace exploringBB;
using namespace std;


int main()
{
	if(getuid() != 0)
	{
		cout << "You must run this program as root. Exiting..." << endl;
		return -1;
	}

	cout << "Starting0..." << endl;

	// The number used is the "GPIO" number on Figure 6-7
	GPIO outGPIO(49);

	outGPIO.setDirection(OUTPUT);

	while(1)
	{
		outGPIO.setValue(HIGH);
		usleep(50);
		outGPIO.setValue(LOW);
		usleep(50);
	}

	return 0;
}
