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
#include "typeDefs.h"

#include<stdio.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<stdint.h>
#include<linux/spi/spidev.h>

#define SPI_PATH "/dev/spidev1.0"

using namespace exploringBB;
using namespace std;

int transfer(int fd, unsigned char send[], unsigned char receive[], int length){
	struct spi_ioc_transfer transfer;           //the transfer structure
	transfer.tx_buf = (unsigned long) send;     //the buffer for sending data
	transfer.rx_buf = (unsigned long) receive;  //the buffer for receiving data
	transfer.len = length;                      //the length of buffer
	transfer.speed_hz = 1000000;                //the speed in Hz
	transfer.bits_per_word = 8;                 //bits per word
	transfer.delay_usecs = 0;                   //delay in us

	// send the SPI message (all of the above fields, inc. buffers)
	int status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);

	if (status < 0)
	{
		perror("SPI: SPI_IOC_MESSAGE Failed");
		return -1;
	}
	return status;
}


int main(){
	int fd, i=0;                   //file handle and loop counter
	const unsigned char preambleByte = 0xFF;
	unsigned char null=0x00;         //sending only a single char
	uint8_t bits = 8, mode = 3;             //8-bits per word, SPI mode 3
	uint32_t speed = 1000000;               //Speed is 1 MHz

	if(getuid() != 0)
	{
		cout << "You must run this program as root. Exiting..." << endl;
		return -1;
	}

	cout << "Starting44..." << endl;

	// The number used is the "GPIO" number on Figure 6-7
	GPIO outGPIO(49);

	outGPIO.setDirection(OUTPUT);


	outGPIO.setValue(HIGH);
	usleep(50);
	outGPIO.setValue(LOW);
	usleep(50);




	// The following calls set up the SPI bus properties
	if ((fd = open(SPI_PATH, O_RDWR))<0)
	{
		perror("SPI Error: Can't open device.");
		return -1;
	}

	if (ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1)
	{
		perror("SPI: Can't set SPI mode.");
		return -1;
	}

	if (ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1)
	{
		perror("SPI: Can't get SPI mode.");
		return -1;
	}

	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1)
	{
		perror("SPI: Can't set bits per word.");
		return -1;
	}

	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1)
	{
		perror("SPI: Can't get bits per word.");
		return -1;
	}

	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1)
	{
		perror("SPI: Can't set max speed HZ");
		return -1;
	}

	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1)
	{
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	// Check that the properties have been set
	//printf("SPI Mode is: %d \n", mode);
	//printf("SPI Bits is: %d\n", bits);
	//printf("SPI Speed is: %d\n", speed);
	//printf("Counting through all of the LEDs:\n");

	// This delay needs to be here or things to (I think) give the SPI stuff time to set up

	usleep(100000);

	// Send preamble bytes to PIC 32
	for (i=0; i<SPI_PREAMBLE_BYTES; i++)
	{
		// This function can send and receive data, just sending here
		if (transfer(fd, (unsigned char*) &preambleByte, &null, 1)==-1){
			perror("Failed to update the display");
			return -1;
		}
	}

	close(fd);               //close the file
	return 0;
}
