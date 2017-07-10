#include <iostream>
#include <sstream>
#include "miscFunctions.h"
#include "typeDefs.h"
#include <time.h>
#include <stdio.h>

using namespace std;


void getMotorCommands(unsigned char *p_motorCommand)
{
	unsigned char check1Tx, check2Tx;
	unsigned short int i;
	unsigned short int sum1Tx, sum2Tx;

	unsigned char high = 0, low = 0;

	// The first four bytes are preamble bytes
	for (i = 0; i < SPI_PREAMBLE_BYTES; i++)
	{
		p_motorCommand[i] = 0xFF;
	}

	sum1Tx = 0;
	sum2Tx = 0;

	for (i = SPI_PREAMBLE_BYTES; i < (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE); i++)
	{
		p_motorCommand[i] = i-SPI_PREAMBLE_BYTES;

		if (i == 4)
		{
			p_motorCommand[i] = high;
		}

		else if (i == 5)
		{
			p_motorCommand[i] = low;
		}

		else if (i == 6)
		{
			p_motorCommand[i] = high;
		}

		else if (i == 7)
		{
			p_motorCommand[i] = low;
		}

		else if (i == 8)
		{
			p_motorCommand[i] = high;
		}

		else if (i == 9)
		{
			p_motorCommand[i] = 0b10111;
		}

		///////

		else if (i == 10)
		{
			p_motorCommand[i] = low;
		}

		else if (i == 11)
		{
			p_motorCommand[i] = high;
		}

		else if (i == 12)
		{
			p_motorCommand[i] = low;
		}

		else if (i == 13)
		{
			p_motorCommand[i] = high;
		}

		else if (i == 14)
		{
			p_motorCommand[i] = low;
		}

		else if (i == 15)
		{
			p_motorCommand[i] = 0b10101;
		}

		sum1Tx = sum1Helper(sum1Tx, p_motorCommand[i]);
		sum2Tx = sum2Helper(sum1Tx, sum1Tx);
	}

	check1Tx = check1Helper(sum1Tx, sum2Tx);
	check2Tx = check2Helper(sum1Tx, check1Tx);

	p_motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE] = check1Tx;
	p_motorCommand[SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+1] = check2Tx;

	// Fill the rest with 0s...Tx and Rx data have to be the same length for SPI
	for (i = (SPI_PREAMBLE_BYTES+SPI_TX_DATA_SIZE+SPI_CHECKSUM_SIZE); i < SPI_TRANSMISSION_SIZE; i++)
	{
		p_motorCommand[i] = 0x0;
	}
}

unsigned short int sum1Helper(unsigned short int sum1, unsigned char data)
{
	return ((sum1+data) % 255);
}

unsigned short int sum2Helper(unsigned short int sum1, unsigned short int sum2)
{
	return ((sum1+sum2) % 255);
}

void sum1sum2(unsigned short int *p_sum1, unsigned short int *p_sum2, unsigned char *p_data)
{
	*p_sum1 = ((*p_sum1+*p_data) % 255);
	*p_sum2 = ((*p_sum1+*p_sum2) % 255);
}

unsigned char check1Helper(unsigned short int sum1Tx, unsigned short int sum2Tx)
{
	return (255 - ((sum1Tx + sum2Tx) % 255));
}

unsigned char check2Helper(unsigned short int sum1Tx, unsigned char check1Tx)
{
	return (255 - ((sum1Tx + check1Tx) % 255));
}

unsigned short int combineValues(unsigned char upper, unsigned char lower)
{
	return (((unsigned short int)(upper << 8)) | ((unsigned short int)(lower)));
}

void parseSPIfromMAIN(struct LEG_PCB *p_LEGdata, struct FSR_PCBA *p_FSRdata, struct MAIN_PCBA *p_MAINdata, struct QUAD_ROBOT *p_QUADdata, unsigned char *p_receive)
{
	unsigned short int posInData = 5;
	unsigned char OL, IL, counter2 = 0;
	unsigned short int sum1, sum2;
	unsigned char errorCodeIdx;
	static unsigned short int numCyclesSinceLastError[2*NUM_LEG_PCBS+2];

	time_t clk = time(NULL);

	// Initialize fields that may never be written
	p_QUADdata->dataError = 0;

	for (OL = 0; OL < NUM_LEG_PCBS; OL++)
	{
		sum1 = 0;
		sum2 = 0;

		//FSR[OL].firmwareVersion = receive[(unsigned char)posInData];
		//cout << "receive[" << (int)(posInData) << "] = " << (int)p_receive[posInData] << endl;
		//for (IL = 0; IL < 5; IL++)
		//{
		//	cout << "FSR[" << (int)IL << "].firmwareVersion = " << (int)p_FSRdata[IL].firmwareVersion << endl;
		//}
		//cout << endl;
		//FSRdata[0L].firmwareVersion = p_receive[posInData];

		p_FSRdata[counter2].firmwareVersion = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);

		counter2++;
		posInData++;

		//for (IL = 0; IL < 5; IL++)
		//{
		//	cout << "FSR[" << (int)IL << "].firmwareVersion = " << (int)p_FSRdata[IL].firmwareVersion << endl;
		//}
		//cout << "OL = " << (int)OL<< " - 3) FSR[0].firmwareVersion = " << (int)FSRdata[0].firmwareVersion << endl;

		for (IL = 0; IL < NUM_FSRS; IL++)
		{
			// These are 10 bit values sent with the upper byte first
			p_FSRdata[OL].data[IL] = combineValues(p_receive[2*IL+1+posInData], p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+1+posInData]);
		}

		posInData = posInData+48;
		p_FSRdata[OL].dataError = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;
		p_FSRdata[OL].chksum1 = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;
		p_FSRdata[OL].chksum2 = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;

		p_LEGdata[OL].firmwareVersion_ = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			// These are 10 bit values sent with the upper byte first
			p_LEGdata[OL].encoder[IL] = combineValues(p_receive[2*IL+1+posInData], p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+1+posInData]);
		}

		posInData = posInData+10;

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			// These are 10 bit values sent with the upper byte first
			p_LEGdata[OL].motCurrent[IL] = combineValues(p_receive[2*IL+1+posInData], p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+1+posInData]);
		}

		posInData = posInData+10;
		p_LEGdata[OL].dataError = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;
		p_LEGdata[OL].chksum1 = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;
		p_LEGdata[OL].chksum2 = p_receive[posInData];
		sum1sum2(&sum1, &sum2, &p_receive[posInData]);
		posInData++;

		// Because the two computed checksums were read in with the received data, the two running sums should be zero
		// currently, the checksums do not act on the last two pieces of data in the packet (p_MAINdata->firmwareVersion and p_MAINdata->dataError)
		if ((sum1 | sum2) > 0)
		{
			if (OL == 1)
			{
				p_QUADdata->dataError = (p_QUADdata->dataError)	| DATA_ERROR_RX_MAIN_MASK_LEG1;
			}

			else if (OL == 2)
			{
				p_QUADdata->dataError = (p_QUADdata->dataError)	| DATA_ERROR_RX_MAIN_MASK_LEG2;
			}

			else if (OL == 3)
			{
				p_QUADdata->dataError = (p_QUADdata->dataError) | DATA_ERROR_RX_MAIN_MASK_LEG3;
			}

			else if (OL == 4)
			{
				p_QUADdata->dataError = (p_QUADdata->dataError) | DATA_ERROR_RX_MAIN_MASK_LEG4;
			}

			else if (OL == 5)
			{
				p_QUADdata->dataError = (p_QUADdata->dataError) | DATA_ERROR_RX_MAIN_MASK_LEG5;
			}
		}
	}

	sum1 = 0;
	sum2 = 0;

	p_MAINdata->firmwareVersion = p_receive[posInData];
	sum1sum2(&sum1, &sum2, &p_receive[posInData]);
	posInData++;

	p_MAINdata->dataError = p_receive[posInData];
	sum1sum2(&sum1, &sum2, &p_receive[posInData]);
	posInData++;

	p_MAINdata->chksum1 = p_receive[posInData];
	sum1sum2(&sum1, &sum2, &p_receive[posInData]);
	posInData++;

	p_MAINdata->chksum2 = p_receive[posInData];
	sum1sum2(&sum1, &sum2, &p_receive[posInData]);
	posInData++;

	if ((sum1 | sum2) > 0)
	{
		p_QUADdata->dataError = (p_QUADdata->dataError)	| DATA_ERROR_RX_MAIN_MASK_MAIN;
	}

	errorCodeIdx = 0;

	// Write out errors to file if enough cycles have elapsed
	for (OL = 0; OL < NUM_LEG_PCBS; OL++)
	{
		errorCodeIdx++;

		if (p_FSRdata[OL].dataError != 0)
		{
			if (numCyclesSinceLastError[errorCodeIdx] == 0)
			{
				printf("%s\tError code %u on FSR %u\n", ctime(&clk), p_FSRdata[OL].dataError, OL);
				numCyclesSinceLastError[errorCodeIdx] = NUM_CYCLES_BETWEEN_ERROR_LOGS;
			}
		}

		errorCodeIdx++;

		if (p_LEGdata[OL].dataError != 0)
		{
			if (numCyclesSinceLastError[errorCodeIdx] == 0)
			{
				printf("%s\tError code %u on LEG %u\n", ctime(&clk), p_LEGdata[OL].dataError, OL);
				numCyclesSinceLastError[errorCodeIdx] = NUM_CYCLES_BETWEEN_ERROR_LOGS;
			}
		}
	}

	errorCodeIdx++;

	if (p_MAINdata->dataError != 0)
	{
		if (numCyclesSinceLastError[errorCodeIdx] == 0)
		{
			printf("%s\tError code %u on MAIN\n", ctime(&clk), p_MAINdata->dataError);
			//printSensorData(p_LEGdata, p_FSRdata, p_MAINdata, p_QUADdata, 0b10000);
			numCyclesSinceLastError[errorCodeIdx] = NUM_CYCLES_BETWEEN_ERROR_LOGS;
		}
	}

	errorCodeIdx++;

	if (p_QUADdata->dataError != 0)
	{
		if (numCyclesSinceLastError[errorCodeIdx] == 0)
		{
			printf("%s\tError code %u on QUAD\n", ctime(&clk), p_QUADdata->dataError);
			numCyclesSinceLastError[errorCodeIdx] = NUM_CYCLES_BETWEEN_ERROR_LOGS;
		}
	}

	for (OL = 0; OL < (2*NUM_LEG_PCBS+2); OL++)
	{
		if (numCyclesSinceLastError[OL] > 0)
		{
			numCyclesSinceLastError[OL]--;
		}
	}
}

void printSPIstream(unsigned char *p_receive)
{
	unsigned char counter = 0;
	unsigned short int i;

	cout << endl << "Preamble bytes: " << (int)p_receive[1] << " " << (int)p_receive[2] << " " << (int)p_receive[3] << " " << (int)p_receive[4] << endl;

	for (i = 0; i < SPI_TRANSMISSION_SIZE; i++)
	{
		if (counter == 0)
		{
			cout << endl << (unsigned int)i << ") ";
			counter = 10;
		}

		cout << (unsigned int)p_receive[i] << "  ";
		counter--;
	}

	cout << endl;
}

void printSensorData(struct LEG_PCB *p_LEGdata, struct FSR_PCBA *p_FSRdata, struct MAIN_PCBA *p_MAINdata, struct QUAD_ROBOT *p_QUADdata, unsigned char whichToPrint)
{
	unsigned char OL, IL, IIL;

	for (OL = 0; OL < 5; OL++)
	{
		if ((whichToPrint >> OL) & 0b1)
		{
			cout << "--------- [FSR " << (int)(OL+1) << "]------------" << endl;

			for (IL = 0; IL < 2; IL++)
			{
				cout << "FSR Data "<< (int)(IL*12+1) << "-" << (int)((IL+1)*12) << ": ";

				for (IIL = 0; IIL < 12; IIL++)
				{
					cout << (int)p_FSRdata[OL].data[IL*12+IIL] << " ";
				}

				cout << endl;
			}

			cout << "Firmware Version: " << (int)p_FSRdata[OL].firmwareVersion << endl;
			cout << "Checksum 1: " << (int)p_FSRdata[OL].chksum1 << endl;
			cout << "Checksum 2: " << (int)p_FSRdata[OL].chksum2 << endl;
			cout << "Error Code: " << (int)p_FSRdata[OL].dataError << endl;

			cout << "--------- [LEG " << (int)(OL+1) << "]------------" << endl;

			cout << "Encoder Data: ";

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				cout << (int)p_LEGdata[OL].encoder[IL] << " ";
			}

			cout << endl;

			cout << "Motor Current Data: " << endl;

			for (IL = 0; IL < NUM_ENCODERS; IL++)
			{
				cout << (int)p_LEGdata[OL].motCurrent[IL] << " ";
			}

			cout << endl;

			cout << "Firmware Version: " << (int)p_LEGdata[OL].firmwareVersion_ << endl;
			cout << "Checksum 1: " << (int)p_LEGdata[OL].chksum1 << endl;
			cout << "Checksum 2: " << (int)p_LEGdata[OL].chksum2 << endl;
			cout << "Error Code: " << (int)p_LEGdata[OL].dataError << endl;
		}
	}

	cout << "---------- [MAIN]------------" << endl;
	cout << "Firmware Version: " << (int)p_MAINdata->firmwareVersion << endl;
	cout << "Checksum 1: " << (int)p_MAINdata->chksum1 << endl;
	cout << "Checksum 2: " << (int)p_MAINdata->chksum2 << endl;
	cout << "Error Code: " << (int)p_MAINdata->dataError << endl;

	cout << "---------- [QUAD ROBOT]------------" << endl;
	cout << "Error Code: " << (int)p_QUADdata->dataError  << endl;
}
