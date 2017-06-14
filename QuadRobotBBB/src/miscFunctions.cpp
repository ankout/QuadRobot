#include <iostream>
#include <sstream>
#include "miscFunctions.h"
#include "typeDefs.h"

using namespace std;

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
	p_sum1 = ((*p_sum1+*p_data) % 255);
	p_sum2 = ((*p_sum1+*p_sum2) % 255);
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

void parseSPIfromMAIN(struct LEG_PCB *p_LEGdata, struct FSR_PCBA *p_FSRdata, struct MAIN_PCBA *p_MAINdata, unsigned char *p_receive)
{
	unsigned short int posInData = 5;
	unsigned char OL, IL, counter2 = 0;
	unsigned short int sum1, sum2;

	for (OL = 0; OL < NUM_LEG_PCBS; OL++)
	{
		sum1 = 0, sum2 = 0;

		//cout << endl << "OL = " << (int)OL<< endl;
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
			p_FSRdata[OL].data[IL] = combineValues(p_receive[2*IL+posInData]&0b00000011, p_receive[2*IL+1+posInData]);
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
			p_LEGdata[OL].encoder[IL] = combineValues(p_receive[2*IL+posInData]&0b00000011, p_receive[2*IL+1+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+posInData]);
			sum1sum2(&sum1, &sum2, &p_receive[2*IL+1+posInData]);
		}

		posInData = posInData+10;

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			// These are 10 bit values sent with the upper byte first
			p_LEGdata[OL].motCurrent[IL] = combineValues((p_receive[2*IL+posInData] & 0b00000011), p_receive[2*IL+1+posInData]);
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

		if ((sum1 | sum2) > 0)
		{

		}
	}

	p_MAINdata->firmwareVersion = p_receive[posInData];
	posInData++;
	p_MAINdata->dataError = p_receive[posInData];
	posInData++;
}

void printSPIstream(unsigned char *p_receive)
{
	unsigned char counter = 0;
	unsigned short int i;

	cout << endl << "Preamble bytes are: " << (int)p_receive[1] << " " << (int)p_receive[2] << " " << (int)p_receive[3] << " " << (int)p_receive[4] << endl;

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

void printSensorData(struct LEG_PCB *p_LEGdata, struct FSR_PCBA *p_FSRdata, struct MAIN_PCBA *p_MAINdata)
{
	unsigned char OL, IL, IIL;

	for (OL = 0; OL < 5; OL++)
	{
		cout << "--------- [FSR " << (int)(OL+1) << "]------------" << endl;
		cout << "Firmware Version: " << (int)p_FSRdata[OL].firmwareVersion << endl;
		cout << "Error Flag: " << (int)p_FSRdata[OL].dataError << endl;
		cout << "Checksum 1: " << (int)p_FSRdata[OL].chksum1 << endl;
		cout << "Checksum 2: " << (int)p_FSRdata[OL].chksum2 << endl;

		for (IL = 0; IL < 2; IL++)
		{
			cout << "FSR Data "<< (int)(IL*12+1) << "-" << (int)((IL+1)*12) << ": ";

			for (IIL = 0; IIL < 12; IIL++)
			{
				cout << (int)p_FSRdata[OL].data[IL*12+IIL] << " ";
			}

			cout << endl;
		}

		cout << "--------- [LEG " << (int)(OL+1) << "]------------" << endl;
		cout << "Firmware Version: " << (int)p_LEGdata[OL].firmwareVersion_ << endl;
		cout << "Error Flag: " << (int)p_LEGdata[OL].dataError << endl;
		cout << "Checksum 1: " << (int)p_LEGdata[OL].chksum1 << endl;
		cout << "Checksum 2: " << (int)p_LEGdata[OL].chksum2 << endl;

		cout << "Encoder Data: ";

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			cout << (int)p_LEGdata[OL].encoder[IL] << " ";
		}

		cout << endl;

		cout << "Motor Current Data: ";

		for (IL = 0; IL < NUM_ENCODERS; IL++)
		{
			cout << (int)p_LEGdata[OL].motCurrent[IL] << " ";
		}

		cout << endl;
	}

	cout << "---------- [MAIN]------------" << endl;
	cout << "Firmware Version: " << (int)p_MAINdata->firmwareVersion << endl;
	cout << "Error Flag: " << (int)p_MAINdata->dataError << endl;
}
