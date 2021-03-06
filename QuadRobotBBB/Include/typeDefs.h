#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#define NUM_CYCLES_BETWEEN_ERROR_LOGS 200
// SPI
#define SPI_TX_DATA_SIZE 30 // 5 board, each with 5 1 byte PWMs and 1 byte direction
#define SPI_PREAMBLE_BYTES 4 // Number of 0xFF bytes to send before message
#define SPI_CHECKSUM_SIZE 2

#define SPI_TRANSMISSION_SIZE 389 	// This is only this long to accommodate the length of received data.
									// Preamble bytes: 4
							      	// 5 LEG PCBs, 76 bytes of data each: 380
									// Software version: 1
									// Error code: 1
									// Checksum bytes: 2 (these only apply to the software version and error code for MAIN)
									// Extra length needed to capture last byte: 1
// FSR
#define NUM_FSRS 24

// LEG
#define NUM_ENCODERS 5

#define NUM_LEG_PCBS 5
#define NUM_LEGS 4

// QUAD
// These checksums are on the five blocks of data in the SPI transmission (one from each LEG PCB)
#define DATA_ERROR_RX_MAIN_MASK_LEG1 0b1
#define DATA_ERROR_RX_MAIN_MASK_LEG2 0b10
#define DATA_ERROR_RX_MAIN_MASK_LEG3 0b100
#define DATA_ERROR_RX_MAIN_MASK_LEG4 0b1000
#define DATA_ERROR_RX_MAIN_MASK_LEG5 0b10000

// This checksum is for data just associated with the MAIN board
#define DATA_ERROR_RX_MAIN_MASK_MAIN 0b100000

struct FSR_PCBA
{
	unsigned char firmwareVersion;
	unsigned short data[24];
	unsigned char dataError;
	unsigned char chksum1;
	unsigned char chksum2;
};

struct LEG_PCB
{
	unsigned char firmwareVersion_;
	unsigned short encoder[5];
	unsigned short motCurrent[5];
	unsigned char dataError;
	unsigned char chksum1;
	unsigned char chksum2;
};

struct MAIN_PCBA
{
	unsigned char firmwareVersion;
	unsigned char dataError;
	unsigned char chksum1;
	unsigned char chksum2;
};

struct QUAD_ROBOT
{
	// Bit fields:
	// 1 - checksum error on data from MAIN
	unsigned char dataError;
};

#endif /* TYPEDEFS_H_ */
