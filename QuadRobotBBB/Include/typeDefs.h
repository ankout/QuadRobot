#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

// SPI
#define SPI_TX_DATA_SIZE 30 // 5 board, each with 5 1 byte PWMs and 1 byte direction
#define SPI_PREAMBLE_BYTES 4 // Number of 0xFF bytes to send before message
#define SPI_CHECKSUM_SIZE 2

#define SPI_TRANSMISSION_SIZE 387 	// This is only this long to accommodate the length of received data.
							      	// 5 LEG PCBs, 76 bytes of data each: 380
									// Software version: 1
									// Error code: 1
									// Extra length needed to capture last byte: 1
// FSR
#define NUM_FSRS 24

// LEG
#define NUM_ENCODERS 5


#endif /* TYPEDEFS_H_ */
