#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

// SPI
#define SPI_TX_DATA_SIZE 30 // 5 board, each with 5 1 byte PWMs and 1 byte direction
#define SPI_PREAMBLE_BYTES 4 // Number of 0xFF bytes to send before message
#define SPI_CHECKSUM_SIZE 2
#define SPI_TRANSMISSION_SIZE 36 // SPI_TX_DATA_SIZE+SPI_PREAMBLE_BYTES

#endif /* TYPEDEFS_H_ */
