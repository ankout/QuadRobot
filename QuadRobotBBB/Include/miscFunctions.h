#ifndef MISCFUNCTIONS_H_
#define MISCFUNCTIONS_H_

// Fletcher-8 checksum functions
unsigned short int sum1Helper(unsigned short int, unsigned char);
unsigned short int sum2Helper(unsigned short int, unsigned short int);
void sum1sum2(unsigned short int *, unsigned short int *, unsigned char *);

unsigned char check1Helper(unsigned short int, unsigned short int);
unsigned char check2Helper(unsigned short int, unsigned char);

unsigned short int combineValues(unsigned char, unsigned char);

// SPI Functions
void parseSPIfromMAIN(struct LEG_PCB *, struct FSR_PCBA *, struct MAIN_PCBA *, struct QUAD_ROBOT *, unsigned char *);
void printSensorData(struct LEG_PCB *, struct FSR_PCBA *, struct MAIN_PCBA *, struct QUAD_ROBOT *, unsigned char);
void printSPIstream(unsigned char *);

//
void getMotorCommands(unsigned char *);

#endif /* MISCFUNCTIONS_H_ */
