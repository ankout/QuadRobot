#ifndef MISCFUNCTIONS_H_
#define MISCFUNCTIONS_H_

// Fletcher-8 checksum functions
unsigned short int sum1Helper(unsigned short int, unsigned char);
unsigned short int sum2Helper(unsigned short int, unsigned short int);
unsigned char check1Helper(unsigned short int, unsigned short int);
unsigned char check2Helper(unsigned short int, unsigned char);

unsigned short int combineValues(unsigned char, unsigned char);

#endif /* MISCFUNCTIONS_H_ */
