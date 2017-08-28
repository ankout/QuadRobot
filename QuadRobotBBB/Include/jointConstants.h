#ifndef JOINTCONSTANTS_H_
#define JOINTCONSTANTS_H_

#define DEG_PER_BIT 0.087890625f // Number of degrees per bit of a 4096 count encoder

const unsigned short int minEncRead[20] = {
		100, 100, 100, 100, 100, // Leg 1
		100, 100, 100, 100, 100, // Leg 2
		100, 100, 100, 100, 100, // Leg 3
		100, 100, 100, 100, 100 // Leg 4
};

const unsigned short int maxEncRead[20] = {
		2000, 2000, 2000, 2000, 2000, // Leg 1
		2000, 2000, 2000, 2000, 2000, // Leg 2
		2000, 2000, 2000, 2000, 2000, // Leg 3
		2000, 2000, 2000, 2000, 2000 // Leg 4
};

const float angleAtMinEncoderReading[20] = {
		30.6, 30.6, 30.6, 30.6, 30.6, // Leg 1
		30.6, 30.6, 30.6, 30.6, 30.6, // Leg 2
		30.6, 30.6, 30.6, 30.6, 30.6, // Leg 3
		30.6, 30.6, 30.6, 30.6, 30.6 // Leg 4
};

// This is the direction of the chanmge in encoder counts with increasing angle
// A positive number indicates that an inrease in encoder count is a positive change in joint angle
const float encDir[20] = {
		1.0f,1.0f,1.0f,1.0f,-1.0f, // Leg 1
		1.0f,1.0f,1.0f,1.0f,-1.0f, // Leg 2
		1.0f,1.0f,1.0f,1.0f,-1.0f, // Leg 3
		1.0f,1.0f,1.0f,1.0f,-1.0f  // Leg 4
};

#endif /* JOINTCONSTANTS_H_ */
