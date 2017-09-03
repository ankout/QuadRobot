#ifndef JOINTCONSTANTS_H_
#define JOINTCONSTANTS_H_

#define DEG_PER_BIT 0.087890625f // Number of degrees per bit of a 4096 count encoder

const unsigned short int minEncRead[20] = {
		100, 100, 100, 1230, 1420, // Leg 1
		100, 100, 100, 100, 100, // Leg 2
		100, 100, 100, 100, 100, // Leg 3
		100, 100, 100, 100, 100 // Leg 4
};

const unsigned short int maxEncRead[20] = {
		2000, 2000, 2000, 2386, 2454, // Leg 1
		2000, 2000, 2000, 2000, 2000, // Leg 2
		2000, 2000, 2000, 2000, 2000, // Leg 3
		2000, 2000, 2000, 2000, 2000 // Leg 4
};

const float angleAtMinEncoderReading[20] = {
		30.6f, 30.6f, 30.6f, -45.0f, -45.0f, // Leg 1
		30.6f, 30.6f, 30.6f, 30.6f, 30.6f, // Leg 2
		30.6f, 30.6f, 30.6f, 30.6f, 30.6f, // Leg 3
		30.6f, 30.6f, 30.6f, 30.6f, 30.6f // Leg 4
};

// This is the direction of the change in encoder counts with increasing angle
// A positive number indicates that an increase in encoder count is a positive change in joint angle
const float encDir[20] = {
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f, // Leg 1
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f, // Leg 2
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f, // Leg 3
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f  // Leg 4
};

// PID Gains

#define TS 0.01f // sampling period in seconds

const float P_GAIN[20] = {
		1.0f, 1.0f, 1.0f, 0.0001f, 0.0001f, // Leg 1
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f, // Leg 2
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f, // Leg 3
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f  // Leg 4
};

const float I_GAIN[20] = {
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Leg 1
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Leg 2
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Leg 3
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f  // Leg 4
};

const float D_GAIN[20] = {
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Leg 1
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Leg 2
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, // Leg 3
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f  // Leg 4
};

// There are 25 total motor channels and only 20 physical joints.
// All of the motion planning and control loop calculations are done
// assuming there are 20 motor channels and 20 joints, where:
//
// 		Leg 1: [0]-[4], where [0] is the first joint
// 		Leg 2: [5]-[9], where [5] is the first joint
// 		Leg 3: [10]-[14], where [10] is the first joint
// 		Leg 4: [15]-[19], where [15] is the first joint
//
// In reality, the first joint in each leg has two motors and one of the LEG PCBs
// only has 10A diode protection, so motors and channels need to get reassigned
// before commands are sent out.  This is done at the very end after all calculations
// to make code more maintainable.
//
// jointAssignments[] contains the mapping from the computed control actions (duty cycles)
// from u[20] in robotmotion.cpp to 25 positions in motorCommand[]. The duty cycle
// commands are not continuous in motorCommand[] like they are in u[] because motorCommand[]
// interleaves direction commands after each group of five motor commands (one direction command per PCB)
// If a number is repeated in jointAssignments[] that means that one joint is driven by two motors
const unsigned char jointMapping[25] = {
		0, 0, 1, 2, 3,
		5, 5, 6, 7, 8,
		10, 10, 11, 12, 13,
		15, 15, 16, 17, 18,
		4, 9, 14, 19, 19}; // The last entry doesn't matter because there is no motor on that channel


#define U_MAX 10.0f


#endif /* JOINTCONSTANTS_H_ */
