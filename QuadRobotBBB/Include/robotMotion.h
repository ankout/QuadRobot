/*
 * robotMotion.h
 *
 *  Created on: Aug 3, 2017
 *      Author: acardi
 */

#ifndef ROBOTMOTION_H_
#define ROBOTMOTION_H_

void getMotorCommands(unsigned char *, unsigned char *);
void getJointAngles(unsigned char*, unsigned char*, float *);
void getMotorCommands(unsigned char *, float *, struct LEG_PCB *);

#endif /* ROBOTMOTION_H_ */
