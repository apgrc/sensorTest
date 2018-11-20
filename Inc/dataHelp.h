/*
 * dataHelp.h
 *
 *  Created on: Aug 8, 2018
 *      Author: alex
 */

#ifndef DATAHELP_H_
#define DATAHELP_H_

#endif /* DATAHELP_H_ */

#pragma pack(2)
typedef struct __dataOut {
	float time;
	int32_t position1;
	int32_t position2;
	int16_t distance[5];
} Data;

enum SerialCommand {
	Hello = 0, Start, Continue, Stop
};

#pragma pack(2)
typedef struct _SerialInput {
	enum SerialCommand Instruction;
	int16_t PWM_INPUT1;
	int16_t PWM_INPUT2;
} SerialInput;
