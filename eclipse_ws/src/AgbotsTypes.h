/*
 * AgbotsTypes.h
 *
 *  Created on: Apr 30, 2018
 *      Author: dev
 */

#ifndef AGBOTSTYPES_H_
#define AGBOTSTYPES_H_

#define PC_DEBUG
typedef enum {
	SYS_IDLE = 0,
	ACTIVATE_LIFT_ENG,// lift engine: motors permanently attached to structure
	ACTIVATE_CTRL_ENG // control engine: 2 motors controlled by FC
} SYS_OP_MODE;

void sendManualCtrlCmd(int Th_ctrl_uS, int Pitch_ctrl_uS);
void updateFlightCtrl(float fManual_ctrl, float *fAngleRd);

#endif /* AGBOTSTYPES_H_ */
