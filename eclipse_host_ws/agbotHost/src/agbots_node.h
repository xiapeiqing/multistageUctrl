#ifndef AGBOTS_NODE_H
#define AGBOTS_NODE_H

#include <math.h>
#include <vector>
#include "debug_printf.h"
#include "syscfg.h"


#ifdef PRODUCE_PCD_RECORD
#ifdef OFFLINE_PROCESS
#error "In order to produce PCD saved file, we have to be run program in online mode, instead of offline replay mode"
#endif
#endif

typedef struct{
	float x_minus;
	float x_plus;
	float y_minus;
	float y_plus;
	float z_minus;
	float z_plus;
} sBoxCfg;


template<typename T>
void UNUSED(T &&) {}

string PCDfilename(struct timeval* tv);

#define MAX_TIMER_CNT 3
float estimateRate(int TimerN, int dbgOutputLevel);



#endif
