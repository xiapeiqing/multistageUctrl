/*
 * syscfg.h
 *
 *  Created on: Feb 8, 2018
 *      Author: dev
 */

#ifndef AGBOTS_INCLUDE_SYSCFG_H_
#define AGBOTS_INCLUDE_SYSCFG_H_

enum{
    HOR_CH = 0,
    VERT_CH
};

//#define ENABLE_RAW_OUTPUT
#define Azi_raw_neutral 334
#define Ele_raw_neutral 390 //  floor ceil
#define Manual_input_raw_neutral 331

/*
#define fThKp 15.0f
#define fThKi 0.15f
#define fThKd 0.0f
#define fThIlim 0.15f

#define fPitchKp 15.0f
#define fPitchKi 0.0f
#define fPitchKd 0.0f
#define fPitchIlim 0.0f
*/
#define fThKp 4.0f
#define fThKi 0.0f
#define fThKd 0.0f
#define fThIlim 0.0f

#define fPitchKp 0.2f
#define fPitchKi 0.0f
#define fPitchKd 0.0f
#define fPitchIlim 0.0f

#define iHoverThrotuS 1620  //1450 1520
#define iLiftThrustuS 1400

#define LPFcoef 0.1f

#define SamplingMs 5 // sampling interval in ms SamplingMs*SampPerCtrl
#define SampPerCtrl 10



#endif /* AGBOTS_INCLUDE_SYSCFG_H_ */
