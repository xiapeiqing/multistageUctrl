// #define NEUTRAL_POS_DEBUG // ttyUSBx output: mSysOpMode:2, azi:343, ele:353, manual:353
#include "AgbotsTypes.h"
//#define USE_AS5048A

#define MANUAL_CTRL_ANALOG_IN_PIN 0
#define MODE_SEL_IN_PIN 1

#ifndef USE_AS5048A
#define AZI_IN_PIN 3
#define ELE_IN_PIN 4
#endif

#define servo_pwm_ctrl_PIN 2

enum{
    HOR_CH = 0,
    VERT_CH
};

//#define ENABLE_RAW_OUTPUT
#define Azi_raw_neutral 354
#define Ele_raw_neutral 353
#define Manual_input_raw_neutral 331

#define fThKp 3.0f
#define fThKi 0.0f
#define fThKd 0.0f
#define fThIlim 0.0f

#define fPitchKp 3.0f
#define fPitchKi 0.0f
#define fPitchKd 0.0f
#define fPitchIlim 0.0f 

#define iHoverThrotMs 1100

#define LPFcoef 0.1f

#define SamplingMs 5 // sampling interval in ms SamplingMs*SampPerCtrl
#define SampPerCtrl 10

#include "Arduino.h"
#include "pid.h"
#include "misc_math.h"
#include <Servo.h>
Servo servo_pwm_ctrl;
#define Lift_Eng_IDLE_PWM_uS 1000
#define Lift_Eng_PWM_uS 1600

#ifdef USE_AS5048A
// The sensor should be connected to the hardware SPI pins (MISO, MOSI, SCK). The CS pin can be connected to any GPIO pin but should be passed to the constructor.
// 3-wire SPI (MOSI not connected) not supported!
#include "AS5048A.h"
#endif

enum {
	PID_THROTTLE,
	PID_PITCH,
	PID_MAX
};

SYS_OP_MODE mSysOpMode = SYS_IDLE;
SYS_OP_MODE mPendingSysOpMode = SYS_IDLE;
uint8_t PendingSysOpCnt = 0;

struct pid pids[PID_MAX];
float fManualHeightCtrl = 0.0; // positive: right bank

unsigned long curr_ms = 0;
unsigned long last_ms = 0;

int iiLPF = 0;
float fAngleRd[PID_MAX];

float fManual_ctrl = 0.0f;

#ifdef USE_AS5048A
AS5048A angleSensor0(8); // height change
AS5048A angleSensor1(9);  // forward/backward change
#endif

//The setup function is called once at startup of the sketch
void setup()
{
	analogReference(EXTERNAL);
	// Add your initialization code here
	Serial.begin(115200);
	pid_configure(&pids[PID_THROTTLE],fThKp,fThKi,fThKd,fThIlim);
	pid_configure(&pids[PID_PITCH],fPitchKp,fPitchKi,fPitchKd,fPitchIlim);

	for (int pidii = 0; pidii < PID_MAX; pidii++)
	{
		pid_zero(&pids[pidii]);
	}
    #ifdef USE_AS5048A
        angleSensor0.init();
        angleSensor1.init();
    #endif
    fManual_ctrl = ((float)analogRead(MANUAL_CTRL_ANALOG_IN_PIN) - (float)Manual_input_raw_neutral)/1024.0f;
	
	servo_pwm_ctrl.attach(servo_pwm_ctrl_PIN);
	servo_pwm_ctrl.write((uint32_t)(Lift_Eng_IDLE_PWM_uS-540)*180/1860);
}

void activateLiftEng()
{
	//[0,90,180]=>[540,1480,2400]uS
	int ctrl_0to180 = (uint32_t)(Lift_Eng_PWM_uS-540)*180/1860;
	if (ctrl_0to180 < 0)
		ctrl_0to180 = 0;
	if (ctrl_0to180 > 180)
		ctrl_0to180 = 180;
	Serial.println(ctrl_0to180);
	servo_pwm_ctrl.write(ctrl_0to180);
}

SYS_OP_MODE getUserModeInput()
{
	int mode_raw = analogRead(MODE_SEL_IN_PIN);
	// idle:669 turn on lift:626 full operation:569
	if (mode_raw < 597)
		return 	ACTIVATE_CTRL_ENG;
	else if (mode_raw < 647)
		return ACTIVATE_LIFT_ENG;
	else
		return SYS_IDLE;
}

// The loop function is called in an endless loop
void loop()
{
	curr_ms = millis();

	if (curr_ms - last_ms > SamplingMs) // unsigned long will take care of the u32 overflow
	{
        int azi_raw, ele_raw, man_raw;
		last_ms = curr_ms;

		bool enterNewState = false;

		SYS_OP_MODE currOpMode = getUserModeInput();
		if (currOpMode == mSysOpMode)
		{
			PendingSysOpCnt = 0;
		}
		else
		{
			if (currOpMode == mPendingSysOpMode)
			{
				PendingSysOpCnt++;
				if (PendingSysOpCnt == 5)
				{
					enterNewState = true;
					PendingSysOpCnt = 0;
					mSysOpMode = currOpMode;
				}
			}
			else
			{
				PendingSysOpCnt = 0;
				mPendingSysOpMode = currOpMode;
			}
		}

		bool bForceStopFCctrl = true;
		switch (mSysOpMode)
		{
		case SYS_IDLE:
		default:
			if (enterNewState)
			{
				servo_pwm_ctrl.write((uint32_t)(Lift_Eng_IDLE_PWM_uS-540)*180/1860);
			}
			bForceStopFCctrl = true;
			break;
		case ACTIVATE_LIFT_ENG:
			if (enterNewState)
			{
				activateLiftEng();
			}
			bForceStopFCctrl = true;
			break;
		case ACTIVATE_CTRL_ENG:
			if (enterNewState)
			{
				activateLiftEng();
				for (int pidii = 0; pidii < PID_MAX; pidii++)
				{
					pid_zero(&pids[pidii]);
				}
			}
			bForceStopFCctrl = false;
			break;
		}
#ifdef PC_DEBUG
		azi_raw = analogRead(AZI_IN_PIN);
		ele_raw = analogRead(ELE_IN_PIN);
		man_raw = analogRead(MANUAL_CTRL_ANALOG_IN_PIN); // range [9,703]
		char writeBuf[10]={0};// 1800 = 0x708
		writeBuf[0] = 'M';
		writeBuf[1] = '2';
		writeBuf[2] = 'R';
		writeBuf[3] = mSysOpMode;
		writeBuf[4] = azi_raw&0xFF;
		writeBuf[5] = (azi_raw >> 8)&0xFF;
		writeBuf[6] = ele_raw&0xFF;
		writeBuf[7] = (ele_raw >> 8)&0xFF;
		writeBuf[8] = man_raw&0xFF;
		writeBuf[9] = (man_raw >> 8)&0xFF;
#ifdef NEUTRAL_POS_DEBUG
		Serial.print("mSysOpMode:");
		Serial.print(mSysOpMode);
		Serial.print(", azi:");
		Serial.print(azi_raw);
		Serial.print(", ele:");
		Serial.print(ele_raw);
		Serial.print(", manual:");
		Serial.println(man_raw);
#else
		Serial.write(writeBuf,10);
#endif

#else
		if (bForceStopFCctrl)
		{
			sendManualCtrlCmd((int)1000,(int)1500);
		}
		else
		{
			#ifdef USE_AS5048A
				azi_raw = angleSensor0.getRawRotation();
			#else
				azi_raw = analogRead(AZI_IN_PIN);
			#endif
			#ifdef ENABLE_RAW_OUTPUT
				Serial.print("azi_raw:");
				Serial.print(azi_raw);
			#endif
			fAngleRd[HOR_CH] = fAngleRd[HOR_CH]*(1-LPFcoef)+LPFcoef*(azi_raw - Azi_raw_neutral)/1024.0f;

			#ifdef USE_AS5048A
				ele_raw = angleSensor1.getRawRotation();
			#else
				ele_raw = analogRead(ELE_IN_PIN);
			#endif
			#ifdef ENABLE_RAW_OUTPUT
				Serial.print("ele_raw:");
				Serial.print(ele_raw);
			#endif
			fAngleRd[VERT_CH] = fAngleRd[VERT_CH]*(1-LPFcoef)+LPFcoef*(ele_raw - Ele_raw_neutral)/1024.0f;

			man_raw = analogRead(MANUAL_CTRL_ANALOG_IN_PIN);
			fManual_ctrl = fManual_ctrl*(1-LPFcoef)+LPFcoef*((float)man_raw - (float)Manual_input_raw_neutral)/(4*1024.0f);
			#ifdef ENABLE_RAW_OUTPUT
				Serial.print("man_raw:");
				Serial.println(man_raw);
			#endif
			iiLPF ++;
			if (iiLPF == SampPerCtrl)
			{
				iiLPF = 0;
				updateFlightCtrl(fManual_ctrl, &(fAngleRd[0]));
			}
		}
#endif
	}
}

#ifndef PC_DEBUG
void sendManualCtrlCmd(int Th_ctrl_uS, int Pitch_ctrl_uS)
{
	#define CTRL_CMD_BYTES 7
	char writeBuf[CTRL_CMD_BYTES+1]={0};// 1800 = 0x708
	writeBuf[0] = 'M';
	writeBuf[1] = '2';
	writeBuf[2] = 'R';
	writeBuf[3] = (Th_ctrl_uS >> 8)&0xFF;
	writeBuf[4] = Th_ctrl_uS&0xFF;
	writeBuf[5] = (Pitch_ctrl_uS >> 8)&0xFF;
	writeBuf[6] = Pitch_ctrl_uS&0xFF;
	#if true
    Serial.print("Th(uS):");
    Serial.print(Th_ctrl_uS);
    Serial.print("Pitch:");
    Serial.println(Pitch_ctrl_uS);
    #else
	Serial.write(writeBuf,7);
    #endif
}

void updateFlightCtrl(float fManual_ctrl, float *fAngleRd)
{
    #if false
    Serial.print("fManual_ctrl:");
    Serial.print(fManual_ctrl);
    Serial.print("fAngleRd[HOR_CH]:");
    Serial.print(fAngleRd[HOR_CH]);
    Serial.print("fAngleRd[VERT_CH]:");
    Serial.println(fAngleRd[VERT_CH]);
    #endif
	// height=>Throttle control
	// forward/backward position=>Pitch control
	float fThrottle, fPitch;
	const float updateIntervalS = float(SamplingMs*SampPerCtrl)/1000.0f;
	fThrottle = pid_apply_setpoint(&pids[PID_THROTTLE], fManual_ctrl, fAngleRd[VERT_CH], updateIntervalS);
	fThrottle = bound_sym(fThrottle,1.0f);
	fPitch = pid_apply(&pids[PID_PITCH], fAngleRd[HOR_CH], updateIntervalS);
	fPitch = bound_sym(fPitch,1.0f);
	int Th_ctrl_uS = iHoverThrotMs + (int)(500.0f*fThrottle);
	int Pitch_ctrl_uS = 1500+500*fPitch;
	sendManualCtrlCmd(Th_ctrl_uS, Pitch_ctrl_uS);
}
#endif
