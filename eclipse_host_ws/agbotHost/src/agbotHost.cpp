//============================================================================
// Name        : agbotHost.cpp
// Author      : peiqing xia
// Version     :
// Copyright   : M2Robots
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "uartcollector.h"
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <sys/time.h>
#include "syscfg.h"
#include "pid.h"
#include "misc_math.h"
#ifdef RASPBERRY_PI
	#include "gpioutils.h"
#endif
void sendManualCtrlCmd(int iThrottle, int iPitch, int iLiftThrust);
#define PAYLOAD_BYTE_NUM 7
typedef enum {
	SYS_IDLE = 0,
	ACTIVATE_LIFT_ENG,// lift engine: motors permanently attached to structure
	ACTIVATE_CTRL_ENG // control engine: 2 motors controlled by FC
} SYS_OP_MODE;
enum {
	PID_THROTTLE,
	PID_PITCH,
	PID_MAX
};
struct pid pids[PID_MAX];
float fAngleRd[PID_MAX];
float fManual_ctrl = 0.0f;
using namespace std;
ofstream* hFile;

template<typename T>
void UNUSED(T &&) {}
UARTCollector* uc = nullptr;

static volatile int reqquit = 0;

void updateFlightCtrl(float fManual_ctrl, float *fAngleRd)
{
	// height=>Throttle control
	// forward/backward position=>Pitch control
	float fThrottle, fPitch;
	const float updateIntervalS = float(SamplingMs*SampPerCtrl)/1000.0f;
	//fThrottle = pid_apply_setpoint(&pids[PID_THROTTLE], fManual_ctrl, fAngleRd[VERT_CH], updateIntervalS);
	fThrottle = pid_apply_setpoint(&pids[PID_THROTTLE], 0, fAngleRd[VERT_CH], updateIntervalS) + fManual_ctrl;
	fThrottle = bound_sym(fThrottle,1.0f);
	fPitch = pid_apply(&pids[PID_PITCH], fAngleRd[HOR_CH], updateIntervalS);
	fPitch = bound_sym(fPitch,1.0f);
	int Th_ctrl_uS = iHoverThrotuS + (int)(500.0f*fThrottle);
	int Pitch_ctrl_uS = 1500+500*fPitch;

	if (Th_ctrl_uS > 2000)
		Th_ctrl_uS = 2000;
	if (Th_ctrl_uS < 1000)
		Th_ctrl_uS = 1000;
	if (Pitch_ctrl_uS > 2000)
		Pitch_ctrl_uS = 2000;
	if (Pitch_ctrl_uS < 1000)
		Pitch_ctrl_uS = 1000;

	sendManualCtrlCmd(Th_ctrl_uS,Pitch_ctrl_uS,iLiftThrustuS);
	//sendManualCtrlCmd(Th_ctrl_uS,1500);
}

void sendManualCtrlCmd(int iThrottle, int iPitch, int iLiftThrust) // [1000,2000] (uS)
{
	char buffer[200];
	int n = sprintf (buffer, "Throt:%d Pitch:%d\n",
			iThrottle,iPitch);
	cerr << buffer;
	hFile->write(buffer,n);

	#define CTRL_CMD_BYTES 9
	char writeBuf[CTRL_CMD_BYTES+1]={0};// 1800 = 0x708
	writeBuf[0] = 'M';
	writeBuf[1] = '2';
	writeBuf[2] = 'R';
	writeBuf[3] = (iThrottle >> 8)&0xFF;
	writeBuf[4] = iThrottle&0xFF;
	writeBuf[5] = (iPitch >> 8)&0xFF;
	writeBuf[6] = iPitch&0xFF;
	writeBuf[7] = (iLiftThrust >> 8)&0xFF;
	writeBuf[8] = iLiftThrust&0xFF;

	uc->writePort(1, &writeBuf[0], CTRL_CMD_BYTES);
	uc->poll(0, NULL, uc);
}

SYS_OP_MODE mLastSysOpMode;
void processPayload(unsigned char *p)
{
	SYS_OP_MODE mSysOpMode = (SYS_OP_MODE)p[0];
	int azi_raw = p[1]+p[2]*256;
	int ele_raw = p[3]+p[4]*256;
	int man_raw = p[5]+p[6]*256;

	fAngleRd[HOR_CH] =  (1.0f-LPFcoef)*fAngleRd[HOR_CH]  -1.0f*LPFcoef*((float)azi_raw - (float)Azi_raw_neutral)/1024.0f;
	fAngleRd[VERT_CH] = (1.0f-LPFcoef)*fAngleRd[VERT_CH] -1.0f*LPFcoef*((float)ele_raw - (float)Ele_raw_neutral)/1024.0f;
	fManual_ctrl = (float)( (float)man_raw - (float)Manual_input_raw_neutral )
			/(1.0f*1024.0f);

	char buffer[200];
	int n = sprintf (buffer, " AZI:%d ELE:%d fHor:%5.4f fVer:%5.4f ",
			azi_raw,ele_raw,
			fAngleRd[HOR_CH],fAngleRd[VERT_CH]);
	cerr << buffer;
	hFile->write(buffer,n);

	char tmbuf[16];
	struct timeval tv;
	gettimeofday(&tv, nullptr);
	std::time_t tm = tv.tv_sec;
	std::strftime(tmbuf, sizeof(tmbuf), "_%Y%m%d%H%M%S", std::localtime(&tm));
	int ms = tv.tv_usec / 1000.0; // microseconds to milliseconds
	hFile->write(tmbuf,15);

	n = sprintf (buffer, ".%d,man:%d azi:%d ele:%d fManual_ctrl:%3.2f,mode:%d,",
			ms,man_raw,azi_raw,ele_raw,fManual_ctrl,mSysOpMode);
	cerr << buffer;
	hFile->write(buffer,n);
	//hFile->flush();

	if (SYS_IDLE == mSysOpMode)
	{
		sendManualCtrlCmd(1000,1500,1000);
	}
	else if (ACTIVATE_LIFT_ENG == mSysOpMode)
	{
		sendManualCtrlCmd(1000,1500,iLiftThrustuS);
	}
	else
	{
		if (ACTIVATE_CTRL_ENG != mLastSysOpMode)
		{
			for (int pidii = 0; pidii < PID_MAX; pidii++)
			{
				pid_zero(&pids[pidii]);
			}
		}
		updateFlightCtrl(fManual_ctrl, &(fAngleRd[0]));
	}
	mLastSysOpMode = mSysOpMode;
}

void sigfunc(int signo)
{
	UNUSED(signo);
	reqquit = 1;
}

/**
 * Callback function to get immediate data which may be small data pieces.
 *
 * You can get data from ring buffer instead of callback function if
 * you don't care data overwritten which maybe happen in ring buffer.
 *
 */
void readCallback(void* ctx, int portId, char* buf, ssize_t len) {
	UNUSED(buf);
	UARTCollector* uc = reinterpret_cast<UARTCollector*>(ctx);
	IODevice* p = uc->port(portId);
	UNUSED(p);
	if (len < 0) {
		//clog << "**** Port " << p->devName() << " error: " << p->errstring() << endl;
	} else if (len == 0) {
		//clog << "**** Port " << p->devName() << " no more data" << endl;
	} else {
		//clog << "**** Port " << p->devName() << " has " << len << " new bytes" << endl;
	}
	// At your disposal now :)
}

//string devfile2logfile(const string& devfile, std::time_t tm) {
string devfile2logfile(const string& devfile, struct timeval* tv) {
	string fname(devfile);
	for (size_t i=0; i < fname.size(); i++) {
		char c = fname[i];
		if (c==':' || c=='/') {
			fname[i] = '_';
		}
	}
	char tmbuf[16];
	std::time_t tm = tv->tv_sec;
	std::strftime(tmbuf, sizeof(tmbuf), "_%Y%m%d%H%M%S", std::localtime(&tm));
	int ms = tv->tv_usec / 1000.0; // microseconds to milliseconds
	std::ostringstream oss;
	oss << fname << tmbuf << ".";
	oss.width(3);
	oss.fill('0');
	oss << ms << ".log";
	return oss.str();
}

UARTCollector* init_collector(const vector<string>& iodevs, vector<ofstream*>& files)
{
	assert(!files.size());
	//std::time_t now = std::time(nullptr);
	struct timeval tv;
	gettimeofday(&tv, nullptr);
	UARTCollector* uc = new UARTCollector();
	for (size_t i=0; i<iodevs.size(); i++) {
		string name = iodevs[i];
		if (uc->addPort(name) < 0) {
			cerr << "Error add port '" << name << "':" << uc->error()
					<< " " << uc->errstring() << endl;
			delete uc;
			return nullptr;
		}

		/*string logfile = devfile2logfile(name, &tv);
		ofstream* f = new ofstream(logfile, ios::out|ios::app|ios::binary);
		if (!f->is_open()) {
			cerr << "Error open log file " << logfile << endl;
			delete uc;
			return nullptr;
		}
		files.push_back(f);*/
	}
	return uc;
}

void destroy_collector(UARTCollector* col, vector<ofstream*>& files)
{
	/*for(size_t i=0; i<files.size(); i++) {
		files[i]->close();
		delete files[i];
	}
	files.clear();*/
	delete col;
}

void usage(int argc, const char* argv[])
{
	UNUSED(argc);
	cerr << "USAGE: \n    " << argv[0] << " -iodev <IODev1>[,<IODev2>[,...]]"
#ifdef RASPBERRY_PI
		<< " -gpioin <pin> -gpioout <pin>"
#endif
		<< endl;
	cerr << "Arguments:" << endl;
	cerr << "    <IODevN> Device to poll, such as:" << endl;
	cerr << "        TCP:192.168.0.1:1977" << endl;
	cerr << "        UDP:192.168.0.1:1977" << endl;
	cerr << "        Serial:/dev/ttyUSB0:57600" << endl;
}

typedef enum{
	PREAMBLE_M,
	PREAMBLE_2,
	PREAMBLE_R,
	PAYLOAD_BYTES
} DECODE_EXPECTING_STATUS; // the data currently waiting for.
DECODE_EXPECTING_STATUS mDECODE_STATUS = PREAMBLE_M;
int PayloadByteCnt = 0;
unsigned char PayloadBuf[7];

int main(int argc, const char* argv[]) {
	if (argc == 1) { // No options and parameters
		usage(argc, argv);
		return -1;
	}
#ifdef RASPBERRY_PI
	int pinin  = -1;
	int pinout = -1;
#endif
	vector<string> iodevs;
	for (int i=1; i<argc; i++) {
		string opt(argv[i]);
		if (opt == "-iodev") {
			iodevs = IODevice::ioargs(argv[++i], ',');
			auto it = iodevs.begin();
			while (it != iodevs.end()) {
				cerr << "Accepted io device " << *it << endl;
				++it;
			}
#ifdef RASPBERRY_PI
		} else if (opt == "-gpioin") {
			pinin = std::stoi(argv[++i]);
		} else if (opt == "-gpioout") {
			pinout = std::stoi(argv[++i]);
#endif
		} else {
			cerr << "Option '" << opt << "' not accepted" << endl;
			return -1;
		}
	}
	if (!iodevs.size()) {
		cerr << "Option '-iodev' is required" << endl;
		return -1;
	}

#ifdef RASPBERRY_PI
	if (pinin<0 || pinout<0) {
		cerr << "Option '-gpioin' and '-gpioout' is required" << endl;
		return -1;
	}

	// Setup GPIO
	if (gpio_init() != 0) {
		cerr << "GPIO initialization error: " << gpio_errno()
			<< " " << gpio_errstring() << endl;
		return -1;
	}
	gpio_set_input(pinin);
	cerr << "GPIO input set to pin " << pinin << endl;
	gpio_set_output(pinout);
	cerr << "GPIO output set to pin " << pinout << endl;
	gpio_set_pull_updown(pinin, 1); // internal pull up
	cerr << "GPIO output pin enabled pullup" << endl;
#endif

	pid_configure(&pids[PID_THROTTLE],fThKp,fThKi,fThKd,fThIlim);
	pid_configure(&pids[PID_PITCH],fPitchKp,fPitchKi,fPitchKd,fPitchIlim);

	for (int pidii = 0; pidii < PID_MAX; pidii++)
	{
		pid_zero(&pids[pidii]);
	}

	// Press Ctrl+C or use command kill to make program quit
	signal(SIGINT, sigfunc);
	signal(SIGTERM, sigfunc);

	vector<ofstream*> files;

	string logfile = "/home/dev/Documents/test.txt";
	hFile = new ofstream(logfile, ios::out);
	if (!hFile->is_open()) {
		cerr << "Error open log file " << logfile << endl;
	}

#ifdef RASPBERRY_PI
	int lastlevel = 1; // Default is high
#endif
	while (!reqquit) {
#ifdef RASPBERRY_PI
		int level = gpio_input(pinin);
		if (level!=lastlevel && !lastlevel && level) { // Triggered when key released
			if (uc) {
				cerr << "Destroying collector" << endl;
				destroy_collector(uc, files);
				uc = nullptr;
				gpio_output(pinout, 0);
			} else {
				cerr << "Initializing collector" << endl;
				uc = init_collector(iodevs, files);
				if (!uc) {
					cerr << "Failed initilzation of collector" << endl;
					break;
				}
				gpio_output(pinout, 1);
			}
		}
		lastlevel = level;
#else
		if (!uc) {
			cerr << "Initializing collector" << endl;
			uc = init_collector(iodevs, files);
			if (!uc) {
				cerr << "Failed initialization of collector" << endl;
				break;
			}
			else
				cerr << "collector Initialized" << endl;
		}
#endif

		if (uc) {
			int rv = uc->poll(10, readCallback, uc);
			if (rv > 0) {
				for (int i=0; i<uc->numberOfPorts(); i++) {
					//if( i == 0 ) // ttyUSB0: arduino
					{
						IODevice* p = uc->port(i);
						if (p->error()) { // Port error
							continue;
						}

						size_t ByteRdy = uc->readBufferUsed(i);
						if (!ByteRdy) { // No data to read
							continue;
						}
						// Allocate ByteRdy+1 bytes, filled with 0
						vector<char> buf(ByteRdy+1, 0);
						// Receive data from reading buffer of serial device
						uc->readPort(i, &buf[0], ByteRdy);
						// Print the data just received
						for (uint ii = 0; ii < ByteRdy; ii++)
						{
							char cc = buf[ii];
							switch (mDECODE_STATUS)
							{
							case PREAMBLE_M:
								if (cc == 'M')
									mDECODE_STATUS = PREAMBLE_2;
								break;
							case PREAMBLE_2:
								if (cc == '2')
									mDECODE_STATUS = PREAMBLE_R;
								else
									mDECODE_STATUS = PREAMBLE_M;
								break;
							case PREAMBLE_R:
								if (cc == 'R')
								{
									PayloadByteCnt = 0;
									mDECODE_STATUS = PAYLOAD_BYTES;
								}
								else
									mDECODE_STATUS = PREAMBLE_M;
								break;
							case PAYLOAD_BYTES:
								PayloadBuf[PayloadByteCnt++] = cc;
								if (PayloadByteCnt == PAYLOAD_BYTE_NUM)
								{
									mDECODE_STATUS = PREAMBLE_M;
									processPayload(&(PayloadBuf[0]));
								}
								break;
							}
						}
	#if false
						cout << "Port " << p->name() << " >> " << &buf[0] << endl;
						files[i]->write(&buf[0], ByteRdy);
						files[i]->flush();
	#endif
						// Echo data back to writting buffer of serial device.
						// Data will be sent after next calling of poll.
						//uc.writePort(i, &buf[0], ByteRdy);
					}
				}
			} else if (rv < 0) { // Error happened
				cerr << "Error polled collector: " << uc->error()
					<< " " << uc->errstring() << endl;
				break;
			}
		}
	}

	if (uc) { destroy_collector(uc, files); }
#ifdef RASPBERRY_PI
	gpio_output(pinout, 0);
#endif

	hFile->close();
	if (reqquit) {
		cerr << "Quit requested, bye." << endl;
		return 0;
	}
	return -1;
}


