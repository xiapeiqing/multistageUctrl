#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <cassert>
#include "uartport.h"


static UARTPort::ParityType string2parity(const std::string& sp) {
	if (sp=="NONE") return UARTPort::PARITY_NONE;
	else if (sp=="ODD") return UARTPort::PARITY_ODD;
	else if (sp=="EVEN") return UARTPort::PARITY_EVEN;
	//else if (sp=="MARK") return UARTPort::PARITY_MARK;
	//else if (sp=="SPACE") return UARTPort::PARITY_SPACE;
	else return UARTPort::PARITY_NONE;
}

static int parity2int(UARTPort::ParityType parity) {
	switch(parity) {
	case UARTPort::PARITY_NONE:
		return 0;
	case UARTPort::PARITY_ODD:
		return PARENB|PARODD;
	case UARTPort::PARITY_EVEN:
		return PARENB;
	/*case UARTPort::PARITY_MARK:
		return PARENB|PARODD|CMSPAR;
	case UARTPort::PARITY_SPACE:
		return PARENB|CMSPAR;*/
	default:
		return 0;
	}
}

static int baudrate2int(int rate) {
	switch(rate) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	default:
		return B9600;
	}
}

// Serial:DevicePath[:Baud[:ParityType]]
UARTPort::UARTPort(const std::string& name): IODevice(name)
{
	std::vector<std::string> args = ioargs(name);
	assert(args.size()>=2);
	assert(args[0]=="Serial");

	m_devName = args[1];
	m_rate = (args.size()>2) ? std::stoi(args[2]):115200;
	m_parity = (args.size()>3) ? string2parity(args[3]):PARITY_NONE;
	m_errno = 0;
	m_fd = -1;
}


UARTPort::~UARTPort() {
	this->close();
}


bool UARTPort::open() {
     int fd = ::open(m_devName.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
     if (fd < 0) {
    	 return setSystemError((int)errno);
     }

     struct termios tty;
      memset (&tty, 0, sizeof tty);
     if (tcgetattr (fd, &tty) != 0)
     {
    	 setSystemError((int)errno);
    	 ::close(fd);
    	 return false;
     }

     cfsetospeed (&tty, baudrate2int(m_rate));
     cfsetispeed (&tty, baudrate2int(m_rate));

     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
     // disable IGNBRK for mismatched speed tests; otherwise receive break
     // as \000 chars
     tty.c_iflag &= ~IGNBRK;         // disable break processing
     tty.c_lflag = 0;                // no signaling chars, no echo,
                                     // no canonical processing
     tty.c_oflag = 0;                // no remapping, no delays
     tty.c_cc[VMIN]  = 0;            // read doesn't block
     tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
     tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
     tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                     // enable reading
     tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
     int parity = parity2int(m_parity);
     tty.c_cflag |= parity;
     tty.c_cflag &= ~CSTOPB;
     tty.c_cflag &= ~CRTSCTS;

     if (tcsetattr (fd, TCSANOW, &tty) != 0)
     {
    	 setSystemError((int)errno);
    	 ::close(fd);
    	 return false;
     }

     m_fd = fd;
     return true;
}


void UARTPort::close() {
	if (m_fd >= 0) {
		::close(m_fd);
		m_fd = -1;
	}
}


ssize_t UARTPort::read(char* buf, size_t len) {
	ssize_t rv = ::read(m_fd, buf, len);
	if (rv < 0 && errno != EAGAIN && errno != EINTR) {
		setSystemError((int)errno);
		return rv;
	}
	return (rv<0) ? 0:rv;
}


ssize_t UARTPort::write(char* buf, size_t len) {
	ssize_t rv = ::write(m_fd, buf, len);
	if (rv < 0 && errno != EAGAIN && errno != EINTR) {
		setSystemError((int)errno);
		return rv;
	}
	return (rv<0) ? 0:rv;
}

bool UARTPort::setBlocking(bool blocking) {
    struct termios tty;
     memset (&tty, 0, sizeof tty);
     if (tcgetattr (m_fd, &tty) != 0)
     {
    	 return setSystemError((int)errno);
     }

     tty.c_cc[VMIN]  = blocking ? 1 : 0;
     tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

     if (tcsetattr (m_fd, TCSANOW, &tty) != 0)
    	 return setSystemError((int)errno);
     return true;
}

void UARTPort::flush() {
	tcdrain(m_fd);
}

#if 0
int UARTPort::inWaiting() {
	int n = 0;
	int rv = ::ioctl(m_fd, TIOCINQ, &n);
	if (rv < 0) {
		setSystemError((int)errno);
		return -1;
	}
	return n;
}

int UARTPort::outWaiting() {
	int n = 0;
	int rv = ::ioctl(m_fd, TIOCOUTQ, &n);
	if (rv < 0) {
		setSystemError((int)errno);
		return -1;
	}
	return n;
}
#endif
