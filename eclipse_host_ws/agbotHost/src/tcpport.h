#ifndef TCPPORT_H_
#define TCPPORT_H_

#include <string>
#include "iodevice.h"

class TCPPort: public IODevice {
public:
	TCPPort(const std::string& name);
	virtual ~TCPPort();

	bool pollRead(bool needed) override;
	bool pollWrite(bool needed) override;
	bool open() override;
	int fileno() override { return m_fd; }
	ssize_t read(char* buf, size_t len) override;
	ssize_t write(char* buf, size_t len) override;

	void close();

private:
	enum TCPState { Connecting, Ready, Error, Closed };
	std::string m_address;
	int m_port;
	int m_fd;
	TCPState m_state;
};

#endif /* UARTPORT_H_ */
