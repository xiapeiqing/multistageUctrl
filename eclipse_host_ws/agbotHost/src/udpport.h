#ifndef UDPPORT_H_
#define UDPPORT_H_

#include <string>
#include "iodevice.h"

class UDPPort: public IODevice {
public:
	UDPPort(const std::string& name);
	virtual ~UDPPort();

	bool open() override;
	int fileno() override { return m_fd; }
	ssize_t read(char* buf, size_t len) override;
	ssize_t write(char* buf, size_t len) override;

	void close();

private:
	std::string m_address;
	int m_port;
	int m_fd;
};

#endif /* UARTPORT_H_ */
