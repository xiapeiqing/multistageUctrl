/*
 * uartport.h
 *
 *  Created on: 2017年2月14日
 *      Author: leo
 */

#ifndef UARTPORT_H_
#define UARTPORT_H_

#include <string>
#include "iodevice.h"

// Ref. http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
// Ref. pyserial source

class UARTPort: public IODevice {
public:
	enum ParityType {PARITY_NONE, PARITY_ODD, PARITY_EVEN,
		PARITY_MARK, PARITY_SPACE};
	UARTPort(const std::string& name);
	virtual ~UARTPort();

	bool open() override;
	int fileno() override { return m_fd; }
	ssize_t read(char* buf, size_t len) override;
	ssize_t write(char* buf, size_t len) override;

	void close();
	void flush();
	//int inWaiting();
	//int outWaiting();
	bool setBlocking(bool blocking);

private:
	std::string m_devName;
	int m_rate;
	ParityType m_parity;
	int m_fd;
};

#endif /* UARTPORT_H_ */
