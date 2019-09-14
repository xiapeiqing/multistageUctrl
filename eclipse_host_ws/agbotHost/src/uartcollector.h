/*
 * uartcollector.h
 *
 *  Created on: 2017年2月14日
 *      Author: leo
 */

#ifndef UARTCOLLECTOR_H_
#define UARTCOLLECTOR_H_

#include <string>
#include <vector>
#include <map>
#include <unistd.h>
#include "ringbuffer.h"
#include "iodevice.h"

class UARTCollector {
public:
	typedef void (*PollCallback)(void* ctx, int portId, char* buf, ssize_t len);

	UARTCollector(int bufferSize=4096);
	virtual ~UARTCollector();

	int addPort(const std::string& name);
	ssize_t numberOfPorts() { return m_ports.size(); }
	IODevice* port(int portId);
	int portId(const std::string& name);
	ssize_t readPort(int portId, char* buf, size_t len);
	ssize_t writePort(int portId, char* buf, size_t len);
	size_t readBufferUsed(int portId);
	size_t readBufferAvail(int portId);
	size_t writeBufferUsed(int portId);
	size_t writeBufferAvail(int portId);
	int poll(int timeout_ms, PollCallback func=nullptr, void* ctx=nullptr);
	int error() { return m_errno; }
	std::string errstring() { return m_errstring; }

private:
	void setErrorInfo(int en, const std::string& errstring);
	std::vector<IODevice*> m_ports;
	std::vector<RingBuffer*> m_readBuffers;
	std::vector<RingBuffer*> m_writeBuffers;
	std::map<std::string,int> m_mapPortName2Id;
	int m_bufferSize;
	int m_errno;
	std::string m_errstring;
};

#endif /* UARTCOLLECTOR_H_ */
