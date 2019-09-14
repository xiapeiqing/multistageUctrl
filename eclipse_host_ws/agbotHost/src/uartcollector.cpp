#include "uartcollector.h"
#include "uartport.h"
#include "udpport.h"
#include "tcpport.h"
#include <sys/time.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cassert>
#include <iostream>
#include "ringbuffer.h"
#include "iodevice.h"
#include <vector>

#define UART_BUFFER_SIZE	1024

using namespace std;

UARTCollector::UARTCollector(int bufferSize) {
	m_bufferSize = bufferSize;
	m_errno = 0;
}

UARTCollector::~UARTCollector() {
	for (size_t i=0; i<m_ports.size(); i++) {
		delete m_ports[i];
		delete m_readBuffers[i];
		delete m_writeBuffers[i];
	}
}

int UARTCollector::addPort(const std::string& name) {
	auto iditer = m_mapPortName2Id.find(name);
	if (iditer != m_mapPortName2Id.end()) {
		cerr << "iodev " << name << " already registered" << endl;
		return iditer->second;
	}

	std::string iotype = IODevice::iotype(name);
	IODevice* iodev = nullptr;
	if (iotype == "Serial") {
		iodev = new UARTPort(name);
	} else if (iotype == "TCP") {
		iodev = new TCPPort(name);
	} else if (iotype == "UDP") {
		iodev = new UDPPort(name);
	} else {
		setErrorInfo(-1, "iotype not supported");
		return -1;
	}

	if (!iodev->open()) {
		setErrorInfo(iodev->error(), iodev->errstring());
		delete iodev;
		return -1;
	}

	RingBuffer* rr = new RingBuffer(m_bufferSize);
	assert(rr);
	RingBuffer* wr = new RingBuffer(m_bufferSize);
	assert(wr);
	m_ports.push_back(iodev);
	m_readBuffers.push_back(rr);
	m_writeBuffers.push_back(wr);
	size_t id = m_ports.size() - 1;
	m_mapPortName2Id[name] = id;
	return id;
}

IODevice* UARTCollector::port(int portId) {
	int np = (int)m_ports.size();
	return (portId<0 || portId>=np) ? nullptr:m_ports[portId];

}

int UARTCollector::portId(const std::string& name) {
	auto it = m_mapPortName2Id.find(name);
	if (it != m_mapPortName2Id.end())
		return it->second;
	return -1;
}

ssize_t UARTCollector::readPort(int portId, char* buf, size_t len) {
	return m_readBuffers[portId]->read(buf, len);
}

ssize_t UARTCollector::writePort(int portId, char* buf, size_t len) {
	return m_writeBuffers[portId]->write(buf, len);
}

int UARTCollector::poll(int timeout_ms, UARTCollector::PollCallback func, void* ctx) {
	fd_set rfds, wfds;
	int rv;
	struct timeval tv;
	int maxfd = 0;

	FD_ZERO(&rfds);
	FD_ZERO(&wfds);
	for (size_t i = 0; i < m_ports.size(); i++) {
		IODevice* p = m_ports[i];
		if (p->error()) {
			continue;
		}
		int fd = p->fileno();
		bool polled = false;
		if (p->pollRead(true)) {
			FD_SET(fd, &rfds);
			polled = true;
		}
		if (p->pollWrite(m_writeBuffers[i]->used())) { // Sending buffer not empty
			FD_SET(fd, &wfds);
			polled = true;
		}
		if (polled && fd > maxfd) {
			maxfd = fd;
		}
	}
	if (maxfd == 0) {
		setErrorInfo(-1, "No port to poll");
		return -1;
	}
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms - tv.tv_sec*1000)*1000;
	rv = select(maxfd+1, &rfds, &wfds, nullptr, &tv);
	if (rv < 0) { // Error
		setErrorInfo((int)errno, strerror(errno));
		return rv;
	}
	if (rv == 0) { // Not ready
		return rv;
	}
	// Jobs piled up
	char* buf = new char[UART_BUFFER_SIZE];
	assert(buf);
	for (size_t i=0; i<m_ports.size(); i++) {
		if (m_ports[i]->error())
			continue;
		IODevice* p = m_ports[i];
		int fd = p->fileno();
		if (FD_ISSET(fd, &rfds)) { // Reading ready event
			ssize_t rv2 = p->read(buf, UART_BUFFER_SIZE);
			while (rv2 > 0) {
				m_readBuffers[i]->write(buf, rv2);
				if (func) {
					func(ctx, i, buf, rv2);
				}
				rv2 = p->read(buf, UART_BUFFER_SIZE);
			}
			// Error or no more data goes here
			if (func) {
				func(ctx, i, buf, rv2);
			}
			if (rv2<0) {
				assert(p->error());
				std::cerr << "Error read port " << p->name() << " => " << p->errstring() << std::endl;
				continue; // write detection ignored
			}
		}
		if (FD_ISSET(fd, &wfds)) { // Writing ready event
			assert(p->pollWrite(m_writeBuffers[i]->used()));
			if (m_writeBuffers[i]->used()) {
				ssize_t rv2 = m_writeBuffers[i]->read(buf, UART_BUFFER_SIZE);
				ssize_t rv3 = 0;
				while (rv2 > 0) {
					rv3 = p->write(buf, rv2);
					if (rv3 < 0) { // UART error
						assert(p->error());
						std::cerr << "Error write port " << p->name() << " => " << p->errstring() << std::endl;
						break;
					}
					if (rv3 < rv2) {// UART cannot send anymore
						std::cerr << "Not all bytes written, put back " << (rv2-rv3) << " bytes" << std::endl;
						m_writeBuffers[i]->write(buf+rv3, rv2-rv3); // Put bytes back
						break;
					}
					//std::cerr << "Port " << p->devName() << " sent " << rv3 << " bytes" << std::endl;
					rv2 = m_writeBuffers[i]->read(buf, sizeof(buf));
				}
			} else { // Polling is required by device
				ssize_t rv2 = p->write(nullptr, 0);
				if (rv2 < 0) {
					assert(p->error());
					std::cerr << "Error write port " << p->name() << " => " << p->errstring() << std::endl;
				}
			}
		}
	}
	delete[] buf;
	return rv;
}

size_t UARTCollector::readBufferUsed(int portId) {
	return m_readBuffers[portId]->used();
}

size_t UARTCollector::readBufferAvail(int portId) {
	return m_readBuffers[portId]->avail();

}

size_t UARTCollector::writeBufferUsed(int portId) {
	return m_writeBuffers[portId]->used();
}

size_t UARTCollector::writeBufferAvail(int portId) {
	return m_writeBuffers[portId]->avail();

}

void UARTCollector::setErrorInfo(int en, const std::string& errstring) {
	m_errno = en;
	m_errstring = errstring;
}
