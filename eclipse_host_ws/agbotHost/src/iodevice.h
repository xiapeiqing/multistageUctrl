/*
 * iodevice.h
 *
 *  Created on: 2017年3月7日
 *      Author: leo
 */

#ifndef IODEVICE_H_
#define IODEVICE_H_

#include <string>
#include <vector>

class IODevice {
public:
	IODevice(const std::string& name);
	virtual ~IODevice() {}

	/**
	 * To poll read events, the first consideration is device is willing
	 * or anxious to be polled, the second one is polling is needed
	 * by caller.
	 */
	virtual bool pollRead(bool needed) { return needed; }
	virtual bool pollWrite(bool needed) { return needed; }
	virtual bool open() = 0;
	virtual int fileno() = 0;
	virtual ssize_t read(char* buf, size_t len) = 0;
	virtual ssize_t write(char* buf, size_t len) = 0;

	std::string name() const { return m_name; }
	int error() const { return m_errno; }
	std::string errstring() const { return m_errstring; }
	bool setSystemError(int eno);
	bool setError(int eno, const std::string& errstring);

	static std::string iotype(const std::string& name, char delim=':');
	static std::vector<std::string> ioargs(const std::string& name, char delim=':');

protected:
	std::string m_name;
	int m_errno;
	std::string m_errstring;
};

#endif /* IODEVICE_H_ */
