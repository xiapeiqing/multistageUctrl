/*
 * IODevice.cpp
 *
 *  Created on: 2017年3月7日
 *      Author: leo
 */

#include <cerrno>
#include <cstring>
#include "iodevice.h"
using namespace std;

string IODevice::iotype(const string& name, char delim) {
	string::size_type pos = name.find(delim);
	return (pos==string::npos) ? name:name.substr(0, pos);
}

vector<string> IODevice::ioargs(const string& name, char delim) {
	vector<string> result;
	string::size_type from = 0;
	string::size_type to = name.find(delim);
	while (to != string::npos) {
		result.push_back(name.substr(from, to-from));
		from = ++to;
		to = name.find(delim, to);
	}
	if (from < name.size()) { // a:b:c
		result.push_back(name.substr(from));
	}
	if (name[name.size()-1]==delim) { // a:b:c:
		result.push_back("");
	}
	return result;
}

IODevice::IODevice(const string& name): m_name(name) {
	m_errno = 0;
}

bool IODevice::setSystemError(int eno) {
	m_errno = eno;
	m_errstring = strerror(eno);
	return false;
}

bool IODevice::setError(int eno, const string& errstring) {
	m_errno = eno;
	m_errstring = errstring;
	return false;
}

