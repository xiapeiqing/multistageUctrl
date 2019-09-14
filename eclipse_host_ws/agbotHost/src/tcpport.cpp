#include "tcpport.h"
#include <sys/types.h>  /* for Socket data types */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <unistd.h>     /* for close() */
#include <netinet/in.h> /* for IP Socket data types */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <fcntl.h>
#include <cstring>     /* for memset() */
#include <cassert>
#include <cerrno>
#include <iostream>


using namespace std;

// TCP:192.168.0.1:1977
TCPPort::TCPPort(const string& name):IODevice(name) {
	vector<string> args = ioargs(name);
	assert(args.size()==3);
	assert(args[0]=="TCP");
	m_address = args[1];
	m_port = std::stoi(args[2]);
	m_fd = -1;
	m_state = Closed;
}

TCPPort::~TCPPort() {
	this->close();
}

bool TCPPort::open() {
	struct sockaddr_in srvaddr;
	int rc;

	assert(m_state==Closed);
	m_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_fd < 0) {
		return setSystemError(errno);
	}
	int flags = fcntl(m_fd, F_GETFL, 0);
	if (fcntl(m_fd, F_SETFL, flags | O_NONBLOCK)<0) {
		goto cleanup;
	}

	memset(&srvaddr, 0, sizeof(srvaddr));
	srvaddr.sin_family = AF_INET;
	srvaddr.sin_addr.s_addr = inet_addr(m_address.c_str());
	srvaddr.sin_port = htons(m_port);
	rc = connect(m_fd, (struct sockaddr *)&srvaddr, sizeof(srvaddr));
	if (rc < 0) {
		int eno = errno;
		if (eno!=EINPROGRESS) {
			goto cleanup;
		}
		m_state = Connecting;
	} else {
		m_state = Ready;
	}
	return true;

cleanup:
	setSystemError(errno);
	::close(m_fd);
	m_fd = -1;
	m_state= Error;
	return false;
}

void TCPPort::close() {
	if (m_fd >= 0) {
		::close(m_fd);
		m_fd = -1;
		m_state = Closed;
	}
}

// Needed when connect, use "man 2 connect" to see details
bool TCPPort::pollWrite(bool needed) {
	return ((m_state==Connecting) || ((m_state==Ready) && needed));
}

bool TCPPort::pollRead(bool needed) {
	return ((m_state==Ready) && needed);
}

ssize_t TCPPort::read(char* buf, size_t len) {
	assert(m_state==Ready);
	int rc = ::read(m_fd, buf, len);
	if (rc < 0) {
		if (errno != EAGAIN && errno != EINTR) {
			setSystemError(errno);
			m_state = Error;
			return rc;
		} else { // Try again
			return 0;
		}

	} else if (rc == 0) { // Peer closed
		this->close();
		this->setError(-1, "Peer connection closed");
		return -1;
	}

	return (rc < 0) ? 0:rc;
}

ssize_t TCPPort::write(char* buf, size_t len) {
	int rc;
	if (m_state==Connecting) {
		int error = 0;
		socklen_t slen = sizeof(error);
		rc = getsockopt(m_fd, SOL_SOCKET, SO_ERROR, &error, &slen);
		if (rc < 0) {
			setSystemError(errno);
			m_state = Error;
			return -1;
		}
		if (error != 0) {
			setSystemError(error);
			m_state = Error;
			return -1;
		}
		m_state = Ready;
		return 0;
	}
	assert(m_state==Ready);
	rc = ::write(m_fd, buf, len);
	if (rc < 0  && errno != EAGAIN && errno != EINTR) {
		setSystemError(errno);
		m_state = Error;
		return rc;
	}
	return (rc < 0) ? 0:rc;
}

