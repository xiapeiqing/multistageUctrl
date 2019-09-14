#include "udpport.h"
#include <sys/types.h>  /* for Socket data types */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <unistd.h>     /* for close() */
#include <netinet/in.h> /* for IP Socket data types */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <fcntl.h>
#include <cstring>     /* for memset() */
#include <cassert>
#include <cerrno>

// Ref. http://cs.baylor.edu/~donahoo/practical/CSockets/textcode.html

using namespace std;

// UDP:192.168.0.1:1977
UDPPort::UDPPort(const string& name):IODevice(name) {
	vector<string> args = ioargs(name);
	assert(args.size()==3);
	assert(args[0]=="UDP");
	m_address = args[1];
	m_port = std::stoi(args[2]);
	m_fd = -1;
}

UDPPort::~UDPPort() {
	this->close();
}

bool UDPPort::open() {
	m_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_fd < 0) {
		return setSystemError(errno);
	}
	int flags = fcntl(m_fd, F_GETFL, 0);
	if (fcntl(m_fd, F_SETFL, flags | O_NONBLOCK)<0) {
		goto cleanup;
	}

	return true;

cleanup:
	setSystemError(errno);
	::close(m_fd);
	m_fd = -1;
	return false;
}

void UDPPort::close() {
	if (m_fd >= 0) {
		::close(m_fd);
		m_fd = -1;
	}
}

ssize_t UDPPort::read(char* buf, size_t len) {
	struct sockaddr_in fromaddr;
	socklen_t fromlen;
	fromlen = sizeof(fromaddr);
	memset(&fromaddr, 0, sizeof(fromaddr));
	int rc = ::recvfrom(m_fd, buf, len, 0, (struct sockaddr*)&fromaddr, &fromlen);
	if (rc < 0  && errno != EAGAIN && errno != EINTR) {
		setSystemError(errno);
		return rc;
	}
	return (rc < 0) ? 0:rc;
}

ssize_t UDPPort::write(char* buf, size_t len) {
	struct sockaddr_in srvaddr;
	memset(&srvaddr, 0, sizeof(srvaddr));
	srvaddr.sin_family = AF_INET;
	srvaddr.sin_addr.s_addr = inet_addr(m_address.c_str());
	srvaddr.sin_port = htons(m_port);
	int rc = ::sendto(m_fd, buf, len, 0, (struct sockaddr *)&srvaddr,
			sizeof(srvaddr));
	if (rc < 0  && errno != EAGAIN && errno != EINTR) {
		setSystemError(errno);
		return rc;
	}
	return (rc < 0) ? 0:rc;
}
