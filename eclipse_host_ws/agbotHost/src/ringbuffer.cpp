#include "ringbuffer.h"
#include <cassert>
#include <cstdlib>
#include <cstring>

RingBuffer::RingBuffer(size_t size) {
	m_size = size;
	m_buf = new char[m_size];
	assert(m_buf);
	m_ridx = m_widx = 0;
}


RingBuffer::~RingBuffer() {
	if (m_buf) {
		delete[] m_buf;
	}
}


size_t RingBuffer::read(char* buf, size_t len) {
	size_t todo = used();
	if (todo > len) {
		todo = len;
	}
	size_t end = m_ridx + todo;
	if (end > m_size) { // read around
		size_t remain = m_size - m_ridx;
		memcpy(buf, m_buf+m_ridx, remain);
		buf += remain;
		end = todo - remain;
		m_ridx = 0;
	}
	memcpy(buf, m_buf+m_ridx, end-m_ridx);
	m_ridx = (end % m_size);
	return todo;
}


size_t RingBuffer::write(const char* buf, size_t len) {
	size_t todo = len;
	if (len >= m_size) {
		todo = m_size -  1;
		buf += len - todo;
	}
	bool exceed = avail() < todo;
	size_t end = m_widx + todo;
	if (end > m_size) {
		int remain = m_size - m_widx;
		memcpy(m_buf+m_widx, buf, remain);
		buf += remain;
		end = todo - remain;
		m_widx = 0;
	}
	memcpy(m_buf+m_widx, buf, end-m_widx);
	m_widx = end % m_size;
	if (exceed) { // write around
		m_ridx = (m_widx+1) % m_size;
	}
	return todo;
}

#ifdef DEBUG_RINGBUFFER

#include <iostream>
using namespace std;

#define UNUSED(x) do { (void)(x); } while (0)

int main(int argc, const char* argv[]) {
	UNUSED(argc);
	UNUSED(argv);
	RingBuffer rb(4);
	char buf[16];
	int r;

	assert(rb.used()==0);
	r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"")==0);

	rb.write("a",1);assert(rb.used()==1);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"a")==0);
	rb.write("ab",2);assert(rb.used()==2);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"ab")==0);
	rb.write("abc",3);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"abc")==0);
	rb.write("abcd",4);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"bcd")==0);
	rb.write("abcde",5);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"cde")==0);
	rb.write("abcdef",6);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"def")==0);
	rb.write("abcdefg",7);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"efg")==0);
	rb.write("abcdefgh",8);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"fgh")==0);
	rb.write("abcdefghi",9);assert(rb.used()==3);r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"ghi")==0);
	r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"")==0);

	rb.write("abcd",4);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"b")==0);assert(rb.used()==2);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"c")==0);assert(rb.used()==1);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"d")==0);assert(rb.used()==0);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"")==0);

	rb.write("a",1);
	r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"a")==0);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"")==0);

	rb.write("a",1);
	rb.write("b",1);
	r = rb.read(buf,sizeof(buf));buf[r]='\0';assert(strcmp(buf,"ab")==0);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"")==0);
	r = rb.read(buf,1);buf[r]='\0';assert(strcmp(buf,"")==0);

	cout << "OK" << endl;
	return 0;
}
#endif
