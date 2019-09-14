/*
 * ringbuffer.h
 *
 *  Created on: 2017年2月14日
 *      Author: leo
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <cstdlib>
#include <cstddef>

/**
 * A simple ring buffer implementation
 */
class RingBuffer {
public:
	RingBuffer(size_t size);
	virtual ~RingBuffer();

	size_t read(char* buf, size_t len);
	size_t write(const char* buf, size_t len);
	size_t size() { return m_size; }
	size_t used() { return (m_ridx==m_widx) ? 0:((m_widx+m_size-m_ridx)%m_size); }
	size_t avail() { return m_size - used() - 1; }

private:
	char* m_buf;
	size_t m_size;
	size_t m_ridx;
	size_t m_widx;
};


#endif /* RINGBUFFER_H_ */
