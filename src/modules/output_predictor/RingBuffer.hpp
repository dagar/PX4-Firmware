/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <inttypes.h>
#include <cstdio>
#include <cstring>

template <typename T, size_t SIZE>
class RingBuffer
{
public:
	void push(const T &sample)
	{
		uint8_t head_new = _head;

		if (!_first_write) {
			head_new = (_head + 1) % SIZE;
		}

		_buffer[head_new] = sample;
		_head = head_new;

		// move tail if we overwrite it
		if (_head == _tail && !_first_write) {
			_tail = (_tail + 1) % SIZE;

		} else {
			_first_write = false;
		}
	}

	uint8_t get_length() const { return SIZE; }

	T &operator[](const uint8_t index) { return _buffer[index]; }

	const T &get_newest() { return _buffer[_head]; }
	const T &get_oldest() { return _buffer[_tail]; }

	uint8_t get_oldest_index() const { return _tail; }

	bool pop_first_older_than(const uint64_t &timestamp, T &sample)
	{
		// start looking from newest observation data
		for (uint8_t i = 0; i < SIZE; i++) {
			int index = (_head - i);
			index = index < 0 ? SIZE + index : index;

			if (timestamp >= _buffer[index].time_us && timestamp < _buffer[index].time_us + (uint64_t)1e5) {
				sample = _buffer[index];

				// Now we can set the tail to the item which
				// comes after the one we removed since we don't
				// want to have any older data in the buffer
				if (index == _head) {
					_tail = _head;
					_first_write = true;

				} else {
					_tail = (index + 1) % SIZE;
				}

				_buffer[index].time_us = 0;

				return true;
			}

			if (index == _tail) {
				// we have reached the tail and haven't got a
				// match
				return false;
			}
		}

		return false;
	}

private:
	T _buffer[SIZE] {};

	uint8_t _head{0};
	uint8_t _tail{0};

	bool _first_write{true};
};
