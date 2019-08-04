/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_sem.h>

class ReaderWriterLock
{
public:
	explicit ReaderWriterLock()
	{
		px4_sem_init(&_reader_lock_holders_sem, 0, 1);
		px4_sem_init(&_writer_sem, 0, 1);
	}

	~ReaderWriterLock()
	{
		px4_sem_destroy(_reader_lock_holders_sem);
		px4_sem_destroy(_writer_sem);
	}

	void ReadLock()
	{
		do {} while (px4_sem_wait(&_reader_lock_holders_sem) != 0);

		++_reader_lock_holders;

		if (_reader_lock_holders == 1) {
			// the first reader takes the lock, the next ones are allowed to just continue
			do {} while (px4_sem_wait(&_writer_sem) != 0);
		}

		px4_sem_post(&_reader_lock_holders_sem);
	}

	void ReadUnlock()
	{
		do {} while (px4_sem_wait(&_reader_lock_holders_sem) != 0);

		--_reader_lock_holders;

		if (_reader_lock_holders == 0) {
			// the last reader releases the lock
			px4_sem_post(&_writer_sem);
		}

		px4_sem_post(&_reader_lock_holders_sem);
	}

	void WriteLock()
	{
		do {} while (px4_sem_wait(&_writer_sem) != 0);
	}

	void WriteUnlock()
	{
		px4_sem_post(&_writer_sem);
	}

private:

	px4_sem_t	_reader_lock_holders_sem;
	px4_sem_t	_writer_sem;

	int		_reader_lock_holders{0};
};
