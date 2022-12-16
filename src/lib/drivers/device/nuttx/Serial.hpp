/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

namespace device __EXPORT
{

class Serial
{
public:
	Serial(const std::string &port = "",
	       uint32_t baudrate = 57600,
	       Timeout timeout = Timeout(),
	       bytesize_t bytesize = eightbits,
	       parity_t parity = parity_none,
	       stopbits_t stopbits = stopbits_one,
	       flowcontrol_t flowcontrol = flowcontrol_none)
	{

	}

	virtual ~Serial();


	enum class BuadRate {
		//B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800

	};

	bool open();
	bool isOpen() const;

	bool close();
	bool isClosed() const;

	size_t read(uint8_t *buffer, size_t size);

	// TODO:
	//readAtLeast()?

	size_t write(const uint8_t *data, size_t size);

	void setPort(const char *port);
	const char *getPort() const;

	bool setBaudrate(uint32_t baudrate);
	uint32_t getBaudrate() const;

	bool setTimeout(int timeous_ms = 0);

	void flush();


	// printStatus()
	//  port, read bytes, write bytes

protected:

	char _port[32] {};

};


}



// NOTES:
//  - blocking read
//  - timeout
//  - num chars?

#include <unistd.h>

// ssize_t read(int fd, void *buf, size_t count);



class device::Serial::SerialImpl
{
public:
	SerialImpl();
	virtual SerialImpl();

	bool open()
	{
		// if already open first close
		if (isOpen()) {
			close();
		}

		_serial_fd = ::open(_port, O_RDWR | O_NOCTTY); // TODO: O_NDELAY

		// errno:
		//  EACCES
		//  EBUSY
		//

		if (_serial_fd < 0) {
			PX4_ERR("failed to open %s err: %d", _port, errno);
			return false;
		}

		// fcntl(fd, F_SETFL, FNDELAY);

		return true;
	}

	bool close()
	{
		if (_serial_fd >= 0) {
			::close(_serial_fd);
			_serial_fd = -1;
		}

		return true;
	}

	size_t read(uint8_t *buffer, size_t size)
	{

		// TODO:
		//  - lazy port open if necessary?
		if (!isOpen()) {
			open();
		}


		// POLLIN: There is data to read.

		pollfd fds[1];
		fds[0].fd = _serial_fd;
		fds[0].events = POLLIN;

		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

		// int bytes_available = 0;
		// err = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

		// if (err != 0 || bytes_available < (int)character_count) {
		// 	px4_usleep(sleeptime);
		// }

		ret = ::read(_serial_fd, buf, buf_length);

		if (ret > 0) {
			_num_bytes_read += ret;
		}
	}

	size_t write(const uint8_t *data, size_t size)
	{
		if (!isOpen()) {
			open();
		}

		// POLLOUT: Writing is now possible

		// FIONWRITE: Get the number of bytes that have been written to the TX buffer.
		// FIONSPACE: Get the number of free bytes in the TX buffer

		// TCFLSH: Empty the tx/rx buffers


		//size_t written = ::write(_serial_fd, data, len);
		// 	::fsync(_serial_fd);



	}

	void setPort(const char *port);
	const char *getPort() const;

	uint32_t getBaudrate() const { return _baudrate; }
	bool setBaudrate(uint32_t baudrate)
	{
		/* process baud rate */
		int speed;

		switch (baud) {
		case 9600:   speed = B9600;   break;

		case 19200:  speed = B19200;  break;

		case 38400:  speed = B38400;  break;

		case 57600:  speed = B57600;  break;

		case 115200: speed = B115200; break;

		case 230400: speed = B230400; break;

#ifndef B460800
#define B460800 460800
#endif

		case 460800: speed = B460800; break;

#ifndef B921600
#define B921600 921600
#endif

		case 921600: speed = B921600; break;

		default:
			PX4_ERR("ERR: unknown baudrate: %d", baud);
			return -EINVAL;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_serial_fd, &uart_config);

		/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

		//
		// Input flags - Turn off input processing
		//
		// convert break to null byte, no CR to NL translation,
		// no NL to CR translation, don't mark parity errors or breaks
		// no input parity check, don't strip high bit off,
		// no XON/XOFF software flow control
		//
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
					 INLCR | PARMRK | INPCK | ISTRIP | IXON);
		//
		// Output flags - Turn off output processing
		//
		// no CR to NL translation, no NL to CR-NL translation,
		// no NL to CR translation, no column 0 CR suppression,
		// no Ctrl-D suppression, no fill characters, no case mapping,
		// no local output processing
		//
		// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
		//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
		uart_config.c_oflag = 0;

		//
		// No line processing
		//
		// echo off, echo newline off, canonical mode off,
		// extended input processing off, signal chars off
		//
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

		/* no parity, one stop bit, disable flow control */
		uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
			return -1;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
			return -1;
		}

		if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("ERR: %d (tcsetattr)", termios_state);
			return -1;
		}

		return 0;
	}



private:

	int _serial_fd{-1};

	uint32_t _baudrate{0};


	size_t _bytes_read{0};
	size_t _bytes_written{0};
};
