/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Andrew Tridgell
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

/**
 * @file nshterm.c
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <termios.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <nshlib/nshlib.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION("Start an NSH shell on a given port.\n"
				 "\n"
				 "This was previously used to start a shell on the USB serial port.\n"
				 "Now there runs mavlink, and it is possible to use a shell over mavlink.\n"
				);

	PRINT_MODULE_USAGE_NAME_SIMPLE("nshterm", "command");
	PRINT_MODULE_USAGE_ARG("<file:dev>", "Device on which to start the shell (eg. /dev/ttyACM0)", false);
}

static int configure_port(int fd, int speed)
{
	// termiosp->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

	// termiosp->c_oflag &= ~OPOST;

	// termiosp->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	// termiosp->c_cflag &= ~(CSIZE | PARENB);
	// termiosp->c_cflag |= CS8;


	// cfgetispeed(termiosp)
	// cfgetospeed(termiosp)




	struct termios uart_config {};

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
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
	int termios_state;

	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		printf("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		printf("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		return -1;
	}

	return 0;
}

extern "C" __EXPORT int serial_passthrough_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	for (int i = 0; i < argc; i++) {
		printf("argv[%d] = %s\n", i, argv[i]);
	}

	/* set up the serial port with output processing */
	int fd_1 = open(argv[1], O_RDWR | O_NONBLOCK);
	int fd_2 = open(argv[2], O_RDWR | O_NONBLOCK);

	configure_port(fd_1, B115200);
	configure_port(fd_2, B115200);


	hrt_abstime last_stats_update = hrt_absolute_time();

	size_t total_1_bytes = 0;
	size_t total_2_bytes = 0;

	while (1) {


		// TODO: poll
		//   stats optional
		//   automatically exit on disconnect


		// read from 1, write to 2
		uint8_t rd_buffer_usb[32];
		int read_usb_bytes = ::read(fd_1, rd_buffer_usb, sizeof(rd_buffer_usb));

		if (read_usb_bytes > 0) {
			int ret = ::write(fd_2, rd_buffer_usb, read_usb_bytes);

			if (ret > 0) {
				total_1_bytes += ret;
			}
		}

		// read from 2, write to 1
		uint8_t rd_buffer_uart[32];
		int read_uart_bytes = ::read(fd_2, rd_buffer_uart, sizeof(rd_buffer_uart));

		if (read_uart_bytes > 0) {
			int ret = ::write(fd_1, rd_buffer_uart, read_uart_bytes);

			if (ret > 0) {
				total_2_bytes += ret;
			}
		}

		if (hrt_elapsed_time(&last_stats_update) > 1000000) {
			last_stats_update = hrt_absolute_time();
			printf("1->2 %d bytes/s, 2->1 %d bytes/s\n", total_1_bytes, total_2_bytes);
			total_1_bytes = 0;
			total_2_bytes = 0;
		}
	}



	close(fd_1);
	close(fd_2);

	PX4_INFO("exiting");

	return 0;
}
