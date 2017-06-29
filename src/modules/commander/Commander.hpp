/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#ifndef COMMANDER_HPP_
#define COMMANDER_HPP_

#include <controllib/blocks.hpp>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/Publication.hpp>

class Commander : public control::SuperBlock
{
public:
	Commander() :
		SuperBlock(nullptr, "COM"),
		_pub_control_mode(ORB_ID(vehicle_control_mode), -1, &getPublications()),
		_pub_status_flags(ORB_ID(vehicle_status_flags), -1, &getPublications())
	{

	}

	~Commander() = default;

	static void	task_main_trampoline(int argc, char *argv[]);
	int commander_thread_main(int argc, char *argv[]);

private:

	void publish_control_mode();
	void publish_status_flags();

	uORB::Publication<vehicle_control_mode_s> _pub_control_mode;
	uORB::Publication<vehicle_status_flags_s> _pub_status_flags;

};

namespace commander
{
extern Commander *g_control;
} // namespace commander

#endif /* COMMANDER_HPP_ */
