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

#include "send_event.h"

#include <math.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "level_calibration.h"
#include "mag_calibration.h"
#include "rc_calibration.h"

using namespace time_literals;

namespace events
{

// Run it at 30 Hz.
static constexpr uint32_t SEND_EVENT_INTERVAL_US{1_s / 30};

int SendEvent::task_spawn(int argc, char *argv[])
{
	SendEvent *send_event = new SendEvent();

	if (!send_event) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(send_event);
	_task_id = task_id_is_work_queue;

	send_event->start();

	return 0;
}

SendEvent::SendEvent() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	if (_param_ev_tsk_stat_dis.get()) {
		_status_display = new status::StatusDisplay();
	}

	if (_param_ev_tsk_rc_loss.get()) {
		_rc_loss_alarm = new rc_loss::RC_Loss_Alarm();
	}
}

SendEvent::~SendEvent()
{
	ScheduleClear();

	delete _status_display;
	delete _rc_loss_alarm;
}

int SendEvent::start()
{
	ScheduleOnInterval(SEND_EVENT_INTERVAL_US, 10000);

	return PX4_OK;
}

void SendEvent::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	process_commands();

	if (_status_display != nullptr) {
		_status_display->process();
	}

	if (_rc_loss_alarm != nullptr) {
		_rc_loss_alarm->process();
	}
}

void SendEvent::process_commands()
{
	// TODO: do something with vehicle commands
	// TODO: what is this modules purpose?

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* command ack */
	uORB::PublicationQueued<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};

	vehicle_command_s cmd {};

	/* if we reach here, we have a valid command */
	orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

	/* only handle low-priority commands here */
	switch (cmd.command) {
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

			if ((status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			    || status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, command_ack_pub);

			} else {

				int calib_ret = PX4_ERROR;

				/* try to go to INIT/PREFLIGHT arming state */
				if (TRANSITION_DENIED == arming_state_transition(&status, safety_s{}, vehicle_status_s::ARMING_STATE_INIT, &armed,
						false /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags,
						PreFlightCheck::arm_requirements_t{}, // arming requirements not relevant for switching to ARMING_STATE_INIT
						30_s, // time since boot not relevant for switching to ARMING_STATE_INIT
						(cmd.from_external ? arm_disarm_reason_t::COMMAND_EXTERNAL : arm_disarm_reason_t::COMMAND_INTERNAL))
				   ) {

					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
					break;

				} else {
					status_flags.condition_calibration_enabled = true;
				}

				if ((int)(cmd.param1) == 1) {
					/* gyro calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_gyro_calibration(&mavlink_log_pub);

				} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					/* temperature calibration: handled in events module */
					break;

				} else if ((int)(cmd.param2) == 1) {
					/* magnetometer calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_mag_calibration(&mavlink_log_pub);

				} else if ((int)(cmd.param3) == 1) {
					/* zero-altitude pressure calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);

				} else if ((int)(cmd.param4) == 1) {
					/* RC calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					/* disable RC control input completely */
					status_flags.rc_input_blocked = true;
					calib_ret = OK;
					mavlink_log_info(&mavlink_log_pub, "Calibration: Disabling RC input");

				} else if ((int)(cmd.param4) == 2) {
					/* RC trim calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_trim_calibration(&mavlink_log_pub);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_accel_calibration(&mavlink_log_pub);

				} else if ((int)(cmd.param5) == 2) {
					// board offset calibration
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_level_calibration(&mavlink_log_pub);

				} else if ((int)(cmd.param5) == 4) {
					// accelerometer quick calibration
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_accel_calibration_quick(&mavlink_log_pub);

				} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
					// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
					/* airspeed calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_airspeed_calibration(&mavlink_log_pub);

				} else if ((int)(cmd.param7) == 1) {
					/* do esc calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);

				} else if ((int)(cmd.param4) == 0) {
					/* RC calibration ended - have we been in one worth confirming? */
					if (status_flags.rc_input_blocked) {
						/* enable RC control input */
						status_flags.rc_input_blocked = false;
						mavlink_log_info(&mavlink_log_pub, "Calibration: Restoring RC input");
					}

					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					/* this always succeeds */
					calib_ret = OK;

				} else {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, command_ack_pub);
				}

				status_flags.condition_calibration_enabled = false;

				if (calib_ret == OK) {
					tune_positive(true);

					// time since boot not relevant here
					if (PreFlightCheck::preflightCheck(&mavlink_log_pub, status, status_flags, false, false, true, 30_s)) {

						status_flags.condition_system_sensors_initialized = true;
					}

					arming_state_transition(&status, safety_s{}, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
								false /* fRunPreArmChecks */,
								&mavlink_log_pub, &status_flags,
								PreFlightCheck::arm_requirements_t{}, // arming requirements not relevant for switching to ARMING_STATE_STANDBY
								30_s, // time since boot not relevant for switching to ARMING_STATE_STANDBY
								(cmd.from_external ? arm_disarm_reason_t::COMMAND_EXTERNAL : arm_disarm_reason_t::COMMAND_INTERNAL));

				} else {
					tune_negative(true);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW: {
			// Magnetometer quick calibration using world magnetic model and known heading
			if ((status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			    || (status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN)
			    || status_flags.condition_calibration_enabled) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, command_ack_pub);

			} else {
				// parameter 1: Yaw in degrees
				// parameter 3: Latitude
				// parameter 4: Longitude

				// assume vehicle pointing north (0 degrees) if heading isn't specified
				const float heading_radians = PX4_ISFINITE(cmd.param1) ? math::radians(roundf(cmd.param1)) : 0.f;

				float latitude = NAN;
				float longitude = NAN;

				if (PX4_ISFINITE(cmd.param3) && PX4_ISFINITE(cmd.param4)) {
					// invalid if both lat & lon are 0 (current mavlink spec)
					if ((fabsf(cmd.param3) > 0) && (fabsf(cmd.param4) > 0)) {
						latitude = cmd.param3;
						longitude = cmd.param4;
					}
				}

				if (do_mag_calibration_quick(&mavlink_log_pub, heading_radians, latitude, longitude) == PX4_OK) {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					tune_positive(true);

				} else {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
					tune_negative(true);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

			if ((status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			    || status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, command_ack_pub);

			} else {

				if (((int)(cmd.param1)) == 0) {
					int ret = param_load_default();

					if (ret == OK) {
						mavlink_log_info(&mavlink_log_pub, "Settings loaded");
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

					} else {
						mavlink_log_critical(&mavlink_log_pub, "Error loading settings");

						/* convenience as many parts of NuttX use negative errno */
						if (ret < 0) {
							ret = -ret;
						}

						if (ret < 1000) {
							mavlink_log_critical(&mavlink_log_pub, "Error: %s", strerror(ret));
						}

						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
					}

				} else if (((int)(cmd.param1)) == 1) {

					int ret = param_save_default();

					if (ret == OK) {
						/* do not spam MAVLink, but provide the answer / green led mechanism */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

					} else {
						mavlink_log_critical(&mavlink_log_pub, "Error saving settings");

						/* convenience as many parts of NuttX use negative errno */
						if (ret < 0) {
							ret = -ret;
						}

						if (ret < 1000) {
							mavlink_log_critical(&mavlink_log_pub, "Error: %s", strerror(ret));
						}

						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
					}

				} else if (((int)(cmd.param1)) == 2) {

					/* reset parameters and save empty file */
					param_reset_all();

					/* do not spam MAVLink, but provide the answer / green led mechanism */
					mavlink_log_critical(&mavlink_log_pub, "Onboard parameters reset");
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
		/* just ack, implementation handled in the IO driver */
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
		break;

	default:
		/* don't answer on unsupported commands, it will be done in main loop */
		break;
	}
}

void SendEvent::answer_command(const vehicle_command_s &cmd, unsigned result)
{
	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.result = (uint8_t)result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;

	uORB::PublicationQueued<vehicle_command_ack_s>	command_ack_pub{ORB_ID(vehicle_command_ack)};
	command_ack_pub.publish(command_ack);
}

int SendEvent::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("send_event", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int SendEvent::custom_command(int argc, char *argv[])
{
	// TODO: what is my purpose?
	print_usage("unrecognized command");

	return 0;
}

int send_event_main(int argc, char *argv[])
{
	return SendEvent::main(argc, argv);
}

} /* namespace events */
