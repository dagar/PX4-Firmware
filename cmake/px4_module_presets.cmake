############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#=============================================================================
#
#	Defined macros in this file
#
# 	utility functions
#
#		* PX4_ADD_MODULES_CORE
#		* PX4_ADD_DRIVERS_AIRSPEED
#		* PX4_ADD_DRIVERS_DISTANCE
#		* PX4_ADD_DRIVERS_OPTIONAL
#		* PX4_ADD_TESTING
#		* PX4_ADD_EXAMPLES
#		* PX4_ADD_RTPS
#
#		* PX4_ADD_ALL_MODULES
#


macro(PX4_ADD_MODULES_CORE)
	# core px4 modules
	list(APPEND config_module_list
		# core drivers
		drivers/gps
		drivers/led
		drivers/pwm_out_sim

		# core libraries
		lib/airspeed
		lib/controllib
		lib/conversion
		lib/ecl
		lib/external_lgpl
		lib/geo
		lib/geo_lookup
		lib/hysteresis
		lib/ledcontroller
		lib/log
		lib/mathlib
		lib/mixer
		lib/param
		lib/perf
		lib/pid
		lib/pwm_limit
		lib/rc
		lib/systemlib
		lib/tinybson

		# core system modules
		modules/camera_feedback
		modules/camera_trigger
		modules/commander
		modules/dataman
		modules/ekf2
		modules/events
		modules/fw_att_control
		modules/fw_pos_control_l1
		modules/gnd_att_control
		modules/gnd_pos_control
		modules/gpio_led
		modules/land_detector
		modules/load_mon
		modules/logger
		modules/mavlink
		modules/mc_att_control
		modules/mc_pos_control
		modules/navigator
		modules/sensors
		modules/uORB
		modules/vmount
		modules/vtol_att_control

		# core system commands
		systemcmds/mixer
		systemcmds/param
		systemcmds/perf
		systemcmds/pwm
		systemcmds/ver
	)

	message(STATUS "Adding core system modules")
endmacro()

macro(PX4_ADD_EXTRA_MODULES)
	# extra modules and system commands
	list(APPEND config_module_list
		# Additional modules
		modules/attitude_estimator_q
		modules/local_position_estimator
		modules/position_estimator_inav
		modules/sdlog2
		modules/uavcan

		# Additional System commands
		systemcmds/dumpfile
		systemcmds/esc_calib
		systemcmds/led_control
		systemcmds/motor_ramp
		systemcmds/sd_bench
		systemcmds/topic_listener
	)

	message(STATUS "Adding extra modules and systemcmds")
endmacro()

macro(PX4_ADD_DRIVERS_AIRSPEED)
	# all airspeed drivers
	list(APPEND config_module_list
		drivers/ets_airspeed
		drivers/ms4525_airspeed
		drivers/ms5525_airspeed
		drivers/sdp3x_airspeed
	)

	message(STATUS "Adding airspeed sensors (ets, ms4525 ms5525, sdp3x)")
endmacro()

macro(PX4_ADD_DRIVERS_DISTANCE)
	# all airspeed drivers
	list(APPEND config_module_list
		drivers/ll40ls
		drivers/mb12xx
		drivers/pwm_input # needed for ll40ls
		drivers/sf0x
		drivers/sf1xx
		drivers/srf02
		drivers/teraranger
		drivers/ulanding
	)

	message(STATUS "Adding distance sensors (ll40ls, mb12xx, sf0x, sf1xx, srf02, teraranger, ulanding)")
endmacro()

macro(PX4_ADD_DRIVERS_OPTIONAL)
	# optional external drivers
	list(APPEND config_module_list
		drivers/blinkm
		drivers/bst
		drivers/frsky_telemetry
		drivers/hott
		drivers/iridiumsbd
		drivers/oreoled
		drivers/pca9685
		drivers/px4flow
		drivers/mkblctrl
		drivers/tap_esc
	)

	message(STATUS "Adding optional peripherals (blinkm, bst, frsky_telemetry, hott, iridiumsbd, oreoled, pca9685, px4flow, mkblctrl, tap_esc)")
endmacro()

macro(PX4_ADD_TESTING)
	# Testing modules
	list(APPEND config_module_list
		drivers/sf0x/sf0x_tests
		drivers/test_ppm
		lib/controllib/controllib_test
		modules/commander/commander_tests
		modules/mavlink/mavlink_tests
		modules/mc_pos_control/mc_pos_control_tests
		modules/uORB/uORB_tests
		systemcmds/tests
	)

	# RC test data only available on posix
	if (${OS} MATCHES "posix")
		list(APPEND config_module_list
			lib/rc/rc_tests
		)
	endif()

	message(STATUS "Adding testing modules")
endmacro()

macro(PX4_ADD_EXAMPLES)
	# example/demo apps
	list(APPEND config_module_list
		examples/bottle_drop # Outback Challenge bottle drop
		examples/fixedwing_control# Tutorial code from: https://px4.io/dev/example_fixedwing_control
		examples/hwtest # Example Hardware test
		examples/px4_daemon_app # Tutorial code from: https://px4.io/dev/daemon
		examples/px4_mavlink_debug # Tutorial code from: https://px4.io/dev/debug_values
		examples/px4_simple_app # Tutorial code from: https://px4.io/dev/px4_simple_app
		examples/rover_steering_control # Rover example
		examples/segway # Segway
	)

	message(STATUS "Adding example modules")
endmacro()

macro(PX4_ADD_RTPS)
	# FastRTPS and microCDR
	if (GENERATE_RTPS_BRIDGE)
		set(config_rtps_send_topics
			sensor_combined
		   )
	
		set(config_rtps_receive_topics
			sensor_baro
		   )
	
		message(STATUS "Adding FastRTPS modules")
	endif()
endmacro()

macro(PX4_ADD_ALL_MODULES)
	PX4_ADD_MODULES_CORE()
	PX4_ADD_EXTRA_MODULES()
	PX4_ADD_DRIVERS_AIRSPEED()
	PX4_ADD_DRIVERS_DISTANCE()
	PX4_ADD_DRIVERS_OPTIONAL()
	PX4_ADD_TESTING()
	PX4_ADD_EXAMPLES()
	PX4_ADD_RTPS()
endmacro()

# reduced core module set for boards with limited flash (eg px4fmu-v2)
macro(PX4_ADD_ALL_MODULES_SLIM)
	PX4_ADD_MODULES_CORE()
	PX4_ADD_DRIVERS_AIRSPEED()
	PX4_ADD_DRIVERS_DISTANCE()
endmacro()

