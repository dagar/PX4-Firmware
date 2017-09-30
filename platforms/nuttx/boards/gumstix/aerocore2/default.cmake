
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/led
	drivers/px4fmu
	drivers/lsm303d
	drivers/l3gd20
	drivers/ms5611
	drivers/teraranger
	drivers/gps
	drivers/pwm_out_sim
	drivers/ets_airspeed
	drivers/ms4525_airspeed
	drivers/ms5525_airspeed
	drivers/sdp3x_airspeed
	#drivers/frsky_telemetry
	modules/sensors
	#drivers/pwm_input
	#drivers/camera_trigger
	drivers/bst

	#
	# System commands
	#
	#systemcmds/dumpfile
	#systemcmds/esc_calib
	systemcmds/mixer
	#systemcmds/motor_ramp
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	#systemcmds/sd_bench
	#systemcmds/topic_listener
	systemcmds/ver

	#
	# Testing
	#
	#drivers/sf0x/sf0x_tests
	#drivers/test_ppm
	#lib/rc/rc_tests
	#modules/commander/commander_tests
	#lib/controllib/controllib_test
	#modules/mavlink/mavlink_tests
	#modules/unit_test
	#modules/uORB/uORB_tests
	#systemcmds/tests

	#
	# General system control
	#
	modules/commander
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/uavcan
	modules/land_detector

	#
	# Estimation modules
	#
	#modules/attitude_estimator_q
	#modules/position_estimator_inav
	#modules/local_position_estimator
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/gnd_att_control
	modules/gnd_pos_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/logger
	#modules/sdlog2

	#
	# Library modules
	#
	lib/param
	lib/systemlib
	lib/mixer
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/rc
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/led
	lib/version



	#
	# OBC challenge
	#
	#modules/bottle_drop

	#
	# Rover apps
	#
	#examples/rover_steering_control

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	#examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	#examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)
