
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v4
	LABEL rtps
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	TESTING
	UAVCAN_INTERFACES 1

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2

	DRIVERS
		barometer # all available barometer drivers
		batt_smbus
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		heater
		imu # all available imu drivers
		irlock
		lights/blinkm
		lights/oreoled
		lights/rgbled
		lights/rgbled_ncp5623c
		magnetometer # all available magnetometer drivers
		mkblctrl
		pca9685
		protocol_splitter
		pwm_input
		pwm_out_sim
		px4flow
		px4fmu
		rc_input
		stm32
		stm32/adc
		stm32/tone_alarm
		tap_esc
		telemetry # all available telemetry drivers
		test_ppm
		tone_alarm
		uavcan

	MODULES
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		gnd_att_control
		gnd_pos_control
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		micrortps_bridge
		navigator
		position_estimator_inav
		sensors
		vmount
		vtol_att_control
		wind_estimator

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		esc_calib
		hardfault_log
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver

	EXAMPLES
		bottle_drop # OBC challenge
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hello
		hwtest # Hardware test
		#matlab_csv_serial
		#publisher
		px4_mavlink_debug # Tutorial code from https://px4.io/dev/debug_values
		px4_simple_app # Tutorial code from https://px4.io/dev/px4_simple_app
		rover_steering_control # Rover example app
		segway
		#subscriber
		uuv_example_app

	)
