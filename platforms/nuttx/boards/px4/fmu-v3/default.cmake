
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common IO px4io-v2)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	# Drivers
	drivers/hmc5883
	drivers/l3gd20
	drivers/lis3mdl
	drivers/lsm303d
	drivers/mpu6000
	drivers/mpu9250
	drivers/ms5611
	drivers/px4fmu
	drivers/px4io
	drivers/rgbled
	drivers/snapdragon_rc_pwm
)

# add common modules (see cmake/px4_module_presets.cmake)
PX4_ADD_ALL_MODULES()
