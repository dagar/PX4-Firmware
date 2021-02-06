

# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
)

set(uavcanblid_hw_version_major 1)
set(uavcanblid_hw_version_minor 0)
set(uavcanblid_name "\"org.ardupilot.mro_m10025\"")

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

px4_add_board(
	PLATFORM nuttx
	VENDOR mro
	MODEL cannode
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_FLASH
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	UAVCAN_TIMER_OVERRIDE 3
	DRIVERS
		barometer/dps310
		bootloaders
		#gps
		lights/rgbled_ncp5623c
		magnetometer/rm3100
		uavcannode
	MODULES
	SYSTEMCMDS
		param
)
