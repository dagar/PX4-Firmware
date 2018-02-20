
px4_add_board(
	BOARD nuttx_px4io-v2_default
	OS nuttx
	ARCH cortex-m3
	)

set(config_module_list
	drivers/stm32
	modules/px4iofirmware
)
