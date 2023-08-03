

# Install Python modules
ament_python_install_package(px4_offboard)

# Install Python executables
install(PROGRAMS
	px4_offboard/offboard_control.py
	px4_offboard/visualizer.py

	DESTINATION lib/px4_offboard
)
