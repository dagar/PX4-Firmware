
cmake_policy(SET CMP0057 NEW) # Support IN_LIST if() operator

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} ${px4_msg_files})
