cmake_minimum_required(VERSION 3.8)
project(motor_controller_ol)

# Enable compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# Create executable target
add_executable(motor_controller src/motor_controller.cpp)

# Include directories for motor_controller executable
target_include_directories(motor_controller PRIVATE
  ${SERIAL_INCLUDE_DIRS}  # LibSerial include directories
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Local include directories (if any)
)

# Link libraries for motor_controller executable
ament_target_dependencies(motor_controller rclcpp geometry_msgs)

# Link LibSerial library to motor_controller
target_link_libraries(motor_controller
  ${SERIAL_LIBRARIES}  # LibSerial libraries
)

# Export include directories for other targets that depend on this package
ament_export_include_directories(
  include
)

# Install targets
install(TARGETS motor_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# ament package export
ament_package()
