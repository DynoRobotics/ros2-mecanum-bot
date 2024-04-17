cmake_minimum_required(VERSION 3.5)
project(mecanumbot_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  #add_link_options(-L /usr/local/lib/nanotec -lnanolib -ldl)
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_library(NANOTEC_LIBRARY libnanolib.so PATHS "/usr/local/lib/nanotec/")

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mecanumbot_hardware.cpp
  src/mecanumbot_serial_port.cpp
  src/nanolib_helper.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${NANOTEC_LIBRARY}
  dl
)

# target_link_libraries(
#   ${PROJECT_NAME}
#   hardware_interface
#   pluginlib
#   rclcpp

#   nanolib
#   dl
# )

pluginlib_export_plugin_description_file(hardware_interface mecanumbot_hardware.xml)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

# if(AMENT_CMAKE_SYMLINK_INSTALL)
#   install(CODE "execute_process(COMMAND sudo /usr/sbin/setcap 'cap_net_admin,cap_net_raw,cap_sys_nice+eip' \"${CMAKE_CURRENT_BINARY_DIR}/${NODE_NAME}\")")
# else()
#   install(CODE "execute_process(COMMAND sudo /usr/sbin/setcap 'cap_net_admin,cap_net_raw,cap_sys_nice+eip' \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${NODE_NAME}\")")
# endif()

ament_package()
