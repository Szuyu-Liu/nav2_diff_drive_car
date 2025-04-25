cmake_minimum_required(VERSION 3.8)
project(imu_bno055)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
include(FindPkgConfig)

#Add the include directory
include_directories(include)

#Locate the i2c library
find_library(I2C_LIB NAMES i2c)

#Add Executable
add_executable(imu_node 
  #src/BNO055.c 
  #src/imu_node.cpp
  src/imu_euler.cpp
  )

ament_target_dependencies(imu_node rclcpp sensor_msgs std_msgs rcutils)

#Link the i2c library
target_link_libraries(imu_node ${I2C_LIB})

target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
