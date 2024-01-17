cmake_minimum_required(VERSION 3.8)
project(copley_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wpedantic)
endif()

include_directories(src/inc src/inc/ecat)

file(GLOB SOURCES "src/c/*.cpp")
add_library(CMLLib ${SOURCES} src/c/ecat/ecat_linux.cpp src/c/threads/Threads_posix.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(copley_driver src/copley_driver.cpp)
target_link_libraries(copley_driver CMLLib Threads::Threads)
target_include_directories(copley_driver PUBLIC
  include/copley_ros_driver
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(copley_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(copley_driver
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS copley_driver DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
