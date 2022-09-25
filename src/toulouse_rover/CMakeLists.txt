cmake_minimum_required(VERSION 3.0.2)
project(toulouse_rover)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND catkin_dependencies
  i2cpwm_board
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${catkin_dependencies}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   WheelEncoderCounts.msg
   WheelSpeeds.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS ${catkin_dependencies}
#  DEPENDS wiringPi
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/util.cpp
   src/wheel_speed_calculator.cpp
   src/wheel_speed_controller.cpp
   src/odom_calculator.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})

## Declare a C++ executable
# checkign if we are running on raspberry pi
# as we want to compile the wiringPi library if so, and add a custom flag
# so that in code we can tell what system we are running on
EXECUTE_PROCESS(COMMAND uname -r COMMAND tr -d "\n" OUTPUT_VARIABLE KERNELRELEASE)
message([DEBUG] Kernel release is: ${KERNELRELEASE})

if (${KERNELRELEASE} MATCHES "raspi$")
    message([DEBUG] Building on a Raspberry Pi)
else()
    message([DEBUG] Not building on a Raspberry Pi)
endif()

add_executable(test_motors_ros src/hardware_testing_scripts/test_motors_ros.cpp)
if (${KERNELRELEASE} MATCHES "raspi$")
    target_link_libraries(test_motors_ros ${catkin_LIBRARIES} -lwiringPi)
    set_target_properties(test_motors_ros PROPERTIES COMPILE_FLAGS "-DRPI")
else()
    target_link_libraries(test_motors_ros ${catkin_LIBRARIES})
endif()

add_executable(test_ir_ros src/hardware_testing_scripts/test_ir_ros.cpp)
if (${KERNELRELEASE} MATCHES "raspi$")
    target_link_libraries(test_ir_ros -lwiringPi)
    set_target_properties(test_ir_ros PROPERTIES COMPILE_FLAGS "-DRPI")
else()
    target_link_libraries(test_ir_ros)
endif()

add_executable(wheel_speed_calculator_node src/wheel_speed_calculator_node.cpp)
add_dependencies(wheel_speed_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_speed_calculator_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

add_executable(wheel_speed_controller_node src/wheel_speed_controller_node.cpp)
add_dependencies(wheel_speed_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_lib_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
if (${KERNELRELEASE} MATCHES "raspi$")
    target_link_libraries(wheel_speed_controller_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib -lwiringPi)
    set_target_properties(wheel_speed_controller_node PROPERTIES COMPILE_FLAGS "-DRPI")
else()
    target_link_libraries(wheel_speed_controller_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
endif()

add_executable(odom_calculator_node src/odom_calculator_node.cpp)
add_dependencies(odom_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_calculator_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_motors_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_ir_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}_lib
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch urdf rviz param msg maps
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS wheel_speed_controller_node wheel_speed_calculator_node odom_calculator_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_wheel_speed_calculator_node tests/test_wheel_speed_calculator/calculator.test tests/test_wheel_speed_calculator/test.cpp)
  target_link_libraries(test_wheel_speed_calculator_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
  add_dependencies(test_wheel_speed_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_lib)

  add_rostest_gtest(test_wheel_speed_controller_node tests/test_wheel_speed_controller/controller.test tests/test_wheel_speed_controller/test.cpp)
  target_link_libraries(test_wheel_speed_controller_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
  add_dependencies(test_wheel_speed_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_lib)

  #add_rostest_gtest(test_wheel_speed_controller_pid_node tests/test_wheel_speed_controller_pid/controller.test tests/test_wheel_speed_controller_pid/test.cpp)
  #target_link_libraries(test_wheel_speed_controller_pid_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
  #add_dependencies(test_wheel_speed_controller_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_lib)

  add_rostest_gtest(test_odom_calculator_node tests/test_odom_calculator_node/calculator.test tests/test_odom_calculator_node/test.cpp)
  target_link_libraries(test_odom_calculator_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
  add_dependencies(test_odom_calculator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_lib)
endif()