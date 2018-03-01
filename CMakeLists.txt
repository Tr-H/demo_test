cmake_minimum_required(VERSION 2.8.3)
project(demo_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
 add_message_files(
   #DIRECTORY msg
   FILES
   pos_data.msg
   pos_write_data.msg
   pos_write_data2.msg
   pos_status.msg
   quadrotor_collect.msg
   velocity_feedback.msg
   attitude_feedback.msg
   body_attitude.msg
   position_setpoint.msg
   demo_code.msg
   formation_shape.msg
   formation_shape_pub.msg
   formation_data.msg
   pos_write_team.msg
   ekf_status.msg
   ekf_leader.msg
 )
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   teleop_ctrl.srv
#   Service2.srv
 )

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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_test
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(demo_test
#   src/${PROJECT_NAME}/demo_test.cpp
# )

## Declare a cpp executable
# add_executable(demo_test_node src/demo_test_node.cpp)
# add_executable(demo_test_node src/demo_test_node.cpp)
#add_executable(demo_hello src/demo_hello.cpp)
#add_executable(demo_udp src/demo_udp.cpp)
add_executable(ros_client src/ros_client.cpp)
add_executable(ros_client_2 src/ros_client_2.cpp)
#add_executable(demo_client src/demo_client.cpp)
#add_executable(demo_gcs3 src/demo_gcs3.cpp)
#add_executable(demo_gcs4 src/demo_gcs4.cpp)
#add_executable(demo_gcs5 src/demo_gcs5.cpp)
#add_executable(demo_pos_write src/demo_pos_write.cpp)
#add_executable(demo_pos_write_loop src/demo_pos_write_loop.cpp)
#add_executable(demo_gcs_API src/demo_gcs_API.cpp)
#add_executable(demo_collect src/demo_collect.cpp)
#add_executable(demo_read_data2 src/demo_read_data2.cpp)
#add_executable(demo_read_data3 src/demo_read_data3.cpp)
#add_executable(demo_read_data4 src/demo_read_data4.cpp)
add_executable(demo_read_data_traj_e src/demo_read_data_traj_e.cpp)
add_executable(demo_read_data_traj_e2 src/demo_read_data_traj_e2.cpp)
#add_executable(demo_fandl src/demo_fandl.cpp)
#add_executable(time_test src/time_test.cpp)
#add_executable(mavros_send src/mavros_send.cpp)
#add_executable(data_record src/data_record.cpp)
#add_executable(mavros_send_multi src/mavros_send_multi.cpp)
#add_executable(mavros_send_multi2 src/mavros_send_multi2.cpp)
#add_executable(mavros_send_multi3 src/mavros_send_multi3.cpp)
#add_executable(mavros_send_multi4 src/mavros_send_multi4.cpp)
add_executable(mavros_send_multi5 src/mavros_send_multi5.cpp)
add_executable(demo_fandl2 src/demo_fandl2.cpp)
add_executable(yuyin_test1 src/yuyin_test1.cpp)
add_executable(demo_pos_write2 src/demo_pos_write2.cpp)
add_executable(demo_nwforma_test1 src/demo_nwforma_test1.cpp)
add_executable(demo_team_pos_write src/demo_team_pos_write.cpp)
add_executable(demo_ekf_esitimator1 src/demo_ekf_esitimator1.cpp)
add_executable(faleleader_send_test1 src/ekf_test1.cpp)
add_executable(faleleader_send_test2 src/ekf_test2.cpp)
add_executable(mavros_att_test src/mavros_att_test.cpp)
add_executable(mavros_att_test_without_dynamic src/mavros_att_test_without_dynamic.cpp)
#add_executable(demo_traj_write src/demo_traj_write.cpp)
#add_executable(demo_traj_yuan src/demo_traj_yuan.cpp)
#add_executable(demo_yuan_xy src/demo_yuan_xy.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(demo_test_node demo_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(demo_test_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(demo_hello ${catkin_LIBRARIES})
#target_link_libraries(demo_udp ${catkin_LIBRARIES})
target_link_libraries(ros_client ${catkin_LIBRARIES})
target_link_libraries(ros_client_2 ${catkin_LIBRARIES})
#target_link_libraries(demo_client ${catkin_LIBRARIES})
#target_link_libraries(demo_gcs3 ${catkin_LIBRARIES})
#target_link_libraries(demo_gcs4 ${catkin_LIBRARIES})
#target_link_libraries(demo_gcs5 ${catkin_LIBRARIES})
#target_link_libraries(demo_pos_write ${catkin_LIBRARIES})
#target_link_libraries(demo_pos_write_loop ${catkin_LIBRARIES})
#target_link_libraries(demo_gcs_API ${catkin_LIBRARIES})
#target_link_libraries(demo_collect ${catkin_LIBRARIES})
#target_link_libraries(demo_read_data2 ${catkin_LIBRARIES})
#target_link_libraries(demo_read_data3 ${catkin_LIBRARIES})
#target_link_libraries(demo_read_data4 ${catkin_LIBRARIES})
#target_link_libraries(demo_read_data4 ${catkin_LIBRARIES})
target_link_libraries(demo_read_data_traj_e ${catkin_LIBRARIES})
target_link_libraries(demo_read_data_traj_e2 ${catkin_LIBRARIES})
#target_link_libraries(demo_fandl ${catkin_LIBRARIES})
#target_link_libraries(time_test ${catkin_LIBRARIES})
#target_link_libraries(mavros_send ${catkin_LIBRARIES})
#target_link_libraries(data_record ${catkin_LIBRARIES})
#target_link_libraries(mavros_send_multi ${catkin_LIBRARIES})
#target_link_libraries(mavros_send_multi2 ${catkin_LIBRARIES})
#target_link_libraries(mavros_send_multi3 ${catkin_LIBRARIES})
#target_link_libraries(mavros_send_multi4 ${catkin_LIBRARIES})
target_link_libraries(mavros_send_multi5 ${catkin_LIBRARIES})
target_link_libraries(demo_fandl2 ${catkin_LIBRARIES})
target_link_libraries(yuyin_test1 ${catkin_LIBRARIES})
target_link_libraries(demo_pos_write2 ${catkin_LIBRARIES})
target_link_libraries(demo_nwforma_test1 ${catkin_LIBRARIES})
target_link_libraries(demo_team_pos_write ${catkin_LIBRARIES})
target_link_libraries(demo_ekf_esitimator1 ${catkin_LIBRARIES})
target_link_libraries(faleleader_send_test1 ${catkin_LIBRARIES})
target_link_libraries(faleleader_send_test2 ${catkin_LIBRARIES})
target_link_libraries(mavros_att_test ${catkin_LIBRARIES})
target_link_libraries(mavros_att_test_without_dynamic ${catkin_LIBRARIES})
#target_link_libraries(demo_traj_write ${catkin_LIBRARIES})
#target_link_libraries(demo_traj_yuan ${catkin_LIBRARIES})
#target_link_libraries(demo_yuan_xy ${catkin_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#install(PROGRAMS
#    scripts/demo_test_py.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
catkin_install_python(PROGRAMS
    scripts/demo_test_py.py 
    scripts/demo_test_py2.py 
    scripts/mocap_send.py
    scripts/mocap_send_1.py 
    scripts/offboard_test.py 
    scripts/teleop_keyboard_control.py
    scripts/teleop_keyboard_control_1.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
## Mark executables and/or libraries for installation
# install(TARGETS demo_test demo_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(TARGETS demo_test demo_test_py_node
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
