cmake_minimum_required(VERSION 3.5)

# Appends the cmake/modules path to MAKE_MODULE_PATH variable.
project(udp_com)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
  message_generation
  nodelet
)

## System dependencies are found with CMake's conventions
## find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
## catkin_python_setup()

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
 add_message_files(
   DIRECTORY msg
   FILES UdpPacket.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  UdpSend.srv
  UdpSocket.srv
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
   DEPENDENCIES std_msgs
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nodelet
  message_generation
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

## Declare a cpp library
add_library( ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_nodelet.cpp
)

## Declare a cpp executable
# add_executable(udp_com_node src/udp_com_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}
	${PROJECT_NAME}_generate_messages_cpp
	${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
 )

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

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
#Launch Files 
install(FILES
  launch/${PROJECT_NAME}.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

#nodelets.xml
install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# ROS linter
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(gtest)
  
  # Add rostest gtest
  add_rostest_gtest(tests_${PROJECT_NAME}
	  test/${PROJECT_NAME}.test
	  test/${PROJECT_NAME}-test.cpp
          test/test_main.cpp)
  target_link_libraries(tests_${PROJECT_NAME}
          ${PROJECT_NAME}
          ${catkin_LIBRARIES})
endif()


####################
## Deb Generation ##
####################

set(CPACK_GENERATOR "DEB")

#Creating UNIX structured package
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/dist")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

#Set directory to /../ to avoid /share part of ROS_PACKAGE_PATH
set(CPACK_PACKAGING_INSTALL_PREFIX "$ENV{ROS_PACKAGE_PATH}/../")
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};ALL;/./")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "RCS")

#Now we can simply create the dependencies using almost the same package names as they declared in ROS repository
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-melodic-roscpp, ros-melodic-std-msgs, ros-melodic-message-runtime, ros-melodic-sensor-msgs")

#IMPORTANT: set this to keep on auto-introspection for system dependencies like libc++6
set (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

#Keep Debian package structure untouched
set(CPACK_DEBIAN_PACKAGE_CONTROL_STRICT_PERMISSION TRUE)

#Standard naming schema for Debian-based distros
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
set(CPACK_PACKAGE_VENDOR "Continental LiDAR Segment")
set(CPACK_PACKAGE_CONTACT "<moises.diaz@continental-corporation.com>")

#Turn the process on    
include(CPack)
