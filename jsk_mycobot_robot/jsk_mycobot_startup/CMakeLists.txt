cmake_minimum_required(VERSION 2.8.3)
project(jsk_mycobot_startup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roslaunch roslint roscpp geometry_msgs tf2_geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS geometry_msgs tf2_geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(tf_broadcaster src/tf_broadcaster.cpp) 
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint roscpp geometry_msgs tf2_geometry_msgs)

  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  set(ROSLINT_PYTHON_OPTS --max-line-length=180 --ignore=E221,E222,E241) # skip multiple spaces before/after operator
  roslint_python()
  #roslint_add_test()
endif()
