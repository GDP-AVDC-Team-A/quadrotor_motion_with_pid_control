cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME thrust_controller_process)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

 

# MLA: Middle-Level Autopilot
set(DRONE_MLA_ROSMODULE_SOURCE_DIR
	src/source) 
	
set(DRONE_MLA_ROSMODULE_INCLUDE_DIR
	src/include)

set(DRONE_MLA_ROSMODULE_SOURCE_FILES
        ${DRONE_MLA_ROSMODULE_SOURCE_DIR}/thrust_controller_process.cpp

)
 
set(DRONE_MLA_ROSMODULE_HEADER_FILES
        ${DRONE_MLA_ROSMODULE_INCLUDE_DIR}/thrust_controller_process.h
)

find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs lib_cvgutils mav_msgs robot_process aerostack_msgs thrust_controller tf_conversions dynamic_reconfigure)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
    message(WARN "Using Eigen2!")
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

generate_dynamic_reconfigure_options(
    cfg/thrustController.cfg
 )


catkin_package(
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs lib_cvgutils mav_msgs robot_process aerostack_msgs thrust_controller tf_conversions dynamic_reconfigure
  )


include_directories(${DRONE_MLA_ROSMODULE_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(thrust_controller_process ${DRONE_MLA_ROSMODULE_SOURCE_FILES} ${DRONE_MLA_ROSMODULE_HEADER_FILES})
add_dependencies(thrust_controller_process ${catkin_EXPORTED_TARGETS})
add_dependencies(thrust_controller_process ${PROJECT_NAME}_gencfg)
target_link_libraries(thrust_controller_process ${catkin_LIBRARIES})

add_executable(thrust_controller_process_main ${DRONE_MLA_ROSMODULE_SOURCE_DIR}/thrust_controller_process_main.cpp)
add_dependencies(thrust_controller_process_main ${catkin_EXPORTED_TARGETS})
add_dependencies(thrust_controller_process_main ${PROJECT_NAME}_gencfg)
target_link_libraries(thrust_controller_process_main thrust_controller_process)
target_link_libraries(thrust_controller_process_main ${catkin_LIBRARIES})