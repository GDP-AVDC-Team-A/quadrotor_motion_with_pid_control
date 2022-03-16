cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME thrust_controller)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
#add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
add_definitions(-std=c++03)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




#DRONE ARUCO EYE
set(THRUSTCONTROLLER_SOURCE_DIR
	src/source) 
	
set(THRUSTCONTROLLER_INCLUDE_DIR
	src/include)

set(THRUSTCONTROLLER_SOURCE_FILES
        ${THRUSTCONTROLLER_SOURCE_DIR}/ThrustController.cpp
        ${THRUSTCONTROLLER_SOURCE_DIR}/DMLA_AltSpeedController.cpp
        ${THRUSTCONTROLLER_SOURCE_DIR}/DMLA_HoverController.cpp
        ${THRUSTCONTROLLER_SOURCE_DIR}/DMLA_YawController.cpp
)
 
set(THRUSTCONTROLLER_HEADER_FILES
        ${THRUSTCONTROLLER_INCLUDE_DIR}/ThrustController.h
        ${THRUSTCONTROLLER_INCLUDE_DIR}/DMLA_AltSpeedController.h
        ${THRUSTCONTROLLER_INCLUDE_DIR}/DMLA_HoverController.h
        ${THRUSTCONTROLLER_INCLUDE_DIR}/DMLA_YawController.h
)


find_package(catkin REQUIRED
		COMPONENTS lib_pugixml lib_cvgutils
)


catkin_package(
        INCLUDE_DIRS ${THRUSTCONTROLLER_INCLUDE_DIR}
        LIBRARIES thrust_controller
        CATKIN_DEPENDS lib_pugixml lib_cvgutils
  )


include_directories(${THRUSTCONTROLLER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(thrust_controller ${THRUSTCONTROLLER_SOURCE_FILES} ${THRUSTCONTROLLER_HEADER_FILES})
add_dependencies(thrust_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(thrust_controller ${catkin_LIBRARIES})

