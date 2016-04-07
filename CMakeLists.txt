cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME imu_calibration)
project(${PROJECT_NAME})


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    #set(CMAKE_CXX_FLAGS "-std=c++11")
    add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
    #set(CMAKE_CXX_FLAGS "-std=c++0x")
    add_definitions(-std=c++0x)
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
 


set(IMU_CALIBRATION_SOURCE_DIR
	src/source) 
	
set(IMU_CALIBRATION_INCLUDE_DIR
	src/include)

set(IMU_CALIBRATION_SOURCE_FILES
  ${IMU_CALIBRATION_SOURCE_DIR}/calibration.cc
  ${IMU_CALIBRATION_SOURCE_DIR}/mikrokopter_codels.c
  ${IMU_CALIBRATION_SOURCE_DIR}/mikrokopter_comm_codels.c
  ${IMU_CALIBRATION_SOURCE_DIR}/mikrokopter_main_codels.c
  ${IMU_CALIBRATION_SOURCE_DIR}/tty.c

  ${IMU_CALIBRATION_SOURCE_DIR}/calibration_ros.cpp
)
 
set(IMU_CALIBRATION_HEADER_FILES
  ${IMU_CALIBRATION_INCLUDE_DIR}/imu_calibration/codels.h
  ${IMU_CALIBRATION_INCLUDE_DIR}/imu_calibration/mikrokopter_c_types.h
  ${IMU_CALIBRATION_INCLUDE_DIR}/imu_calibration/acmikrokopter.h

  ${IMU_CALIBRATION_INCLUDE_DIR}/imu_calibration/calibration_ros.h
)



find_package(catkin REQUIRED
                COMPONENTS cmake_modules roscpp std_srvs std_msgs sensor_msgs)

find_package(Eigen REQUIRED)


catkin_package(
    DEPENDS Eigen
    CATKIN_DEPENDS roscpp std_srvs std_msgs sensor_msgs
  )


include_directories(${IMU_CALIBRATION_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME}_lib ${IMU_CALIBRATION_SOURCE_FILES} ${IMU_CALIBRATION_HEADER_FILES})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME} ${IMU_CALIBRATION_SOURCE_DIR}/calibration_node.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

