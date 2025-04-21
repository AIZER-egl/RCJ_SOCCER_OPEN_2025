cmake_minimum_required(VERSION 3.0.2)
project(robocup2024)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS
		core imgproc highgui videoio
		cudaarithm
		cudaimgproc
		cudawarping
		cudafilters
)

# Require C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES robocup2024
		#  CATKIN_DEPENDS roscpp rospy
		#  DEPENDS system_lib
)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
)

#add_executable(frontcamera
#		src/frontcamera.cpp
#		lib/blob_detection.cpp
#		lib/preprocessing.cpp
#		lib/gstreamer.cpp
#)
#target_link_libraries(frontcamera ${catkin_LIBRARIES})
#target_link_libraries(frontcamera ${OpenCV_LIBS})


add_executable(omnicamera
		src/omnicamera.cpp
		lib/blob_detection.cpp
		lib/preprocessing.cpp
		lib/gstreamer.cpp
)

target_link_libraries(omnicamera ${catkin_LIBRARIES})
target_link_libraries(omnicamera ${OpenCV_LIBS})

add_executable(pico_data_processor
		src/pico_data_processor.cpp
		lib/chassis.cpp
		lib/angle.cpp
		lib/pid.cpp
		lib/serializer.cpp
)
target_link_libraries(pico_data_processor ${catkin_LIBRARIES})
target_link_libraries(pico_data_processor ${OpenCV_LIBS})
