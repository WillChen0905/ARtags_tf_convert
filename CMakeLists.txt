cmake_minimum_required( VERSION 2.8.3 )
project( ARtags_tf_convert )

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
    tf2_ros
    tf2_geometry_msgs
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable( ARtags_tf_convert src/ARtags_tf_convert.cpp )
target_link_libraries( ARtags_tf_convert ${catkin_LIBRARIES} )
