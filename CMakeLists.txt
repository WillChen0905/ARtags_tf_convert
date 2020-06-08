cmake_minimum_required( VERSION 2.8.3 )
project( artags_tf_convert )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    ar_track_alvar_msgs
)

catkin_package()


include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable( artags_tf_convert src/artags_tf_convert.cpp )
target_link_libraries( artags_tf_convert ${catkin_LIBRARIES} )
