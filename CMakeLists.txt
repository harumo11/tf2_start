cmake_minimum_required(VERSION 3.10)

project(tf_start)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf2
	tf2_ros
	)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
add_executable(frame_tf2_broadcaster src/frame_tf2_broadcaster.cpp)
add_executable(horikiri_rec src/horikiri_rec.cpp)

target_link_libraries(static_turtle_tf2_broadcaster ${catkin_LIBRARIES})
target_link_libraries(turtle_tf2_broadcaster ${catkin_LIBRARIES})
target_link_libraries(turtle_tf2_listener ${catkin_LIBRARIES})
target_link_libraries(frame_tf2_broadcaster ${catkin_LIBRARIES})
target_link_libraries(horikiri_rec ${catkin_LIBRARIES})
