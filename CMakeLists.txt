cmake_minimum_required(VERSION 2.8.3)
project(gazetool_filtering)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	gazetool
)

catkin_package(
	INCLUDE_DIRS src include
	LIBRARIES gazetool_filtering
	#LIBRARIES gazetool_filtering
	CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS system_lib
)

include_directories(
	include	
	${catkin_INCLUDE_DIRS}
)

add_library( subscribeAndPublish src/subscribeAndPublish.cpp )
add_dependencies(subscribeAndPublish ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscribeAndPublish ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)

add_dependencies(subscribeAndPublish ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node subscribeAndPublish
   ${catkin_LIBRARIES}
)

add_library( RLS_nan_filtering src/RLS_nan_filtering.cpp )
add_dependencies(RLS_nan_filtering ${catkin_EXPORTED_TARGETS})
target_link_libraries(RLS_nan_filtering ${catkin_LIBRARIES})

add_executable(RLS_nan_filtering_node src/RLS_nan_filtering.cpp)

add_dependencies(RLS_nan_filtering ${catkin_EXPORTED_TARGETS})
target_link_libraries(RLS_nan_filtering_node RLS_nan_filtering
   ${catkin_LIBRARIES}
)
