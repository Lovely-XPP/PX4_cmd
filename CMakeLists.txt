cmake_minimum_required(VERSION 2.8.3)
project(px4_cmd)

# find package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    mavros_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)


# include dir
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


# Exec
add_executable(set_mode src/set_mode.cpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})

