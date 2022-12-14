cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif(POLICY CMP0048)
# project name version languages
project(px4_cmd VERSION 1.0.3 LANGUAGES CXX)

# set C++ standard
set(CMAKE_CXX_FLAGS "-std=c++11")

# find package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    mavros_msgs
    geometry_msgs
    message_generation
)

# message generation
add_message_files(
    DIRECTORY msg
    FILES
    Command.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs
)

# include dir
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ##################  Exec  ###################
# set_mode
add_executable(set_mode src/set_mode.cpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})

# set_command
add_executable(set_command src/set_command.cpp)
add_dependencies(set_command ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(set_command ${catkin_LIBRARIES})

# send_command
add_executable(send_command src/send_command.cpp)
add_dependencies(send_command ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(send_command ${catkin_LIBRARIES})

