cmake_minimum_required(VERSION 2.8.3)
project(iri_mediapipe_graph)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm iri_skeleton_msgs cv_bridge image_transport camera_info_manager sensor_msgs tf geometry_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)

#include(FetchContent)
#FetchContent_Declare(
#  mediapipe
#  GIT_REPOSITORY https://github.com/google/mediapipe.git
#  GIT_TAG        v0.8.6
#)
#FetchContent_MakeAvailable(mediapipe)

#add_custom_command(OUTPUT libmediapipe.so
#                   COMMAND git reset --hard
#                   COMMAND git clean -f
#                   COMMAND git apply ${PROJECT_SOURCE_DIR}/patch/mediapipe.patch
#                   COMMAND bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop:mediapipe
#                   COMMENT "Bulding mediapipe"
#                   WORKING_DIRECTORY ${mediapipe_SOURCE_DIR})

add_custom_command(OUTPUT libmediapipe.so
                   COMMAND ${PROJECT_SOURCE_DIR}/scripts/set_mediapipe.sh ${PROJECT_SOURCE_DIR} 
                   COMMENT "Setting up mediapipe"
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

#
# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/MediapipeGraph.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS iri_base_algorithm iri_skeleton_msgs cv_bridge image_transport camera_info_manager sensor_msgs tf geometry_msgs
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
#  DEPENDS 
)

###########
## Build ##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_BINARY_DIR}/mediapipe)
include_directories(${CMAKE_BINARY_DIR}/mediapipe/bazel-bin)
include_directories(${CMAKE_BINARY_DIR}/mediapipe/bazel-bin/../../../../../external/com_google_absl)
include_directories(${CMAKE_BINARY_DIR}/mediapipe/bazel-bin/../../../../../external/com_google_protobuf/src/)
# include_directories(${<dependency>_INCLUDE_DIRS})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/mediapipe_graph_alg.cpp src/mediapipe_graph_alg_node.cpp libmediapipe.so)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_BINARY_DIR}/mediapipe/bazel-bin/mediapipe/examples/desktop/libmediapipe.so)
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARIES})

add_custom_command(TARGET ${PROJECT_NAME}
                   POST_BUILD
                   COMMAND ln -sfn ${CMAKE_BINARY_DIR}/mediapipe/mediapipe ~/.ros/mediapipe
                   COMMENT "Creating mediapipe softlink at ROS_HOME"
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} iri_skeleton_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} sensor_msgs_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
