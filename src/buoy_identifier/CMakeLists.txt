SET(execName buoy_identifier)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)
PROJECT(${execName})

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


#Fix for QtCretor
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/")

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(catkin COMPONENTS
  roscpp
  rosconsole
  nodelet
std_msgs
geometry_msgs
message_generation
cv_bridge
image_transport
actionlib 
move_base_msgs
tf
)
add_message_files(
FILES
Buoy.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    zed_nodelets
cv_bridge
image_transport
)



IF(NOT WIN32)
    add_definitions(-Wno-format-extra-args)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF()

###############################################################################
# SOURCES

set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/navigator.cpp)
###############################################################################

###############################################################################

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/msg)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

add_definitions(-std=c++11)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
${OpenCV_LIBS}
)

add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(main ${LINK_LIBRARIES})
add_dependencies(main buoy_identifier_generate_messages_cpp)
add_executable(navigator ${CMAKE_CURRENT_SOURCE_DIR}/src/navigator.cpp)
target_link_libraries(navigator ${LINK_LIBRARIES})
add_dependencies(navigator buoy_identifier_generate_messages_cpp)
FILE(GLOB_RECURSE SRC_FILES src/*.cpp)

###############################################################################
#Add all files in subdirectories of the project in

# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

###############################################################################
# INSTALL


#ADD_EXECUTABLE(${execName} ${SRC_FILES})
add_definitions(-std=c++14 -g -O3)

#if (LINK_SHARED_ZED)
#    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
#else()
#    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
#endif()

#TARGET_LINK_LIBRARIES(${execName} ${ZED_LIBS} ${OpenCV_LIBRARIES})


install(TARGETS
  main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY
 # launch
#  urdf
 # params
 # DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


