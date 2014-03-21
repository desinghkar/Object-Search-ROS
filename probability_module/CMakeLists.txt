cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(fuseProbabilities src/fuseProbabilities.cpp src/just_time.cpp src/just_vp_prob.cpp)
#rosbuild_add_executable(just_time src/just_time.cpp src/just_vp_prob.cpp)
#rosbuild_add_executable(localglobalmap src/localglobalmap.cpp src/just_vp_prob.cpp src/just_time.cpp)
rosbuild_add_executable(probability_update src/main.cpp src/localglobalmap.cpp src/fuseProbabilities.cpp src/just_vp_prob.cpp src/just_time.cpp)
rosbuild_add_executable(icvgip_results src/icvgip_results.cpp src/localglobalmap.cpp)
rosbuild_add_executable(potentialmap_update src/potentialmap_update.cpp src/localglobalmap.cpp src/fuseProbabilities.cpp src/just_vp_prob.cpp src/just_time.cpp)
#target_link_libraries(example ${PROJECT_NAME})
