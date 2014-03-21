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
rosbuild_add_executable(test_viewpoint src/test_viewpoint.cpp)
rosbuild_add_executable(test_goals src/test_goals.cpp)
rosbuild_add_executable(time_test src/time_test.cpp)
rosbuild_add_executable(test_goal_poses src/test_goal_poses.cpp)
rosbuild_add_executable(test_Mat src/test_Mat.cpp)
rosbuild_add_executable(draw_nodes src/draw_nodes.cpp)
rosbuild_add_executable(test_karthik src/test_karthik.cpp)
rosbuild_add_executable(test_initial_pose src/test_initial_pose.cpp)
rosbuild_add_executable(probability_update src/probability_update.cpp)
rosbuild_add_executable(probability_update_backup src/probability_update_backup.cpp)
rosbuild_add_executable(potentialmap_update src/potentialmap_update.cpp)
#target_link_libraries(example ${PROJECT_NAME})
