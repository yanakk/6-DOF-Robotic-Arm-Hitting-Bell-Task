cmake_minimum_required(VERSION 2.8.3)
project(probot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        moveit_msgs
        moveit_ros_perception
        moveit_ros_planning_interface
        trajectory_msgs
        probot_msgs
        probot_anno_ikfast_manipulator_plugin
        message_generation
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy moveit_core moveit_ros_planning_interface probot_msgs probot_anno_ikfast_manipulator_plugin std_msgs message_runtime
)

include_directories(
${PROJECT_SOURCE_DIR}/inc
${catkin_INCLUDE_DIRS}
)
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
set(PATH ${DIR_SRCS})
add_executable(cxz ${PATH})

target_link_libraries(cxz ${catkin_LIBRARIES})


install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


