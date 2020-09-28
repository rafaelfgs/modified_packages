cmake_minimum_required(VERSION 2.8.3)
project(espeleo_pose_ekf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  tf
)

find_package(Boost REQUIRED COMPONENTS thread)

# services
add_service_files(
    DIRECTORY srv
    FILES
    GetStatus.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(espeleo_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
target_link_libraries(espeleo_pose_ekf
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    )
add_dependencies(espeleo_pose_ekf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        espeleo_pose_ekf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES launch/espeleo_pose_ekf.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    PROGRAMS scripts/pitch_to_z_axis.py scripts/pose_to_odom.py scripts/wtf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
