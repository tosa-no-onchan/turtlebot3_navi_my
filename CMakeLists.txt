cmake_minimum_required(VERSION 3.5)
project(turtlebot3_navi_my)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED )


add_executable(multi_goals4_cmd_vel src/multi_goals4_cmd_vel.cpp src/multi_goals.cpp src/robot_drive.cpp src/com_lib.cpp)
target_compile_options(multi_goals4_cmd_vel PRIVATE -Wno-unused-parameter)

add_executable(drive_base src/drive_base.cpp src/robot_drive.cpp src/com_lib.cpp)
target_compile_options(drive_base PRIVATE -Wno-unused-parameter)

#add_executable(drive_base src/drive_base.cpp src/robot_driveNAV2.cpp src/com_lib.cpp)
add_executable(tf_listener_test src/tf_listener_test.cpp src/com_lib.cpp)
#add_executable(nav2_send_goal src/nav2_send_goal.cpp)

add_executable(multi_goals4_nav2 src/multi_goals4_nav2.cpp src/multi_goals.cpp src/robot_driveNAV2.cpp src/com_lib.cpp)
target_compile_options(multi_goals4_nav2 PRIVATE -DUSE_MOVE_BASE)
target_compile_options(multi_goals4_nav2 PRIVATE -Wno-unused-parameter)

target_include_directories(multi_goals4_cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  )

target_include_directories(multi_goals4_nav2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  )

target_include_directories(drive_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tf_listener_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#target_include_directories(nav2_send_goal PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(multi_goals4_cmd_vel
  rclcpp
  std_msgs 
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  OpenCV
)

ament_target_dependencies(multi_goals4_nav2
  rclcpp
  std_msgs 
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  OpenCV
)

ament_target_dependencies(drive_base 
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
    nav2_msgs
)

ament_target_dependencies(tf_listener_test
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
)

#ament_target_dependencies(nav2_send_goal
#    rclcpp
#    std_msgs 
#    geometry_msgs
#    tf2
#    tf2_ros
#    nav2_msgs
#)

install(TARGETS
  multi_goals4_cmd_vel
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  drive_base
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  tf_listener_test
  DESTINATION lib/${PROJECT_NAME})

#install(TARGETS
#  nav2_send_goal
#  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  multi_goals4_nav2
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
