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

# gdb option
#set(CMAKE_CXX_FLAGS "-Warray-bounds -g3")

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
find_package(sensor_msgs REQUIRED )
find_package(tf2_geometry_msgs REQUIRED)

# add by nishi 2024.12.23
list(APPEND CMAKE_MODULE_PATH "/home/nishi/Documents/VisualStudio-CPP/tflite/opp_tflite/cmake")
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Tflite REQUIRED) # TFLITE_INCLUDE_DIR, TFLITE_LIBRARY
find_package(Opp_tflite REQUIRED)

add_executable(multi_goals4_cmd_vel 
  src/multi_goals4_cmd_vel.cpp 
  src/pro_control.cpp
  src/pro_control_sub.cpp
  src/robot_driveCmd_Vel.cpp 
  src/com_lib.cpp)
target_compile_options(multi_goals4_cmd_vel PRIVATE -Wno-unused-parameter)

add_executable(drive_base src/drive_base.cpp 
  src/pro_control_sub.cpp 
  src/robot_driveCmd_Vel.cpp 
  src/com_lib.cpp)
target_compile_options(drive_base PRIVATE -Wno-unused-parameter)

add_executable(tf_listener_test src/tf_listener_test.cpp src/com_lib.cpp)
add_executable(tf_rate_checker src/tf_rate_checker.cpp src/com_lib.cpp)

add_executable(subscribe_topic_once src/subscribe_topic_once.cpp src/com_lib.cpp)
add_executable(subscribe_test src/subscribe_test.cpp src/com_lib.cpp)

add_executable(multi_goals4_nav2 src/multi_goals4_nav2.cpp 
  src/pro_control.cpp 
  src/pro_control_sub.cpp
  src/robot_driveNAV2.cpp 
  src/robot_driveCmd_Vel.cpp 
  src/com_lib.cpp)

#target_compile_options(multi_goals4_nav2 PRIVATE -DUSE_MOVE_BASE)
target_compile_options(multi_goals4_nav2 PRIVATE -DUSE_NAV2)
target_compile_options(multi_goals4_nav2 PRIVATE -Wno-unused-parameter)

add_executable(go_auto_map src/go_auto_map.cpp 
  src/pro_control.cpp 
  src/pro_control_sub.cpp
  src/pro_control_map.cpp 
  src/robot_driveNAV2.cpp 
  src/robot_driveCmd_Vel.cpp 
  src/com_lib.cpp)

#target_compile_options(go_auto_map PRIVATE -DUSE_MOVE_BASE)
target_compile_options(go_auto_map PRIVATE -DUSE_NAV2)
target_compile_options(go_auto_map PRIVATE -Wno-unused-parameter)


add_executable(go_auto_mower src/go_auto_mower.cpp 
  src/pro_control.cpp 
  src/pro_control_sub.cpp
  src/ml_planner.cpp 
  src/pro_control_mower.cpp 
  src/robot_driveNAV2.cpp 
  src/robot_driveCmd_Vel.cpp 
  src/com_lib.cpp)

target_compile_options(go_auto_mower PRIVATE -DUSE_NAV2)
target_compile_options(go_auto_mower PRIVATE -Wno-unused-parameter)


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

target_include_directories(go_auto_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  )

target_include_directories(go_auto_mower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${Tflite_INCLUDE_DIR}
  ${Opp_tflite_INCLUDE_DIR}
  )

target_include_directories(drive_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tf_listener_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tf_rate_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(subscribe_topic_once PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(subscribe_test PUBLIC
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
  #nav_msgs
  nav2_msgs
  OpenCV
  sensor_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(multi_goals4_nav2
  rclcpp
  std_msgs 
  geometry_msgs
  tf2
  tf2_ros
  #nav_msgs
  nav2_msgs
  OpenCV
  sensor_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(go_auto_map
  rclcpp
  std_msgs 
  geometry_msgs
  tf2
  tf2_ros
  #nav_msgs
  nav2_msgs
  OpenCV
  sensor_msgs
  tf2_geometry_msgs
)

# link tf-lite and opp_tflite
target_link_libraries(go_auto_mower
  -Wl,--no-as-needed
  ${Opp_tflite_LIBRARY} # 同一パッケージの共有ライブラリ名(add_libraryの名前)
  ${Tflite_LIBRARY}
)
ament_target_dependencies(go_auto_mower
  rclcpp
  std_msgs 
  geometry_msgs
  tf2
  tf2_ros
  #nav_msgs
  nav2_msgs
  OpenCV
  sensor_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(drive_base 
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
    nav2_msgs
    OpenCV
    sensor_msgs
    tf2_geometry_msgs
)

ament_target_dependencies(tf_listener_test
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
    sensor_msgs
    tf2_geometry_msgs
)

ament_target_dependencies(tf_rate_checker
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
    sensor_msgs
    tf2_geometry_msgs
)

ament_target_dependencies(subscribe_topic_once
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
)

ament_target_dependencies(subscribe_test
    rclcpp
    std_msgs 
    geometry_msgs
    tf2
    tf2_ros
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
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

install(TARGETS
  tf_rate_checker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  subscribe_topic_once
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  subscribe_test
  DESTINATION lib/${PROJECT_NAME})

#install(TARGETS
#  nav2_send_goal
#  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  multi_goals4_nav2
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  go_auto_map
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  go_auto_mower
  DESTINATION lib/${PROJECT_NAME})
  
# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# add by nishi for params 2024.4.6
install(
  DIRECTORY params map
  DESTINATION share/${PROJECT_NAME}/
)

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
