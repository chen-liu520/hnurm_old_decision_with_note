cmake_minimum_required(VERSION 3.8)
project(hnurm_decision)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(hnurm_interfaces_DIR /install/hnurm_interfaces/share/hnurm_interfaces/cmake)
# set(hnurm_decision_interfaces_DIR /install/hnurm_decision_interfaces/share/hnurm_decision_interfaces/cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hnurm_interfaces REQUIRED)
find_package(hnurm_armor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(hnurm_decision_interfaces REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(bt_node src/decision.cpp)
ament_target_dependencies(bt_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  hnurm_interfaces
  hnurm_armor
  visualization_msgs
  
)

add_executable(points_generator_node src/points_generator_node.cpp)
ament_target_dependencies(points_generator_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  geometry_msgs
  hnurm_interfaces
  visualization_msgs
)


add_library(gamestart_condition_node STATIC
  plugins/condition/GameStart.cpp  # 或其他相关的源文件
)
ament_target_dependencies(gamestart_condition_node
  rclcpp
  std_msgs
  behaviortree_cpp_v3
  hnurm_interfaces
)



add_library(pubRobotStatus_action_node STATIC
  plugins/action/pubRobotStatus.cpp  # 或其他相关的源文件
)
ament_target_dependencies(pubRobotStatus_action_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  behaviortree_cpp_v3
  hnurm_interfaces
  hnurm_armor
  visualization_msgs
)

add_library(status_condition_node STATIC
  plugins/condition/Status.cpp  # 或其他相关的源文件
)
ament_target_dependencies(status_condition_node
  rclcpp
  std_msgs
  behaviortree_cpp_v3
  hnurm_interfaces
)



target_link_libraries(bt_node   
 gamestart_condition_node  
 pubRobotStatus_action_node
 status_condition_node
)  # 添加这行来链接库


install(DIRECTORY launch params
	DESTINATION share/${PROJECT_NAME}
	  )
    
# Install the executable
install(TARGETS
  bt_node
  points_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Export dependencies
ament_package()
