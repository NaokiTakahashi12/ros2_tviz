cmake_minimum_required(VERSION 3.8)
project(ros2_tviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(urdfdom REQUIRED)
find_package(ftxui REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(joint_trajectory_tui_node_parameters
  src/joint_trajectory_tui_node_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_components SHARED
  src/joint_trajectory_tui_node.cpp
)
target_link_libraries(${PROJECT_NAME}_components
  ${urdfdom_LIBRARIES}
  ftxui::screen
  ftxui::dom
  ftxui::component
  joint_trajectory_tui_node_parameters
)
target_include_directories(${PROJECT_NAME}_components
  PUBLIC
    ${urdfdom_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ros2_tviz::JointTrajectoryTuiNode"
  EXECUTABLE joint_trajectory_tui_node
)

install(
  TARGETS
    ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
