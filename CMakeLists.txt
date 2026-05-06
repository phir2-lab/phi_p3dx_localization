cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# ---------- C++ executable nodes ----------
add_executable(localization_node_cpp src/localization_example.cpp src/localization_node.cpp)
ament_target_dependencies(localization_node_cpp
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# ---------- Installation ----------
install(TARGETS localization_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts rviz
  DESTINATION share/${PROJECT_NAME}
)

# ---------- Testing ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
