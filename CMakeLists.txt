set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 2.8.3)
project(robot_self_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED 
  src/load_mesh.cpp
  src/shapes.cpp
  src/bodies.cpp
)

add_library(${PROJECT_NAME} src/self_mask.cpp)
target_link_libraries(${PROJECT_NAME} robot_geometric_shapes assimp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "::Publisher"
  EXECUTABLE test_filter
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "kccs_pub_sub_cpp_02::Subscriber"
  EXECUTABLE self_filter
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
add_executable(test_filter src/test_filter.cpp)


add_executable(self_filter src/self_filter.cpp)


