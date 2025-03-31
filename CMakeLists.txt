cmake_minimum_required(VERSION 3.8)
project(robot_self_filter_oedo)

# C++14標準の設定
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージを明示的に検索
find_package(ament_cmake_auto REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(resource_retriever REQUIRED)  # resource_retrieverパッケージを追加
find_package(PkgConfig REQUIRED)
pkg_check_modules(ASSIMP REQUIRED assimp)

# Mark all packages as REQUIRED
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# まず、robot_geometric_shapesを作成
add_library(robot_geometric_shapes SHARED
  src/load_mesh.cpp
  src/shapes.cpp
)

# インクルードディレクトリを設定
target_include_directories(robot_geometric_shapes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_geometry_msgs_INCLUDE_DIRS}  # tf2_geometry_msgsのインクルードパスを追加
  ${resource_retriever_INCLUDE_DIRS}  # resource_retrieverのインクルードパスを追加
  ${ASSIMP_INCLUDE_DIRS}  # Assimpのインクルードディレクトリを追加
  ${visualization_msgs_INCLUDE_DIRS}
)

# 依存関係を追加
target_link_libraries(robot_geometric_shapes
  ${tf2_geometry_msgs_LIBRARIES}
  ${resource_retriever_LIBRARIES}  # resource_retrieverライブラリを追加
  ${ASSIMP_LIBRARIES}  # Assimpライブラリを追加
  ${visualization_msgs_LIBRARIES}
)

# robot_self_filterのメインライブラリを作成
ament_auto_add_library(${PROJECT_NAME} SHARED 
  src/bodies.cpp
  src/self_mask.cpp
)

# ライブラリの依存関係を設定
target_link_libraries(${PROJECT_NAME} 
  robot_geometric_shapes 
  ${ASSIMP_LIBRARIES}  # ${PROJECT_NAME}に対してもAssimpライブラリを直接リンク
)

# 実行ファイルを作成
add_executable(test_filter_exec src/test_filter.cpp)
target_link_libraries(test_filter_exec ${PROJECT_NAME})
set_target_properties(test_filter_exec PROPERTIES OUTPUT_NAME test_filter)

add_executable(self_filter_exec src/self_filter.cpp)
target_link_libraries(self_filter_exec ${PROJECT_NAME})
set_target_properties(self_filter_exec PROPERTIES OUTPUT_NAME self_filter)

# コンポーネントとして登録
# 注：クラス名を正確に指定する必要があります
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "robot_self_filter::TestSelfFilter"  # 正確なクラス名と名前空間を指定
  EXECUTABLE test_filter_component
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "robot_self_filter::SelfFilter"  # 正確なクラス名と名前空間を指定
  EXECUTABLE self_filter_component
)

# インストール設定
install(TARGETS
  robot_geometric_shapes
  test_filter_exec
  self_filter_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# インクルードディレクトリもインストール
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()