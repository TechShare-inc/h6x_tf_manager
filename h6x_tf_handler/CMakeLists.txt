cmake_minimum_required(VERSION 3.8)
project(h6x_tf_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TARGET pose_tf_handler)
  ament_add_gtest(${TARGET} test/src/test_${TARGET}.cpp)
  ament_target_dependencies(${TARGET} tf2_geometry_msgs tf2_ros)
  target_include_directories(${TARGET} PRIVATE include)
endif()

ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_ros)
ament_auto_package()