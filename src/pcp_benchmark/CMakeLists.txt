cmake_minimum_required(VERSION 3.16)
project(pcp_benchmark CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)

# Core library: constraint builder, metrics, shelf scene, deterministic query generator.
add_library(pcp_benchmark_core
  src/constraints.cpp
  src/metrics.cpp
  src/shelf_scene.cpp
  src/query_generator.cpp
  src/robot_model_files.cpp
)
target_include_directories(pcp_benchmark_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pcp_benchmark_core PUBLIC
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_planning_scene
  moveit_core::moveit_kinematic_constraints
  ${moveit_msgs_TARGETS}
  geometric_shapes::geometric_shapes
  srdfdom::srdfdom
  urdf::urdf
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp)

add_executable(generate_queries src/generate_queries_main.cpp)
target_link_libraries(generate_queries pcp_benchmark_core)

add_executable(pcp_benchmark src/benchmark_main.cpp)
target_link_libraries(pcp_benchmark
  pcp_benchmark_core
  rclcpp::rclcpp
  moveit_ros_planning::moveit_cpp
  moveit_ros_planning::moveit_planning_scene_monitor
  ${OCTOMAP_LIBRARIES}
  ompl::ompl)

# pcp_benchmark_core is exported (v0.2) so that pcp_direct reuses the same query generator, robot
# model loader and path metrics instead of copies.
install(TARGETS pcp_benchmark_core EXPORT export_pcp_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS generate_queries pcp_benchmark
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/make_robot_files.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # URDF/SRDF for the tests, expanded from the same xacros the launch files use.
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/ur5e_rgbd.urdf)
  set(TEST_SRDF ${CMAKE_CURRENT_BINARY_DIR}/ur5e.srdf)
  add_custom_command(
    OUTPUT ${TEST_URDF} ${TEST_SRDF}
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_robot_files.sh ${TEST_URDF} ${TEST_SRDF}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_robot_files.sh
    COMMENT "Expanding UR5e xacros for unit tests")
  add_custom_target(pcp_test_robot_files ALL DEPENDS ${TEST_URDF} ${TEST_SRDF})

  set(TEST_DEFS
    PCP_TEST_URDF="${TEST_URDF}"
    PCP_TEST_SRDF="${TEST_SRDF}"
    PCP_SHELF_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../pcp_scenes/config/shelf.yaml"
    PCP_COMMITTED_QUERIES="${CMAKE_CURRENT_SOURCE_DIR}/../../results/queries.json")
  foreach(t test_constraints test_metrics test_query_determinism)
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 300)
    target_link_libraries(${t} pcp_benchmark_core)
    target_compile_definitions(${t} PRIVATE ${TEST_DEFS})
    add_dependencies(${t} pcp_test_robot_files)
  endforeach()
endif()

ament_export_targets(export_pcp_benchmark HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core moveit_msgs geometric_shapes srdfdom urdf Eigen3 nlohmann_json
  yaml-cpp)
ament_package()
