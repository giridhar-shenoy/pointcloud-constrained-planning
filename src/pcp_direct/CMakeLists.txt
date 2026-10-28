cmake_minimum_required(VERSION 3.16)
project(pcp_direct CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcp_benchmark REQUIRED)
# Pinocchio last: its config pulls in eigenpy's own FindBoost module, which leaves
# Boost_USE_DEBUG_RUNTIME set and makes later Boost component lookups (MoveIt's) fail.
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)
unset(Boost_USE_DEBUG_RUNTIME)

# ---- The direct pipeline: no MoveIt in these libraries ----------------------------------------
add_library(pcp_direct_collision src/collision.cpp)
target_include_directories(pcp_direct_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pcp_direct_collision PUBLIC
  pinocchio::pinocchio_parsers
  pinocchio::pinocchio_collision
  coal::coal
  Eigen3::Eigen
  yaml-cpp::yaml-cpp)

add_library(pcp_direct_planner src/planner.cpp)
target_link_libraries(pcp_direct_planner PUBLIC pcp_direct_collision ompl::ompl yaml-cpp::yaml-cpp)

add_library(pcp_direct_traj src/traj.cpp)
target_link_libraries(pcp_direct_traj PUBLIC
  pcp_direct_planner ${trajectory_msgs_TARGETS} yaml-cpp::yaml-cpp)

# ---- MoveIt reference side ----------------------------------------------------------------------
# Shelf boxes -> moveit_msgs/CollisionObject for MoveIt's planning-scene monitor.
add_executable(shelf_boxes_loader src/shelf_boxes_loader_main.cpp)
target_link_libraries(shelf_boxes_loader pcp_direct_collision rclcpp::rclcpp
  ${moveit_msgs_TARGETS} ${shape_msgs_TARGETS} ${geometry_msgs_TARGETS})

# MoveIt's planning scene vs the direct checker on seeded random configurations.
add_executable(collision_agreement src/collision_agreement_main.cpp)
target_link_libraries(collision_agreement
  pcp_direct_collision
  pcp_benchmark::pcp_benchmark_core
  rclcpp::rclcpp
  moveit_ros_planning::moveit_cpp
  moveit_ros_planning::moveit_planning_scene_monitor
  nlohmann_json::nlohmann_json)

install(TARGETS pcp_direct_collision pcp_direct_planner pcp_direct_traj
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS shelf_boxes_loader collision_agreement RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # URDF/SRDF for the tests, expanded by pcp_benchmark's script from the same xacros (and with the
  # same arguments) that the launch files hand to MoveIt.
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/ur5e_rgbd.urdf)
  set(TEST_SRDF ${CMAKE_CURRENT_BINARY_DIR}/ur5e.srdf)
  set(MAKE_ROBOT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../pcp_benchmark/scripts/make_robot_files.sh)
  add_custom_command(
    OUTPUT ${TEST_URDF} ${TEST_SRDF}
    COMMAND ${MAKE_ROBOT_FILES} ${TEST_URDF} ${TEST_SRDF}
    DEPENDS ${MAKE_ROBOT_FILES}
    COMMENT "Expanding UR5e xacros for pcp_direct unit tests")
  add_custom_target(pcp_direct_test_robot_files ALL DEPENDS ${TEST_URDF} ${TEST_SRDF})
  set(TEST_DEFS
    PCP_TEST_URDF="${TEST_URDF}"
    PCP_TEST_SRDF="${TEST_SRDF}"
    PCP_SHELF_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../pcp_scenes/config/shelf.yaml"
    PCP_OMPL_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../pcp_moveit_config/config/ompl_planning.yaml"
    PCP_JOINT_LIMITS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/joint_limits.yaml"
    PCP_COMMITTED_BOX_QUERIES="${CMAKE_CURRENT_SOURCE_DIR}/../../results/queries_boxes.json")

  foreach(t test_collision test_planner test_traj test_queries_boxes)
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 300)
    target_link_libraries(${t} pcp_direct_traj pcp_benchmark::pcp_benchmark_core
      ament_index_cpp::ament_index_cpp)
    target_compile_definitions(${t} PRIVATE ${TEST_DEFS})
    add_dependencies(${t} pcp_direct_test_robot_files)
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(pcp_direct_collision pcp_direct_planner pcp_direct_traj)
ament_package()
