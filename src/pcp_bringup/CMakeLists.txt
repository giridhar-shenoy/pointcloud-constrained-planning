cmake_minimum_required(VERSION 3.16)
project(pcp_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/smoke_check.py scripts/record_demo.py DESTINATION lib/${PROJECT_NAME})

ament_package()
