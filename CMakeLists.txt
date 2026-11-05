cmake_minimum_required(VERSION 3.8)

project(sicnav_airport_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds maps models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/controller_bootstrap.sh
    scripts/sicnav_tcp_bridge.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
