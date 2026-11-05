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
    scripts/sicnav_gazebo_client.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Include the inference Python sources, configs and checkpoint assets for local mode.
# Native Acados/RVO2 libraries must be built on Ubuntu, not copied from macOS.
install(
  DIRECTORY sicnav
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "acados_cache" EXCLUDE
  PATTERN "*.dylib" EXCLUDE
  PATTERN "*.so" EXCLUDE
)

ament_package()
