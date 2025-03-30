cmake_minimum_required(VERSION3.5)
project(robo_entrega) find_package(ament_cmake
REQUIRED)
ament_package()

install(PROGRAMS
  scripts/load_map_client.py
  DESTINATION lib/${PROJECT_NAME}
)
