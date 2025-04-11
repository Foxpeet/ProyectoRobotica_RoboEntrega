cmake_minimum_required(VERSION 3.5)
project(robo_entrega)

find_package(ament_cmake REQUIRED)
ament_package()

install(PROGRAMS
  ../robo_entrega_provide_map/robo_entrega_provide_map/load_map_client.py
  DESTINATION lib/${PROJECT_NAME}
)