cmake_minimum_required(VERSION 3.5)  # ✅ Espacio después de VERSION
project(robo_entrega)

find_package(ament_cmake REQUIRED)
ament_package()

install(PROGRAMS
  ../provide_map/provide_map/load_map_client.py
  DESTINATION lib/${PROJECT_NAME}
)