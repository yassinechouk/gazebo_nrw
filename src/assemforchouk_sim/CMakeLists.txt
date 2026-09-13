cmake_minimum_required(VERSION 3.16)
project(assemforchouk_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY urdf meshes config worlds launch rviz
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
          scripts/cycle_demo.py
          scripts/spawn_payload.py
          scripts/jog.py
        DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

ament_package()
