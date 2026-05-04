set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(Vortex
  main.cpp
  radar_algo.cpp
  ${VORTEX_CORE_SOURCES}
)

target_include_directories(Vortex PRIVATE ${VORTEX_CORE_INCLUDE_DIRS})
