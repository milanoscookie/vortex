set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(vortex_core STATIC
  fmcw_tracker.cpp
  radar_algo.cpp
  ${VORTEX_CORE_SOURCES}
)
target_include_directories(vortex_core PUBLIC
  ${CMAKE_SOURCE_DIR}/lib
  ${CMAKE_SOURCE_DIR}/src
  ${VORTEX_CORE_INCLUDE_DIRS}
)
target_compile_definitions(vortex_core PUBLIC EIGEN_FFTW_DEFAULT)
target_link_libraries(vortex_core PUBLIC ${VORTEX_CORE_LINK_LIBRARIES})

add_executable(Vortex
  main.cpp
)
target_link_libraries(Vortex PRIVATE vortex_core)
target_include_directories(Vortex PRIVATE ${VORTEX_CORE_INCLUDE_DIRS})
