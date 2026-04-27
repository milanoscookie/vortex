add_executable(Vortex
  main.cpp
  ${VORTEX_CORE_SOURCES}
)

target_include_directories(Vortex PRIVATE ${VORTEX_CORE_INCLUDE_DIRS})
target_link_libraries(Vortex PRIVATE OpenMP::OpenMP_CXX)
