find_package(PkgConfig REQUIRED)
pkg_check_modules(VORTEX_FFTW3 REQUIRED fftw3)
pkg_check_modules(VORTEX_FFTW3F REQUIRED fftw3f)

set(VORTEX_CORE_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/_deps/eigen
  ${VORTEX_FFTW3_INCLUDE_DIRS}
  ${VORTEX_FFTW3F_INCLUDE_DIRS}
)

set(VORTEX_CORE_LINK_LIBRARIES
  ${VORTEX_FFTW3_LIBRARIES}
  ${VORTEX_FFTW3F_LIBRARIES}
)

add_library(vortex_lib_deps INTERFACE)
target_include_directories(vortex_lib_deps
  INTERFACE
    ${VORTEX_CORE_INCLUDE_DIRS}
)
target_link_libraries(vortex_lib_deps
  INTERFACE
    ${VORTEX_CORE_LINK_LIBRARIES}
)

add_library(vortex_lib_obj OBJECT
  dynamics.cpp
  environment.cpp
  simulation.cpp
)
target_link_libraries(vortex_lib_obj PUBLIC vortex_lib_deps)
target_include_directories(vortex_lib_obj PUBLIC ${VORTEX_CORE_INCLUDE_DIRS})

set(VORTEX_CORE_SOURCES
  $<TARGET_OBJECTS:vortex_lib_obj>
  PARENT_SCOPE
)

set(VORTEX_CORE_INCLUDE_DIRS ${VORTEX_CORE_INCLUDE_DIRS} PARENT_SCOPE)

set(VORTEX_CORE_LINK_LIBRARIES ${VORTEX_CORE_LINK_LIBRARIES} PARENT_SCOPE)
