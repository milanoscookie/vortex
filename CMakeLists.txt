cmake_minimum_required(VERSION 3.12)
project(VORTEX CXX)

option(VORTEX_ENABLE_PGO_GENERATE "Build with profile generation enabled" OFF)
option(VORTEX_ENABLE_PGO_USE "Build with profile use enabled" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    "$<$<CONFIG:Release>:-O3>"
    "$<$<CONFIG:Release>:-march=native>"
    "$<$<CONFIG:Release>:-mtune=native>"
    "$<$<CONFIG:Release>:-ffast-math>"
    "$<$<CONFIG:Release>:-fno-math-errno>"
    "$<$<CONFIG:Release>:-fomit-frame-pointer>"
    "$<$<CONFIG:RelWithDebInfo>:-O3>"
    "$<$<CONFIG:RelWithDebInfo>:-march=native>"
    "$<$<CONFIG:RelWithDebInfo>:-mtune=native>"
    "$<$<CONFIG:RelWithDebInfo>:-ffast-math>"
    "$<$<CONFIG:RelWithDebInfo>:-fno-math-errno>"
    "$<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>"
  )

  if(VORTEX_ENABLE_PGO_GENERATE)
    add_compile_options(-fprofile-generate)
    add_link_options(-fprofile-generate)
  endif()

  if(VORTEX_ENABLE_PGO_USE)
    add_compile_options(-fprofile-use -fprofile-correction)
    add_link_options(-fprofile-use -fprofile-correction)
  endif()
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT vortex_ipo_supported)
if(vortex_ipo_supported)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
endif()

# Enable CTest at the top level so tests are discoverable from build/
enable_testing()

# Build the sensor interface library (independent)
add_subdirectory(lib)

# Build the executable (depends on library)
add_subdirectory(src)

# Tests
add_subdirectory(tests)
