cmake_minimum_required(VERSION 3.12)
project(VORTEX CXX)

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
    "$<$<CONFIG:RelWithDebInfo>:-fomit-frame-pointer>"
  )
endif()

# Enable CTest at the top level so tests are discoverable from build/
enable_testing()

# Build the sensor interface library (independent)
add_subdirectory(lib)

# Build the executable (depends on library)
add_subdirectory(src)

# Tests
add_subdirectory(tests)
