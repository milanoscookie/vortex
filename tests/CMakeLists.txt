# Tests
enable_testing()

set(TEST_SOURCES
  vortex_test_ring_buffer.cpp
  vortex_test_double_buffer_spsc.cpp
  vortex_test_iir_filter.cpp
  vortex_test_lp_butterworth.cpp
  vortex_test_hp_butterworth.cpp
  vortex_test_linear_system.cpp
  vortex_test_fmcw_tracker.cpp
  vortex_test_simulation.cpp
)

foreach(src ${TEST_SOURCES})
  get_filename_component(test_name ${src} NAME_WE)
  add_executable(${test_name} ${src})
  target_include_directories(${test_name}
    PRIVATE
      ${CMAKE_SOURCE_DIR}/lib
      ${CMAKE_SOURCE_DIR}/src
      ${CMAKE_SOURCE_DIR}/_deps/eigen
      ${VORTEX_CORE_INCLUDE_DIRS}
  )
  target_compile_definitions(${test_name} PRIVATE EIGEN_FFTW_DEFAULT)
  target_link_libraries(${test_name} PRIVATE ${VORTEX_CORE_LINK_LIBRARIES})
  if(test_name STREQUAL "vortex_test_fmcw_tracker" OR test_name STREQUAL "vortex_test_simulation")
    target_link_libraries(${test_name} PRIVATE vortex_core)
  endif()
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
