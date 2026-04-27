# Tests
enable_testing()

set(TEST_SOURCES
  vortex_test_ring_buffer.cpp
  vortex_test_double_buffer_spsc.cpp
  vortex_test_iir_filter.cpp
  vortex_test_lp_butterworth.cpp
  vortex_test_linear_system.cpp
)

foreach(src ${TEST_SOURCES})
  get_filename_component(test_name ${src} NAME_WE)
  add_executable(${test_name} ${src})
  target_include_directories(${test_name}
    PRIVATE
      ${CMAKE_SOURCE_DIR}/lib
      ${CMAKE_SOURCE_DIR}/_deps/eigen
  )
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
