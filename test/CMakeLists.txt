cmake_minimum_required(VERSION 3.5.0)
project(JoyBot VERSION 0.1.0 LANGUAGES C CXX)

add_executable(test_gpiod_pin
    ${CMAKE_SOURCE_DIR}/../hal/gpiod_pin.cpp
    test_gpiod_pin.cpp
)

target_link_libraries(test_gpiod_pin PRIVATE gpiodcxx)
target_include_directories(test_gpiod_pin PRIVATE ${CMAKE_SOURCE_DIR}/../include)

add_executable(test_gpiodcxx_pulse_output
    test_gpiodcxx_pulse_output.cpp
)

target_link_libraries(test_gpiodcxx_pulse_output PRIVATE gpiodcxx)

add_executable(test_gpiodcxx_pulse_input
    test_gpiodcxx_pulse_input.cpp
)

target_link_libraries(test_gpiodcxx_pulse_input PRIVATE gpiodcxx)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
