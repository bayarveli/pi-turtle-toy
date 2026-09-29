cmake_minimum_required(VERSION 3.5.0)
project(JoyBot VERSION 0.1.0 LANGUAGES C CXX)

add_executable(gpiod_pin_test
    ${CMAKE_SOURCE_DIR}/../hal/gpiod_pin.cpp
    test_gpiod_pin.cpp
)

target_link_libraries(gpiod_pin_test PRIVATE gpiodcxx)
target_include_directories(gpiod_pin_test PRIVATE ${CMAKE_SOURCE_DIR}/../include)

add_executable(libgpiod_cpp_wrapper_test
    test_libgpiod_cpp_wrapper.cpp
)

target_link_libraries(libgpiod_cpp_wrapper_test PRIVATE gpiodcxx)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
