cmake_minimum_required(VERSION 3.5.0)
project(JoyBot VERSION 0.1.0 LANGUAGES C CXX)

add_executable(JoyBot 
    main.cpp
    PupaClass/gpioLib.cpp
    PupaClass/pwmLib.cpp
    # src/mailbox/mailbox.c
    # src/memory/aligned_mem.c
    # src/memory/map_peripheral.c
    # src/memory/uncached_mem.c
    # src/utility/get_pi_version.c
    # src/main/dma_pwm.c
    # src/gpio/gpio.h
)

target_include_directories(JoyBot PUBLIC
    src/gpio
    src/mailbox
    src/memory
    src/utility
    src/main
)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
