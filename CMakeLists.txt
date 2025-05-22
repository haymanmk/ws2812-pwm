# @file CMakeLists.txt
# @brief CMake configuration for the WS2812 driver.
#
# The source files are included in the build
# if the CONFIG_WS2812_PWM option is enabled.

if (CONFIG_WS2812_PWM)
    file(GLOB_RECURSE src_files src/*.c)
    zephyr_library_sources(src_files)
    zephyr_include_directories(src/include)
endif()