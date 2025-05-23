# @file CMakeLists.txt
# @brief CMake configuration for the WS2812 driver.
#
# The source files are included in the build
# if the CONFIG_WS2812_PWM option is enabled.

# Subdirectories
add_subdirectory(drivers)

# Include headers
zephyr_include_directories(drivers)