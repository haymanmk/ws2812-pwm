# WS2812-PWM
## Overview
ws2812-pwm is a zephyr custom driver used to control ws2812-based LED strip via PWM. This driver also offloads the CPU by DMA which transfers and feeds the pulse width data from a memory buffer to the timer's register.
## Get Started

> Note:
> When using this driver, `cmake` might show the following warning message. The reason responsible for this warning is that the `CONFIG_LED_STRIP` is enabled, but none of the built-in drivers located in `drivers/led_strip` is chosen by `CONFIG_<DRIVER_NAME>`. That is because this driver is a separate module and is not placed under the source tree of led strip driver. If you are interesting about this, you can refer to the `CMakeLists.txt` in `drivers/led_strip` for the details.
  
  ```sh
  CMake Warning at /home/hayman/zephyrproject/zephyr/CMakeLists.txt:1022 (message):
    No SOURCES given to Zephyr library: drivers__led_strip
  
    Excluding target from build.
  ```
