cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

add_executable(main
		main.cpp
		lib/hardware/compass_classes.cpp
		lib/hardware/motor.cpp
		lib/hardware/kicker.cpp
		lib/hardware/USB_Serial.cpp
		lib/software/pid.cpp
		lib/software/bitmask.cpp
		lib/software/hex.cpp
		pico-lib/definitions/gpio.cpp
		pico-lib/definitions/i2c.cpp
		pico-lib/definitions/serial.cpp
		pico-lib/definitions/time.cpp
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(main pico_stdlib)
target_link_libraries(main pico_multicore)
target_link_libraries(main pico_stdio_usb)
target_link_libraries(main hardware_gpio)
target_link_libraries(main hardware_adc)
target_link_libraries(main hardware_pwm)
target_link_libraries(main hardware_uart)
target_link_libraries(main hardware_spi)
target_link_libraries(main hardware_i2c)
target_link_libraries(main hardware_irq)
target_link_libraries(main hardware_timer)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
