add_library(drivers_board
	init.c
	led.c
	usb.c
	spi_init.c
	spi.cpp
	i2c.cpp
	timer_config.cpp
)
target_include_directories(drivers_board PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(drivers_board PRIVATE px4_layer nuttx_arch)
