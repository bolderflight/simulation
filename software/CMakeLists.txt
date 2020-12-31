cmake_minimum_required(VERSION 3.13)
# Using the MK66FX1M0 microcontroller
set(MCU MK66FX1M0)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(Flight
	VERSION 0.1.0
	DESCRIPTION "SPAARO"
	LANGUAGES C CXX
)
# Grab the processor and set up definitions and compile options
include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
configMcu(${MCU})
# Define a larger Serial 2 RX buffer
add_definitions(
	-DSERIAL2_RX_BUFFER_SIZE=1024
)
# Define a larger Serial 3 RX buffer
add_definitions(
	-DSERIAL3_RX_BUFFER_SIZE=2048
)
# Define a larger Serial 4 TX buffer
add_definitions(
	-DSERIAL4_TX_BUFFER_SIZE=2048
)
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
	core
	GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/core.git
	GIT_TAG v1.0.3
)
FetchContent_MakeAvailable(core)
FetchContent_Declare(
	sbus
	GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/sbus.git
	GIT_TAG v1.0.3
)
FetchContent_MakeAvailable(sbus)
FetchContent_Declare(
	pwm
	GIT_REPOSITORY 	https://gitlab.com/bolderflight/software/pwm.git
	GIT_TAG v1.0.3
)
FetchContent_MakeAvailable(pwm)
# Add the executable
add_executable(flight
	flight/flight.cc
	flight/print_msg.cc
	flight/inceptor.cc
	flight/airdata.cc
	flight/imu.cc
	flight/gnss.cc
	flight/effector.cc
)
# Add the includes
target_include_directories(flight PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
# Link libraries to the executable
target_link_libraries(flight
	PRIVATE 
		core
		sbus
		pwm
)
# Add hex and upload targets
include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU})
