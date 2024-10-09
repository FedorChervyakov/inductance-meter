cmake_minimum_required(VERSION 3.22)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(CMAKE_SYSTEM_NAME "Generic")
set(CMAKE_GENERATOR "Unix Makefiles")
project(attiny202_L_meter VERSION 1.0 LANGUAGES C) # Set the project name

set(CMAKE_BUILD_TYPE Prod)

# Default to Debug build
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug)
endif()

# lsp support
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

# MCU configuration
# Set MCU type, clock frequency and programmer
set(MCU "attiny202")
set(CLOCK_FREQ "8000000")
set(PROG_STR serialupdi)
set(ARCH "avrxmega3")


# C compiler flags
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)
set(GCC_FLAGS "-Wall -Wextra -DF_CPU=${CLOCK_FREQ} -mmcu=${MCU}") # Add -Werror
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_FLAGS}")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    add_compile_options(-O0 -g)
else ()
    add_compile_options(-O1)
endif ()


file(GLOB_RECURSE SOURCES
	"src/main.c"
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE inc)

set_property(TARGET ${PROJECT_NAME}.elf
	APPEND
	PROPERTY ADDITIONAL_CLEAN_FILES ${PROJECT_NAME}.hex ${PROJECT_NAME}.bin
)

add_custom_command(TARGET ${PROJECT_NAME}.elf
	POST_BUILD
	COMMAND avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
	COMMAND avr-objcopy -j .text -j .data -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)

add_custom_target(flash
	DEPENDS ${PROJECT_NAME}.elf
    COMMAND pymcuprog write --erase -t uart -u /dev/ttyUSB0 -d ${MCU} --uart-timeout 2 -f ${PROJECT_NAME}.hex

	COMMENT "flashes the hex file onto the MCU"
)

add_custom_target(flash-debug
	DEPENDS ${PROJECT_NAME}.elf
	COMMAND avrdude -c ${PROG_STR} -p ${MCU} -U flash:w:${PROJECT_NAME}.elf:e
	COMMENT "flashes the elf file onto the MCU"
)

add_custom_target(read_fuses
    COMMAND avrdude -c ${PROG_STR} -p ${MCU} -U lfuse:r:-:i -v
    COMMENT "read fuses from the MCU"
)
