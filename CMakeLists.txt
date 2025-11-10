cmake_minimum_required(VERSION 3.13)
if (NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

# Check if this is being built standalone or as part of a larger project
if(NOT TARGET pico_stdlib)
    # Standalone build - need to initialize Pico SDK

    if(NOT DEFINED ENV{PICO_SDK_PATH})
        message(FATAL_ERROR "PICO_SDK_PATH environment variable not set.\n"
                "Please set it to your Pico SDK installation directory.\n"
                "Example: export PICO_SDK_PATH=/path/to/pico-sdk")
    endif()

    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

    project(pico2_swd_riscv C CXX ASM)

    pico_sdk_init()
else()
    # Being built as part of a larger project - SDK already initialized
    message(STATUS "pico2-swd-riscv: Building as subproject")
endif()


# Library metadata
set(PICO2_SWD_RISCV_VERSION_MAJOR 1)
set(PICO2_SWD_RISCV_VERSION_MINOR 0)
set(PICO2_SWD_RISCV_VERSION_PATCH 0)

# pico2-swd-riscv library
add_library(pico2_swd_riscv STATIC
    src/swd.c
    src/swd_protocol.c
    src/dap.c
    src/rp2350.c
)

# Generate PIO header from assembly
pico_generate_pio_header(pico2_swd_riscv ${CMAKE_CURRENT_LIST_DIR}/src/swd.pio)

# Public include directory
target_include_directories(pico2_swd_riscv PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link required Pico SDK libraries
target_link_libraries(pico2_swd_riscv PUBLIC
    pico_stdlib
    hardware_pio
    hardware_gpio
    hardware_clocks
)

# Configurable debug level (default: warnings only)
if(NOT DEFINED PICO2_SWD_DEBUG_LEVEL)
    set(PICO2_SWD_DEBUG_LEVEL 1 CACHE STRING "Debug level: 0=none, 1=warn, 2=info, 3=debug")
endif()

target_compile_definitions(pico2_swd_riscv PUBLIC
    PICO2_SWD_DEBUG_LEVEL=${PICO2_SWD_DEBUG_LEVEL}
)

# Compiler warnings
target_compile_options(pico2_swd_riscv PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wunused-function          # Warn about unused static functions
    -Wunused-variable          # Warn about unused variables
    -Wunused-but-set-variable  # Warn about set but unused variables
    -Wunreachable-code         # Warn about code that will never execute
    -fdata-sections            # Place each data item in separate section
    -ffunction-sections        # Place each function in separate section
)

# Linker options for dead code elimination
target_link_options(pico2_swd_riscv PRIVATE
    -Wl,--gc-sections          # Remove unused sections at link time
    -Wl,--print-gc-sections    # Print what sections are being removed (optional)
)

# Optional: Build examples if requested
option(PICO2_SWD_BUILD_EXAMPLES "Build example programs" OFF)
if(PICO2_SWD_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
