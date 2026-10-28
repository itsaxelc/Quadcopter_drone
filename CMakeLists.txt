cmake_minimum_required(VERSION 3.22)

# ============================================================
# Compiler settings
# ============================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME Quadcopter_drone)

include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})

message("Build type: ${CMAKE_BUILD_TYPE}")

# Mask warnings by uncommenting this line
# add_compile_options(-w)

# ============================================================
# Executable
# ============================================================

add_executable(${CMAKE_PROJECT_NAME})

# ============================================================
# Source files
# ============================================================

file(GLOB_RECURSE CORE_SOURCES
    Core/Src/*.c
    Core/Src/*.s
    Core/Src/*.S
)

file(GLOB_RECURSE INA219_SOURCES
    Drivers/INA219/*.c
)

file(GLOB_RECURSE MPU9250_SOURCES
    Drivers/MPU9250/*.c
)

file(GLOB_RECURSE I2C_SOURCES
    Drivers/I2C/*.c
)

file(GLOB_RECURSE HAL_SOURCES
    Drivers/STM32L4xx_HAL_Driver/*.c
)

file(GLOB_RECURSE FREERTOS_SOURCES
    FreeRTOS-KernelV11.2.0/tasks.c
    FreeRTOS-KernelV11.2.0/queue.c
    FreeRTOS-KernelV11.2.0/list.c
    FreeRTOS-KernelV11.2.0/portable/GCC/ARM_CM4F/port.c
    FreeRTOS-KernelV11.2.0/portable/MemMang/heap_4.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    # Application / BSP
    ${CORE_SOURCES}

    # Drivers
    ${INA219_SOURCES}
    ${MPU9250_SOURCES}
    ${HAL_SOURCES}
    ${I2C_SOURCES}

    # FreeRTOS
    ${FREERTOS_SOURCES}

    # Startup
    startup_stm32l476xx.s
)

# ============================================================
# Include directories
# ============================================================

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Application
    Core/Inc

    # CMSIS
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32L4xx/Include

    # STM32 HAL
    Drivers/STM32L4xx_HAL_Driver/Inc
    Drivers/STM32L4xx_HAL_Driver/Inc/Legacy

    # Sensors
    Drivers/INA219
    Drivers/MPU9250
    Drivers/I2C

    # FreeRTOS
    FreeRTOS-KernelV11.2.0/include
    FreeRTOS-KernelV11.2.0/portable/GCC/ARM_CM4F
)

# ============================================================
# Preprocessor definitions
# ============================================================

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    STM32L476xx

    USE_HAL_DRIVER

    $<$<CONFIG:Debug>:DEBUG>
)

# ============================================================
# Linker
# ============================================================

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE

    -T${CMAKE_SOURCE_DIR}/STM32L476RGTx_FLASH.ld

    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

    -Wl,--gc-sections

    -Wl,-Map=${CMAKE_PROJECT_NAME}.map

    --specs=nano.specs
    --specs=nosys.specs
)

# ============================================================
# Compiler options
# ============================================================

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE

    -mcpu=cortex-m4
    -mthumb

    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16

    -ffunction-sections
    -fdata-sections

    -Wall
    -Wextra
)

# ============================================================
# Libraries
# ============================================================

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    m
)