#top cmake file
#author liangzhu

cmake_minimum_required  (VERSION 2.8)

project (test)
#FIND_PACKAGE(CMSIS REQUIRED)
#FIND_PACKAGE(STM32HAL  REQUIRED)
#ADD_DEFINITIONS(-DCORTEX_USE_FPU=TRUE)
ENABLE_LANGUAGE(ASM)

#add_subdirectory(RawOS)
#add_subdirectory(STM32F4xx_StdPeriph_Driver)
add_subdirectory(Drivers/STM32F4xx_HAL_Driver)
add_subdirectory(Drivers/CMSIS)
add_subdirectory(Middlewares/Third_Party/FreeRTOS/Source)
add_subdirectory(Middlewares/ST/STM32_USB_Device_Library)
include_directories(
#${CMAKE_SOURCE_DIR}/RawOs/Source/include
Inc
#${CMSIS_INCLUDE_DIRS}
 #   ${STM32HAL_INCLUDE_DIR}
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include

${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

#mavlink v2
#${CMAKE_SOURCE_DIR}/Mavlink
#${CMAKE_SOURCE_DIR}/Mavlink/common

#mavlink v1
${CMAKE_SOURCE_DIR}/mavlink-v1
${CMAKE_SOURCE_DIR}/mavlink-v1/common
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0  ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0  ")
set (main
Src/adc.c       Src/main.c    Src/stm32f4xx_hal_msp.c           Src/tim.c
Src/freertos.c    Src/stm32f4xx_hal_timebase_TIM.c  Src/usart.c
Src/gpio.c      Src/spi.c     Src/stm32f4xx_it.c                Src/dma.c
Src/usbd_cdc_if.c   Src/usbd_conf.c   Src/usbd_desc.c           Src/usb_device.c         
Src/syscalls.c  Src/mpu9250.c  Src/mavlink.c    Src/i2c.c       Src/ppm_capture.c
Src/ahrs.c      Src/attitude_controll.c           Src/pid.c      Src/esc.c
Src/ultrasonic.c
)

#set(source rtos )
#message(${STM32HAL_SOURCES})
add_executable(${CMAKE_PROJECT_NAME}.elf ${main}  )
#${CMSIS_SOURCES} ${STM32HAL_SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME}.elf HAL CMSIS freertos USB )

# CMSIS HAL rtos

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME}.elf)
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME}.elf)
STM32_PRINT_SIZE_OF_TARGETS(${CMAKE_PROJECT_NAME}.elf)

