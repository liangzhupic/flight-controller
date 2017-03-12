include_directories(
Core/Inc
Class/CDC/Inc
${CMAKE_SOURCE_DIR}/Inc
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include

${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
)
add_library(USB 

Core/Src/usbd_core.c  Core/Src/usbd_ctlreq.c  Core/Src/usbd_ioreq.c
Class/CDC/Src/usbd_cdc.c
)
