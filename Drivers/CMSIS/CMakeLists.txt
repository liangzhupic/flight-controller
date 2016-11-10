include_directories(
Include
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
 )
message(${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
add_library( CMSIS
Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
)
