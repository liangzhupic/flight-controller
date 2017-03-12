include_directories(Inc ../../Inc 
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)
add_library( HAL
Src/stm32f4xx_hal_adc.c       Src/stm32f4xx_hal_flash_ramfunc.c  Src/stm32f4xx_hal_rcc.c
Src/stm32f4xx_hal_adc_ex.c    Src/stm32f4xx_hal_gpio.c           Src/stm32f4xx_hal_rcc_ex.c
Src/stm32f4xx_hal.c                      Src/stm32f4xx_hal_spi.c
Src/stm32f4xx_hal_cortex.c             Src/stm32f4xx_hal_tim.c
Src/stm32f4xx_hal_dma.c       Src/stm32f4xx_hal_pcd.c            Src/stm32f4xx_hal_tim_ex.c
Src/stm32f4xx_hal_dma_ex.c    Src/stm32f4xx_hal_pcd_ex.c         Src/stm32f4xx_hal_uart.c
Src/stm32f4xx_hal_flash.c     Src/stm32f4xx_hal_pwr.c            Src/stm32f4xx_ll_usb.c
Src/stm32f4xx_hal_flash_ex.c  Src/stm32f4xx_hal_pwr_ex.c      Src/stm32f4xx_hal_i2c_ex.c
Src/stm32f4xx_hal_i2c.c

)


