cmake_minimum_required(VERSION 3.22)

# HAL object library
add_library(hal OBJECT)
target_include_directories(hal
    PUBLIC
        Core/Inc
        Drivers/STM32F3xx_HAL_Driver/Inc
        Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F3xx/Include
        Drivers/CMSIS/Include
)
target_sources(hal
    PRIVATE
        Core/Src/gpio.c
        Core/Src/i2c.c
        Core/Src/spi.c
        Core/Src/stm32f3xx_hal_msp.c
        Core/Src/stm32f3xx_it.c
        Core/Src/syscalls.c
        Core/Src/sysmem.c
        Core/Src/system_stm32f3xx.c
        Core/Src/tim.c
        Core/Src/usart.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
        Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
)
target_compile_definitions(hal
    PUBLIC
    	USE_HAL_DRIVER
	    STM32F302xC
        $<$<CONFIG:Debug>:DEBUG>
)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
