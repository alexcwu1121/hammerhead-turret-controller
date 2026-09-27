#include "bsp.hpp"

// Interrupt callbacks
extern "C"
{
    /// @brief QP assertion handler
    /// @param module
    /// @param id
    /// @return
    Q_NORETURN Q_onError(char const* const module, int_t const id)
    {
        // NOTE: this implementation of the error handler is intended only
        // for debugging and MUST be changed for deployment of the application
        // (assuming that you ship your production code with assertions enabled).
        Q_UNUSED_PAR(module);
        Q_UNUSED_PAR(id);
        QS_ASSERTION(module, id, 10000U);  // report assertion to QS

        // Reset
        /// TODO: Enable in release only
        // NVIC_SystemReset();
        for (;;) {}
    }

    /// @brief Hardfault handler
    __attribute__((naked)) void HardFault_Handler(void)
    {
        __asm volatile(
            "tst lr, #4        \n"  // Which stack? MSP or PSP
            "ite eq            \n"
            "mrseq r0, msp     \n"
            "mrsne r0, psp     \n"
            "b hardfault_c     \n");
    }

    /// @brief Break out registers
    /// @param stack
    void hardfault_c(uint32_t* stack)
    {
        /// TODO: Consider writing pc and lr to eeprom
        volatile uint32_t r0 = stack[0];
        volatile uint32_t r1 = stack[1];
        volatile uint32_t r2 = stack[2];
        volatile uint32_t r3 = stack[3];
        volatile uint32_t r12 = stack[4];
        volatile uint32_t lr = stack[5];
        volatile uint32_t pc = stack[6];
        volatile uint32_t psr = stack[7];

        (void)r0;
        (void)r1;
        (void)r2;
        (void)r3;
        (void)r12;
        (void)lr;
        (void)pc;
        (void)psr;

        __BKPT(1);
        for (;;) {}
    }

    void SysTick_Handler(void);
    void SysTick_Handler(void)
    {
        QK_ISR_ENTRY();                     // Inform QK about entering an ISR
        HAL_IncTick();                      // Increment global timebase
        QP::QTimeEvt::TICK_X(0U, nullptr);  // Process QP time events at rate 0
        QK_ISR_EXIT();                      // Inform QK about exiting an ISR
    }

    /// @brief QP assertion callback
    /// @param module source file/module of the assert
    /// @param id assert code
    void assert_failed(char const* const module, int_t const id);
    void assert_failed(char const* const module, int_t const id)
    {
        Q_onError(module, id);
    }
}

namespace QP
{
/// @brief QF startup callback
void QF::onStartup()
{
    // Set up the SysTick timer to fire at bsp::TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / bsp::TICKS_PER_SEC);

    // Assign all priority bits for preemption-prio. And none to sub-prio.
    NVIC_SetPriorityGrouping(0U);

    // UART RX interrupt
    HAL_NVIC_SetPriority(USART2_IRQn, 4U, 4U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    // GPIO interrupts
    // These are fault signals and thus are kernel unaware
    // HAL_NVIC_SetPriority(EXTI1_IRQn, 0U, 0U);
    // HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    // HAL_NVIC_SetPriority(EXTI3_IRQn, 0U, 0U);
    // HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/// @brief QF idle callback
void QK::onIdle() {}
}  // namespace QP

/// @brief USART2 interrupt handler
extern "C" void USART2_IRQHandler(void)
{
    QK_ISR_ENTRY();
    HAL_UART_IRQHandler(&huart2);
    QK_ISR_EXIT();
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/// @brief Application error handler callback
// TODO: Get rid of this
void Error_Handler(void)
{
    __disable_irq();
    for (;;) {}
}
