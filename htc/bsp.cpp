#include "bsp.hpp"

#include "tim.h"

/// @brief Flag tracking pan low state
static volatile bool pan_step_low = false;
/// @brief Flag tracking tilt low state
static volatile bool tilt_step_low = false;

// Interrupt callbacks
extern "C"
{
void SysTick_Handler(void)
{
    // Increment hal counter
    HAL_IncTick();
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

//void TIM6_IRQHandler(void)
//{
//    HAL_TIM_IRQHandler(&htim4);
//}

// __HAL_TIM_SET_AUTORELOAD(&htim4, new_period);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        //HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_SET);
        //__NOP(); __NOP(); __NOP();  // small delay (~100 ns each)
        //HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_RESET);
        __NOP();
        __NOP();
        __NOP();  // small delay (~100 ns each)
        HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_SET);
    }
}

// __HAL_TIM_SET_AUTORELOAD(&htim6, new_period);
void TIM6_DAC_IRQHandler(void)
{
    /*
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6))
    {
        LL_TIM_ClearFlag_UPDATE(TIM6);

        HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_SET);
        __NOP(); __NOP(); __NOP();  // small delay (~100 ns each)
        HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_RESET);
    }
    */
}
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
