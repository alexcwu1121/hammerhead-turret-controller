#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp.hpp"
#include "stm32f3xx_it.h"

// electrical todo
// adjust vref of stepper drivers either with DAC or potentiometer
// add mounting support holes near power connector
// somehow add bigger uart debug pads
// Consider allowing PWMing of motor step pin
// Add testpoints
// Add 2 fan controllers
// Add temperature sensor

#include "bmi270.hpp"

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  HAL_GPIO_WritePin(PAN_RESET_GPIO_Port, PAN_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAN_MS1_GPIO_Port, PAN_MS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAN_MS2_GPIO_Port, PAN_MS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PAN_MS3_GPIO_Port, PAN_MS3_Pin, GPIO_PIN_RESET);
 
  HAL_GPIO_WritePin(TILT_RESET_GPIO_Port, TILT_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_DIR_GPIO_Port, TILT_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TILT_MS1_GPIO_Port, TILT_MS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TILT_MS2_GPIO_Port, TILT_MS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TILT_MS3_GPIO_Port, TILT_MS3_Pin, GPIO_PIN_RESET);

  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  //HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
  //HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  HAL_TIM_Base_Start_IT(&htim4);
  //HAL_TIM_Base_Start_IT(&htim6);

  __HAL_TIM_SET_AUTORELOAD(&htim4, 100);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Initialize IMU
  imu::BMI270 _imu(SPI_CS_GPIO_Port, SPI_CS_Pin, &hspi3);

  // Verify IMU device ID and enable SPI peripheral
  volatile imu::Fault fault = _imu.Initialize();
  fault = _imu.Initialize();

  // Set accelerometer ODR
  fault = _imu.SetAccODR(imu::BMI270::AccODR::ODR_800);

  // Set accelerometer range
  fault = _imu.SetAccRange(imu::BMI270::AccRange::G_4);

  // Set gyroscope ODR
  fault = _imu.SetGyrODR(imu::BMI270::GyrODR::ODR_1K6);

  // Set gyroscope range
  fault = _imu.SetGyrRange(imu::BMI270::GyrRange::DPS_500);

  fault = _imu.RunCompensation();

  imu::IMUData imu_data;
  while (1)
  {
    //volatile uint32_t pan_pos = __HAL_TIM_GET_COUNTER(&htim2);
    //volatile uint32_t tilt_pos = __HAL_TIM_GET_COUNTER(&htim3);
    
    //imu::Fault fault = _imu.ReadData(imu_data);
    //HAL_Delay(1);
    //HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_RESET);
    //HAL_Delay(1);
    //HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_SET);
  }
}
