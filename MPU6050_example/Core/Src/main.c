/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * Author: Anastasis Vagenas -> Contact: anasvag29@gmail.com
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  flag = 0;
  DWT->CYCCNT = 0;  // Counter

  // Reset of all peripherals, Initializes the Flash interface and the Systick. //
  HAL_Init();

  // Configure the system clock //
  SystemClock_Config();

  // Core Counter initialization //
  // Used to count cycles/performance //
  MX_Core_Counter_Init();

  // Initialize all configured peripherals //
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  // Main Data structure //
  MPU6050_t MPU6050;

  MPU6050_peripheral_config();
  uint8_t check = MPU6050_Init(MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250_deg, MPU6050_DataRate_1KHz);

  // Check if INIT was successful //
  if(check == MPU6050_SUCCESS)
    printf("MPU6050 Initialization Success\n");

  // Set power mode //
  MPU6050_set_power_mode(MPU6050_CYCLE, MPU6050_low_power_1_25Hz);

  // Enable interrupts //
  MPU6050_enable_irq(LATCH_IRQ_EN | IRQ_RD_CLEAR);

  /* Infinite loop */
  while (1){

    if(flag == 1){

      MPU6050_Read_Accel(&MPU6050);
      printf("Accel Data Read\n");
      printf("Accel x is %.2f | Accel y is %.2f | Accel z is %.2f\n\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);

      MPU6050_Read_Gyro(&MPU6050);
      printf("Gyro Data Read\n");
      printf("Gyro x is %.2f | Gyro y is %.2f | Gyro z is %.2f\n\n", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);

      printf("All Data Read\n");
      MPU6050_Read_All(&MPU6050);
      printf("Accel x is %.2f | Accel y is %.2f | Accel z is %.2f\n", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
      printf("Gyro x is %.2f | Gyro y is %.2f | Gyro z is %.2f\n\n", MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);

      flag = 0;
    }

    // Button Press //
    if(flag == 2){

      // Toggle the interrupts //
      if(mpu6050_config.irq_enable == IRQ_ENABLE)
        MPU6050_disable_irq();
      else
        MPU6050_enable_irq(LATCH_IRQ_EN | IRQ_RD_CLEAR);
    }
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
	// Set the FLASH Latency //
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2);

  // Power //
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);

  // HSI enable and calibration //
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

  // Wait till HSI is ready //
  while(LL_RCC_HSI_IsReady() != 1);

  // Set the PLL parameters //
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();

  // Wait till PLL is ready //
  while(LL_RCC_PLL_IsReady() != 1);

  // Set Clock prescalers for the AHBP, APB1, APB2 busses and the system clock source //
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  // Wait till System clock is ready //
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  // Set the correct system core clock value //
  LL_SetSystemCoreClock(84000000);

  // Update the time base //
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
    while(1);

  // Set timer clock prescalers //
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief USART2 Initialization Function
  * UART parameters
  * * PARITY -> NONE
  * * TRANSFER DIRECTION -> TX_ONLY
  * * DATA_WIDTH -> 8-BIT
  * * STOP BITS -> 1-BIT
  * * HARDWARE FLOW CONTROL -> DISABLED
  * * BAUD RATE -> 230400
  *
  */
void MX_USART2_UART_Init(void)
{
	// Enable USART2 Clock //
  __HAL_RCC_USART2_CLK_ENABLE();

  // Assign the handle to USART2
  usart_handle = USART2;

  // Baud Rate Calculation //
  // If clock frequency is not properly defined yet //
  uint32_t pclk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos];
  usart_handle->BRR = UART_BRR_SAMPLING16(pclk, 230400);

  // When Freq is set for the app //
  // Set the BRR (Baud Rate register) directly //
  //	usart_handle->BRR = 0xb6;

  // Control Register 2 //
  // Set TE (Transmitter Enable) Bit //
  // We use USART only for asynchronous transmitting //
  usart_handle->CR1 = 0x0008;

  /* UART Pin config */
  LL_GPIO_SetPinSpeed(USART_TX_GPIO_Port, USART_TX_Pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(USART_TX_GPIO_Port, USART_TX_Pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USART_TX_GPIO_Port, USART_TX_Pin, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(USART_TX_GPIO_Port, USART_TX_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(USART_TX_GPIO_Port, USART_TX_Pin, LL_GPIO_AF_7);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable //
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /* Reset LED Pin*/
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /* LED2 Pin config*/
  LL_GPIO_SetPinSpeed(LD2_GPIO_Port, LD2_Pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(LD2_GPIO_Port, LD2_Pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(LD2_GPIO_Port, LD2_Pin, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(LD2_GPIO_Port, LD2_Pin, LL_GPIO_MODE_OUTPUT);

  /*Configure GPIO pin : PC13 - USER BUTTON */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);

  // MPU6050 Int pin //
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

  // Enable both interrupts //
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief Core DWT Initialization Function
  * @param None
  * @retval None
  *
  * Used for counting execution time of functions and routines.
  */
void MX_Core_Counter_Init(void)
{
  unsigned int *DWT_CYCCNT   = (unsigned int *) 0xE0001004; //address of the register
  unsigned int *DWT_CONTROL  = (unsigned int *) 0xE0001000; //address of the register
  unsigned int *DWT_LAR      = (unsigned int *) 0xE0001FB0; //address of the register
  unsigned int *SCB_DEMCR    = (unsigned int *) 0xE000EDFC; //address of the register

  // ??? Helps though //
  *DWT_LAR = 0xC5ACCE55; // unlock (CM7)

  // Enable access //
  *SCB_DEMCR |= 0x01000000;

  // Reset the counter //
  *DWT_CYCCNT = 0;

  // Enable the counter //
  *DWT_CONTROL |= 1 ; //

}

/**
  * @brief EXTI Line interrupt handler
  * @param None
  * @retval None
  *
  * Handles interrupts from EXTI 10 to 15 (GPIOs).
  * Right now only the user push button is active.
  */
void EXTI15_10_IRQHandler(void)
{

  // Clear the pending interrupt //
  // Button Pressed //
  if(EXTI->PR & GPIO_PIN_13) {
    EXTI->PR = GPIO_PIN_13; // Clear //
    flag = 2;
  }

  // Clear the pending interrupt //
  //   MPU6050 Int pin //
  if(EXTI->PR & GPIO_PIN_15) {

    EXTI->PR = GPIO_PIN_15; // Clear //
    flag = 1;
  }

}
