/**
  ******************************************************************************
  * @file      : mpu6050.c
  * @brief     : Source code for the MPU6050 driver
  ******************************************************************************
  * Author: Anastasis Vagenas -> Contact: anasvag29@gmail.com
  *
  * This driver expands upon the previous implementations from:
  * -> https://github.com/sinadarvi/SD_HAL_MPU6050
  * -> https://github.com/leech001/MPU6050
  * ! Check them out !
  *
  * This Library has routines in order to communicate with MPU6050 sensor either via HAL
  * or Direct access (using LL or Direct access to registers).
  * For the I2C Driver user can choose either HAL or our own implementation in the i2c_driver.h
  *
  * ------- ROUTINES -------
  * I2C Routines:
  * - i2c_MPU6050_write_reg() -> Writes to the MPU6050 I2C bus
  * - i2c_MPU6050_read_reg() -> Reads from the MPU6050 using the I2C bus
  *
  * Accelerometer/Gyroscope/Temp Data Routines:
  * - MPU6050_Read_Accel() -> Reads the accelerometer data
  * - MPU6050_Read_Gyro() -> Reads the gyroscope data
  * - MPU6050_Read_Temp() -> Reads the temperature data
  * - MPU6050_Read_All() -> Reads all of the above data
  *
  * Configuration Routines:
  * - accel_sensitiviy_config() -> Finds the sensitivity range for the accelerometer
  * - gyro_sensitiviy_config() -> Finds the sensitivity range for the gyroscope
  * - MPU6050_set_power_mode() -> Sets the power profile based on the user's choice
  * - MPU6050_Init() -> Initializes the sensor with default configuration
  * - MPU6050_peripheral_config() -> Initializes the I2C peripheral and the GPIOs used
  *
  * Interrupt Routines:
  * - MPU6050_enable_irq()-> Enables IRQs generation on the MPU6050 based on user configuration choice
  * - MPU6050_disable_irq()-> Disables IRQs generation on the MPU6050
  * - MPU6050_get_irq_status()-> Reads IRQ status register on the MPU6050
  *
  * Extra Routines:
  * - Kalman_getAngle() -> Sub-routine used to calculate Kalman angle
  * - MPU6050_self_test() -> Sub-routine that performs the self-test routine described in the MPU6050 manual
  *
  * ------- MAIN DATA STRUCTURES -------
  * MPU6050_t -> Holds measurements for all sensors and Kalman angle values
  * MPU6050_configuration -> Holds the current configuration of the MPU6050
  *
  * ------- USAGE -------
  * In order to simply use the driver, set:
  * An I2C handle based on the header file naming used
  * Set the Defines for the GPIOs used (if they are a different board)
  *
  * The call:
  * 1.MPU6050_peripheral_config() -> Initialize the peripherals
  * 2.MPU6050_Init() -> Initialize the MPU6050 with the basic configuration
  * 3.Use on the Data Routines (MPU6050_Read_All()) to read data from the MPU6050.
  *
  * In order to set the power mode, use MPU6050_set_power_mode(), with the power profile you want:
  *     MPU6050_ACCEL_ONLY  : Accelerometer only mode
  *     MPU6050_GYRO_ONLY   : Gyroscope only mode
  *     MPU6050_CYCLE,      : Cycle Mode - Sample at a fixed rate
  *     MPU6050_SLEEP,      : Sleep mode - Everything Disabled
  *     MPU6050_POWER_ON,   : Power ON - Everything ON
  *
  * In order to use interrupts, use the appropriate routine for enabling or disabling.
  * @ Note -> User has to configure an EXTII line appropriate GPIO if he want to use
  * the INT PIN on the MPU6050
  *
**/
#include "mpu6050.h"

// Setup MPU6050 //
uint32_t timer;

Kalman_t KalmanX = {
  .Q_angle = 0.001f,
  .Q_bias = 0.003f,
  .R_measure = 0.03f
};

Kalman_t KalmanY = {
  .Q_angle = 0.001f,
  .Q_bias = 0.003f,
  .R_measure = 0.03f,
};

/* -- i2c_MPU6050_write_reg() --
 * Input: Command
 * Return: None
 * Description:
 *
 * Internal function of the MPU6050 driver, that uses I2C to write data to the I2C bus.
 *
 * */
static void i2c_MPU6050_write_reg(uint8_t *data, uint8_t num_of_data)
{
  // HAL Driver Usage //
  #ifdef MPU6050_USE_HAL
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDR, data[0], 1, data + 1, 1, HAL_I2C_TIMEOUT);
  #endif

  #ifndef MPU6050_USE_HAL
    i2c_write_data(i2c_handle, MPU6050_WRITE_ADDR, data, 2);
  #endif
}

/* -- i2c_MPU6050_read_reg() --
 * Input: Command
 * Return: None
 * Description:
 *
 * Internal function of the MPU6050 driver, that uses I2C to write a command to the I2C bus.
 * Follows Mem_Read format.
 *
 * */
static void i2c_MPU6050_read_reg(uint8_t command, uint8_t *data, uint8_t num_of_data)
{
  #ifdef MPU6050_USE_HAL
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_WRITE_ADDR, command, 1, data, num_of_data, HAL_I2C_TIMEOUT);
  #endif

  #ifndef MPU6050_USE_HAL
    i2c_write_data(i2c_handle, MPU6050_WRITE_ADDR, &command, 1);
    i2c_recv_data(i2c_handle, MPU6050_READ_ADDR, data, num_of_data);
  #endif
}

/* -- accel_sensitiviy_config() --
 * Input: uint8_t configuration
 * Return: uint8_t register_value
 * Description:
 *
 * Internal function of the MPU6050 driver, that configures the accelerometer parameters
 *
 * It returns the register value to be written for the specified configuration
 * and configures the sensitivity value with which we divide every time we convert to g/s.
 *
 * */
static uint8_t accel_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Accelerometer_2G:
      reg_value = ACCEL_SCALE_2G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_2G;
      break;
    case MPU6050_Accelerometer_4G:
      reg_value = ACCEL_SCALE_4G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_4G;
      break;
    case MPU6050_Accelerometer_8G:
      reg_value = ACCEL_SCALE_8G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_8G;
      break;
    case MPU6050_Accelerometer_16G:
      reg_value = ACCEL_SCALE_16G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_16G;
      break;
    default:
      reg_value = ACCEL_SCALE_2G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_2G;
      break;
  }

  return reg_value;
}

/* -- gyro_sensitiviy_config() --
 * Input: uint8_t configuration
 * Return: uint8_t register_value
 * Description:
 *
 * Internal function of the MPU6050 driver, that configures the gyroscope parameters
 *
 * It returns the register value to be written for the specified configuration
 * and configures the sensitivity value with which we divide every time we convert to deg/s.
 *
 * */
static uint8_t gyro_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Gyroscope_250_deg:
      reg_value = GYRO_SCALE_250_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_250_DEG;
      break;
    case MPU6050_Gyroscope_500_deg:
      reg_value = GYRO_SCALE_500_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_500_DEG;
      break;
    case MPU6050_Gyroscope_1000_deg:
      reg_value = GYRO_SCALE_1K_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_1K_DEG;
      break;
    case MPU6050_Gyroscope_2000_deg:
      reg_value = GYRO_SCALE_2K_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_2K_DEG;
      break;
    default:
      reg_value = GYRO_SCALE_250_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_250_DEG;
      break;
  }

  return reg_value;
}

/* -- MPU6050_self_test() --
 * Input: Command
 * Return: SUCCESS or FAILURE
 * Description:
 *
 * Internal function of the MPU6050 driver, that performs the internal self test that the
 * MPU6050 offers .
 * It returns SUCCESS or FAILURE depending on the results.
 *
 * */
static inline uint8_t MPU6050_self_test(void)
{
  uint8_t reg_trx[4];
  uint8_t self_test[6];
  uint8_t i;
  float ft_accel[3];
  float ft_gyro[3];

  // Set accelerometer configuration in ACCEL_CONFIG Register //
  // XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> � 8g //
  reg_trx[0] = ACCEL_CONFIG_REG ;
  reg_trx[1] = accel_sensitiviy_config(MPU6050_Accelerometer_8G)| ACCEL_ENABLE_SELF_TEST;
  i2c_MPU6050_write_reg(reg_trx, 2);

  // Set Gyro configuration in GYRO_CONFIG Register //
  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s //
  reg_trx[0] = GYRO_CONFIG_REG;
  reg_trx[1] = gyro_sensitiviy_config(MPU6050_Gyroscope_250_deg) | GYRO_ENABLE_SELF_TEST;
  i2c_MPU6050_write_reg(reg_trx, 2);

  // Read the Self test values from the registers//
  i2c_MPU6050_read_reg(SELF_TEST_REG, reg_trx, 4);

  // Set the X-axis gyro and accel parameters //
  self_test[0] = (((reg_trx [0]) & ACCEL_SELF_TEST_UPPER_MASK) >> 3) | (((reg_trx [3]) & ACCEL_SELF_TEST_X_MASK) >> 4);
  self_test[3] = reg_trx[0] & GYRO_SELF_TEST_MASK;

  #ifdef DEBUG_MPU6050
    printf("X-axis FT Raw -> Accel %d and Gyro %d\n", self_test[0], self_test[3]);
  #endif

  // Calculate the factory trim value //
  ft_accel[0] = ((self_test[0] - 1))/((float) 30);
  ft_accel[0] = ACCEL_CONSTANT_MULT * pow(ACCEL_CONSTANT_BASE, ft_accel[0]);
  ft_gyro[0] = GYRO_CONSTANT_MULT * pow(GYRO_CONSTANT_BASE, self_test[3] - 1);

  // Set the Y-axis gyro and accel parameters //
  self_test[1] = (((reg_trx [1]) & ACCEL_SELF_TEST_UPPER_MASK) >> 3) | (((reg_trx [3]) & ACCEL_SELF_TEST_Y_MASK) >> 2);
  self_test[4] = reg_trx[1] & GYRO_SELF_TEST_MASK;

  #ifdef DEBUG_MPU6050
    printf("X-axis FT Raw -> Accel %d and Gyro %d\n", self_test[1], self_test[4]);
  #endif

  // Calculate the factory trim value //
  ft_accel[1] = ((self_test[1] - 1))/((float) 30);
  ft_accel[1] = ACCEL_CONSTANT_MULT * pow(ACCEL_CONSTANT_BASE, ft_accel[1]);
  ft_gyro[1] = -GYRO_CONSTANT_MULT * pow(GYRO_CONSTANT_BASE, self_test[4] - 1);

  // Set the Z-axis gyro and accel parameters //
  self_test[2] = (((reg_trx [2]) & ACCEL_SELF_TEST_UPPER_MASK) >> 3) | ((reg_trx [3]) & ACCEL_SELF_TEST_Z_MASK);
  self_test[5] = reg_trx[2] & GYRO_SELF_TEST_MASK;

  #ifdef DEBUG_MPU6050
    printf("X-axis FT Raw -> Accel %d and Gyro %d\n", self_test[2], self_test[5]);
  #endif

  // Calculate the factory trim value //
  ft_accel[2] = ((self_test[2] - 1))/((float) 30);
  ft_accel[2] = ACCEL_CONSTANT_MULT * pow(ACCEL_CONSTANT_BASE, ft_accel[2]);
  ft_gyro[2] = GYRO_CONSTANT_MULT * pow(GYRO_CONSTANT_BASE, self_test[5] - 1);

  // Calculate percentages for accel //
  ft_accel[0] = 100.0 + 100 * (((float) self_test[0] - ft_accel[0])/ft_accel[0]);
  ft_accel[1] = 100.0 + 100 * (((float) self_test[1] - ft_accel[1])/ft_accel[1]);
  ft_accel[2] = 100.0 + 100 * (((float) self_test[2] - ft_accel[2])/ft_accel[2]);

  // Calculate percentages for gyro //
  ft_gyro[0] = 100.0 + 100 * (((float) self_test[3] - ft_gyro[0])/ft_gyro[0]);
  ft_gyro[1] = 100.0 + 100 * (((float) self_test[4] - ft_gyro[1])/ft_gyro[1]);
  ft_gyro[2] = 100.0 + 100 * (((float) self_test[5] - ft_gyro[2])/ft_gyro[2]);

  #ifdef DEBUG_MPU6050
    printf("Accel: X axis STR is %.3f\nAccel: Y axis STR is %.3f\nAccel: Z axis STR is %.3f\n", ft_accel[0], ft_accel[1], ft_accel[2]);
    printf("Gyro: X axis STR is %.3f\nGyro: Y axis STR is %.3f\nGyro: Z axis STR is %.3f\n", ft_gyro[0], ft_gyro[1], ft_gyro[2]);
  #endif

  // Check whether we are out of the maximum ratings //
  // If we are return MPU6050_SUCCESS //
  for(i = 0; i < 3; i++){

    if((ft_accel[i] > MAX_SELF_TEST_VAL) | (ft_accel[i] < -MAX_SELF_TEST_VAL))
      return MPU6050_FAILURE;

    if((ft_gyro[i] > MAX_SELF_TEST_VAL) | (ft_gyro[i] < -MAX_SELF_TEST_VAL))
      return MPU6050_FAILURE;
  }

  // Self test success //
  return MPU6050_SUCCESS;
}

/* -- MPU6050_set_power_mode() --
 * Input: uint8_t power_mode, uint8_t frequency
 * Return: None
 * Description:
 *
 * Function of the MPU6050 that sets the power mode of the sensor
 * based on the input arguments.
 * Freq argument is taken into account only when CYCLE mode is used.
 * */
void MPU6050_set_power_mode(uint8_t power_mode, uint8_t freq)
{
  uint8_t reg_trx[3]; // Values to be written to the register //

  // MPU6050 is already configured to this power mode //
  mpu6050_config.power_mode = power_mode;

  // Set the register address //
  reg_trx[0] = PWR_MGMT_1_REG;

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (power_mode) {
    case MPU6050_ACCEL_ONLY:
      reg_trx[1] = TEMP_DIS;
      reg_trx[2] = GYRO_STBY;
      break;
    case MPU6050_GYRO_ONLY:
      reg_trx[1] = TEMP_DIS;
      reg_trx[2] = ACCEL_STBY;
      break;
    case MPU6050_CYCLE:
      reg_trx[1] = ENABLE_CYCLE | TEMP_DIS;
      reg_trx[2] = 0x00; // Needs Freq component //
      break;
    case MPU6050_SLEEP:
      reg_trx[1] = ENABLE_SLEEP;
      reg_trx[2] = 0x00; // Don't care Value //
      break;
    case MPU6050_POWER_ON:
      reg_trx[1] = PWR_WAKE_UP;
      reg_trx[2] = 0x00; // Don't care Value //
      break;
    default:
      reg_trx[1] = PWR_WAKE_UP;
      reg_trx[2] = 0x00; // Don't care Value //
      break;
  }

  // Only when in
  if(power_mode == MPU6050_CYCLE) {
    // Based on frequency chosen set power management reg 2 //
    // The value is only valid when in cycle mode  //
    switch (freq) {
      case MPU6050_low_power_1_25Hz:
        reg_trx[2] |= LP_WAKE_FREQ_1_25HZ;
        break;
      case MPU6050_low_power_5Hz:
        reg_trx[2] |= LP_WAKE_FREQ_5HZ;
        break;
      case MPU6050_low_power_20Hz:
        reg_trx[2] |= LP_WAKE_FREQ_20HZ;
        break;
      case MPU6050_low_power_40Hz:
        reg_trx[2] |= LP_WAKE_FREQ_40HZ;
        break;
      case MPU6050_rate_dont_care:
        reg_trx[2] |= 0x00; // Don't care Value //
        break;
      default:
        reg_trx[2] |= 0x00; // Don't care Value //
        break;
    }

    // Update the frequency in the configuration struct //
    mpu6050_config.freq = freq;
  }

  // Write the chosen values to the register //
  i2c_MPU6050_write_reg(reg_trx, 2);

  // Write to the second register //
  // Burst write problems ? //
  reg_trx[0] = PWR_MGMT_2_REG;
  reg_trx[1] = reg_trx[2];
  i2c_MPU6050_write_reg(reg_trx , 2);

  return;
}

/* -- MPU6050_enable_irq() --
 * Input: uint8_t configuration
 * Return: None
 * Description:
 *
 * Function of the MPU6050 that configures and enables interrupts and interrupt
 * pin in the MPU6050.
 *
 * Interrupts are generated when a measurement is complete.
 *
 * */
void MPU6050_enable_irq(uint8_t config)
{
  uint8_t reg_trx[2];

  // Configure the pin //
  reg_trx[0] = INT_PIN_CFG_REG;
  reg_trx[1] = config;
  i2c_MPU6050_write_reg(reg_trx, 2);

  // Configure the pin //
  reg_trx[0] = INT_ENABLE_REG;
  reg_trx[1] = IRQ_ENABLE;
  i2c_MPU6050_write_reg(reg_trx, 2);

  // Read the IRQ status to clear it //
  i2c_MPU6050_read_reg(INT_STATUS_REG, reg_trx, 1);

  // Update the status //
  mpu6050_config.irq_enable = IRQ_ENABLE;
}

/* -- MPU6050_get_irq_status() --
 * Input: None
 * Return: SUCCESS or FAILURE
 * Description:
 *
 * Function of the MPU6050 that reads the irq status register of the MPU6050
 * sensor.
 *
 * When an interrupt is generated the STATUS register bit for DATA_RD is set
 * until it is read back.
 *
 * Interrupts are generated when a measurement is complete. In order to clear it
 * depending on configuration one has to read to perform a read operation:
 *
 * Either read the IRQ_STATUS register of MP6050 or perform a read operation
 * on
 *
 * */
uint8_t MPU6050_get_irq_status(void)
{
  uint8_t reg_rx;

  // Read the register //
  i2c_MPU6050_read_reg(INT_STATUS_REG, &reg_rx, 1);

  // Check if the flag is active //
  if(reg_rx == IRQ_DATA_RD)
    return MPU6050_SUCCESS;
  else
    return 0;

}

/* -- MPU6050_disable_irq() --
 * Input: None
 * Return: None
 * Description:
 *
 * Function of the MPU6050 that disables the irq generation of the MPU6050
 * sensor.
 *
 * Interrupts are generated when a measurement is complete.
 *
 * */
void MPU6050_disable_irq(void)
{
  uint8_t reg_trx[2];

  // Configure the pin //
  reg_trx[0] = INT_ENABLE_REG;
  reg_trx[1] = IRQ_DISABLE;
  i2c_MPU6050_write_reg(reg_trx, 2);

  // Read the IRQ status to clear it //
  i2c_MPU6050_read_reg(INT_STATUS_REG, reg_trx, 1);

  // Update the status //
  mpu6050_config.irq_enable = IRQ_DISABLE;
}

/* -- MPU6050_peripheral_config() --
 * Input: None
 * Return: None
 * Description:
 *
 * Initializes and configures the GPIOs and the I2C peripheral
 *
 * */
void MPU6050_peripheral_config(void)
{
  // Enable the GPIO Port Clock //
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure the GPIOs //
  LL_GPIO_SetPinSpeed(SCL_Port, SCL_Pin, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(SCL_Port, SCL_Pin, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(SCL_Port, SCL_Pin, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(SCL_Port, SCL_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SCL_Port, SCL_Pin, LL_GPIO_AF_4);

  LL_GPIO_SetPinSpeed(SDA_Port, SDA_Pin, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(SDA_Port, SDA_Pin, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(SDA_Port, SDA_Pin, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(SDA_Port, SDA_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(SDA_Port, SDA_Pin, LL_GPIO_AF_4);

  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();

  #ifdef MPU6050_USE_HAL
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
      while(1);
    }
  #endif

  #ifndef MPU6050_USE_HAL

    // Use I2C1 peripheral //
    i2c_handle = I2C1;

    // Reset the peripheral
    i2c_handle->CR1 |= I2C_CR1_SWRST;
    i2c_handle->CR1 &= ~I2C_CR1_SWRST;

    LL_I2C_SetMode(i2c_handle, LL_I2C_MODE_I2C);
    LL_I2C_DisableClockStretching(i2c_handle);
    LL_I2C_ConfigSpeed(i2c_handle,
		       HAL_RCC_GetPCLK1Freq(),
		       300000,
		       LL_I2C_DUTYCYCLE_2);
  #endif
}

/* -- MPU6050_peripheral_config() --
 * Input: None
 * Return: SUCCESS or FAILURE
 * Description:
 *
 * Initializes and configures the MPU6050 sensor.
 * Performs self-test if it is enabled.
 *
 * */
uint8_t MPU6050_Init(uint8_t accel_config, uint8_t gyro_config, uint8_t sample_rate)
{
  uint8_t check;
  uint8_t reg_trx[2];

  // check device ID WHO_AM_I register//
  i2c_MPU6050_read_reg(WHO_AM_I_REG, &check, 1);

  if (check == MPU6050_DEVICE_ID)  // 0x68 will be returned by the sensor if everything goes well //
  {
    // power management register 0X6B we should write all 0's to wake the sensor up //
    reg_trx[0] = PWR_MGMT_1_REG;
    reg_trx[1] = PWR_WAKE_UP;
    i2c_MPU6050_write_reg(reg_trx, 2);

    // Set the power mode //
    mpu6050_config.power_mode = MPU6050_POWER_ON;

    // Set Sample Rate by writing SMPLRT_DIV register //
    reg_trx[0] = SMPLRT_DIV_REG;
    reg_trx[1] = sample_rate;
    i2c_MPU6050_write_reg(reg_trx, 2);

    // Perform the self-test if it is enabled //
    #ifdef MPU6050_SELF_TEST
      check = MPU6050_self_test();

      // Printing messages for debug //
      #ifdef DEBUG_MPU6050
        if(check == 0)
          printf("Self test passed\n");
        else
          printf("Self test not passed\n");
      #endif

      // Terminate initialization in case of failure //
      if(check == 1)
        return MPU6050_FAILURE;
      #endif

    // Set accelerometer configuration in ACCEL_CONFIG Register //
    // XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> � 2g //
    reg_trx[0] = ACCEL_CONFIG_REG;
    reg_trx[1] = accel_sensitiviy_config(accel_config);
    i2c_MPU6050_write_reg(reg_trx, 2);

    #ifdef DEBUG_MPU6050
      printf("Config accel is %f and reg val is %x\n", mpu6050_config.accel_sensitivity, reg_trx[1]);
    #endif

    // Set Gyro configuration in GYRO_CONFIG Register //
    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s //
    reg_trx[0] = GYRO_CONFIG_REG;
    reg_trx[1] = gyro_sensitiviy_config(gyro_config);
    i2c_MPU6050_write_reg(reg_trx, 2);

    printf("Config gyro is %f and reg val is %x\n", mpu6050_config.gyro_sensitivity, reg_trx[1]);

    return MPU6050_SUCCESS;
  }
  return MPU6050_FAILURE;
}

/* -- MPU6050_Read_Accel() --
 * Input: MPU6050_t *DataStruct
 * Return: None
 * Description:
 *
 * Reads the accelerometer raw values from the MPU6050
 * and converts them into g's.
 * The values are stored in the DataStruct corresponding fields.
 *
 * */
void MPU6050_Read_Accel(MPU6050_t *DataStruct)
{
  uint8_t Rx_Data[6];

  // Read 6 BYTES of data starting from ACCEL_XOUT_H register //
  i2c_MPU6050_read_reg(ACCEL_XOUT_H_REG, Rx_Data, 6);

  DataStruct->Accel_X_RAW = (uint16_t) (Rx_Data[0] << 8 | Rx_Data[1]);
  DataStruct->Accel_Y_RAW = (uint16_t) (Rx_Data[2] << 8 | Rx_Data[3]);
  DataStruct->Accel_Z_RAW = (uint16_t) (Rx_Data[4] << 8 | Rx_Data[5]);

  // Convert the RAW values into acceleration in 'g' , so we have //
  // to divide according to the Full scale value set in FS_SEL    //
  DataStruct->Ax = DataStruct->Accel_X_RAW / (mpu6050_config.accel_sensitivity);
  DataStruct->Ay = DataStruct->Accel_Y_RAW / (mpu6050_config.accel_sensitivity);
  DataStruct->Az = DataStruct->Accel_Z_RAW / (mpu6050_config.accel_sensitivity);
}

/* -- MPU6050_Read_Gyro() --
 * Input: MPU6050_t *DataStruct
 * Return: None
 * Description:
 *
 * Reads the gyro's raw values from the MPU6050
 * and converts them into degrees.
 * The values are stored in the DataStruct corresponding fields.
 *
 * */
void MPU6050_Read_Gyro(MPU6050_t *DataStruct)
{
  uint8_t Rx_Data[6];

  // Read 6 BYTES of data starting from GYRO_XOUT_H register //
  i2c_MPU6050_read_reg(GYRO_XOUT_H_REG, Rx_Data, 6);

  DataStruct->Gyro_X_RAW = (uint16_t) (Rx_Data[0] << 8 | Rx_Data[1]);
  DataStruct->Gyro_Y_RAW = (uint16_t) (Rx_Data[2] << 8 | Rx_Data[3]);
  DataStruct->Gyro_Z_RAW = (uint16_t) (Rx_Data[4] << 8 | Rx_Data[5]);

  // Convert the RAW values into into dps , so we have          //
  // to divide according to the Full scale value set in FS_SEL  //
  DataStruct->Gx = DataStruct->Gyro_X_RAW / (mpu6050_config.gyro_sensitivity);
  DataStruct->Gy = DataStruct->Gyro_Y_RAW / (mpu6050_config.gyro_sensitivity);
  DataStruct->Gz = DataStruct->Gyro_Z_RAW / (mpu6050_config.gyro_sensitivity);
}

/* -- MPU6050_Read_Gyro() --
 * Input: MPU6050_t *DataStruct
 * Return: None
 * Description:
 *
 * Reads the temperature raw values from the MPU6050
 * and converts them into degrees.
 * The values are stored in the DataStruct corresponding fields.
 *
 * */
void MPU6050_Read_Temp(MPU6050_t *DataStruct)
{
  uint8_t Rx_Data[2];
  int16_t temp;

  // Read 2 BYTES of data starting from TEMP_OUT_H_REG register
  i2c_MPU6050_read_reg(TEMP_OUT_H_REG, Rx_Data, 2);

  temp = (int16_t) (Rx_Data[0] << 8 | Rx_Data[1]);
  DataStruct->Temperature = (float) ((uint16_t) temp / (float) 340.0 + (float) 36.53);
}

/* -- MPU6050_Read_All() --
 * Input: MPU6050_t *DataStruct
 * Return: None
 * Description:
 *
 * Reads every raw value (temp, accel, gyro) from the MPU6050
 * and converts them into their unit of measurement.
 * The values are stored in the DataStruct corresponding fields.
 *
 * */
void MPU6050_Read_All(MPU6050_t *DataStruct)
{
  uint8_t Rx_Data[14];
  uint16_t temp;

  // Read 14 BYTES of data starting from ACCEL_XOUT_H register //
  i2c_MPU6050_read_reg(ACCEL_XOUT_H_REG, Rx_Data, 14);

  DataStruct->Accel_X_RAW = (uint16_t) (Rx_Data[0] << 8 | Rx_Data[1]);
  DataStruct->Accel_Y_RAW = (uint16_t) (Rx_Data[2] << 8 | Rx_Data[3]);
  DataStruct->Accel_Z_RAW = (uint16_t) (Rx_Data[4] << 8 | Rx_Data[5]);

  DataStruct->Gyro_X_RAW = (uint16_t) (Rx_Data[8] << 8 | Rx_Data[9]);
  DataStruct->Gyro_Y_RAW = (uint16_t) (Rx_Data[10] << 8 | Rx_Data[11]);
  DataStruct->Gyro_Z_RAW = (uint16_t) (Rx_Data[12] << 8 | Rx_Data[13]);

  DataStruct->Gx = DataStruct->Gyro_X_RAW / (mpu6050_config.gyro_sensitivity);
  DataStruct->Gy = DataStruct->Gyro_Y_RAW / (mpu6050_config.gyro_sensitivity);
  DataStruct->Gz = DataStruct->Gyro_Z_RAW / (mpu6050_config.gyro_sensitivity);

  DataStruct->Ax = DataStruct->Accel_X_RAW / (mpu6050_config.accel_sensitivity);
  DataStruct->Ay = DataStruct->Accel_Y_RAW / (mpu6050_config.accel_sensitivity);
  DataStruct->Az = DataStruct->Accel_Z_RAW / (mpu6050_config.accel_sensitivity);

  temp = (uint16_t) (Rx_Data[6] << 8 | Rx_Data[7]);
  DataStruct->Temperature = (float) ((uint16_t) temp / (float) 340.0 + (float) 36.53);

  // Kalman angle solve
  double dt = (double) (HAL_GetTick() - timer) / 1000;
  timer = HAL_GetTick();
  double roll;
  double roll_sqrt = sqrt(
	  DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
  if (roll_sqrt != 0.0) {
      roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
  } else {
      roll = 0.0;
  }
  double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
  if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
      KalmanY.angle = pitch;
      DataStruct->KalmanAngleY = pitch;
  } else {
      DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
  }
  if (fabs(DataStruct->KalmanAngleY) > 90)
      DataStruct->Gx = -DataStruct->Gx;

  DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);

}

/* -- Kalman_getAngle() --
 * Input: Kalman_getAngle *Kalman_Data, double newAngle, double newRate, double dt
 * Return: None
 * Description:
 *
 * Function that given the input arguments it calculates a Kalman angle.
 *
 * */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
  double rate = newRate - Kalman->bias;
  Kalman->angle += dt * rate;

  Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
  Kalman->P[0][1] -= dt * Kalman->P[1][1];
  Kalman->P[1][0] -= dt * Kalman->P[1][1];
  Kalman->P[1][1] += Kalman->Q_bias * dt;

  double S = Kalman->P[0][0] + Kalman->R_measure;
  double K[2];
  K[0] = Kalman->P[0][0] / S;
  K[1] = Kalman->P[1][0] / S;

  double y = newAngle - Kalman->angle;
  Kalman->angle += K[0] * y;
  Kalman->bias += K[1] * y;

  double P00_temp = Kalman->P[0][0];
  double P01_temp = Kalman->P[0][1];

  Kalman->P[0][0] -= K[0] * P00_temp;
  Kalman->P[0][1] -= K[0] * P01_temp;
  Kalman->P[1][0] -= K[1] * P00_temp;
  Kalman->P[1][1] -= K[1] * P01_temp;

  return Kalman->angle;
};
