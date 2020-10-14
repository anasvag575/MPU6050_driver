/**
  ******************************************************************************
  * @file      : i2c_driver.h
  * @brief     : Header for i2c_driver.c file.
  ******************************************************************************
  * Author: Anastasis Vagenas -> Contact: anasvag29@gmail.com
  *
  * I2C Read/Write Optimized routines
  *
  * They use either direct access to reduce code size or LL drivers.
  * Also functions have small memory overhead when called (compared to HAL).
  *
  * For now 2 functions are supported:
  * i2c_write_data() -> Writes to the I2C bus in blocking mode
  * i2c_recv_data() -> Reads from the I2C bus in blocking mode
  *
  * @Timeout is not supported yet
  *
  ******************************************************************************
  */
#ifndef __I2C_DRIV_H // Define to prevent recursive inclusion //
#define __I2C_DRIV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_ll_i2c.h"

/* -- i2c_write_data() --
 * Input: I2C_TypeDef *i2c_handle, uint8_t address, uint8_t *data, uint8_t number_of_data
 * Return: None
 * Description:
 *
 * Routine that writes data to the I2C bus specified by the input arguments.
 *
 * Write to the specified device (address) using the I2C peripheral (i2c_handle)
 * a specific number of bytes (num_of_data) which are stored on an array (uint8_t *data)
 *
 * */
static inline void i2c_write_data(I2C_TypeDef *i2c_handle, uint8_t address, uint8_t *data, uint8_t num_of_data)
{
  uint8_t i;

  // Enable the I2C peripheral //
  LL_I2C_Enable(i2c_handle);

  // Generate the start condition //
  LL_I2C_GenerateStartCondition(i2c_handle);

  // Wait for SB (start condition) flag to be set //
  while (LL_I2C_IsActiveFlag_SB(i2c_handle) == 0);

  // Write the I2C address to the DR register //
  LL_I2C_TransmitData8(i2c_handle, address);

  // Wait for the ADDR flag sent to be set //
  while (LL_I2C_IsActiveFlag_ADDR(i2c_handle) == 0);

  // Clear the ADDR flag //
  LL_I2C_ClearFlag_ADDR(i2c_handle);

  // Enter data transmission loop //
  for(i = 0; i < num_of_data; i++){

    // Wait for the TXE flag to be set //
    while(LL_I2C_IsActiveFlag_TXE(i2c_handle) == 0);

    // Transmit the data //
    LL_I2C_TransmitData8(i2c_handle, data[i]);
  }

  // Wait for the BTF (Byte transfer complete) flag to be set //
  while(LL_I2C_IsActiveFlag_BTF(i2c_handle) == 0);

  LL_I2C_GenerateStopCondition(i2c_handle);

  // Disable the I2C peripheral //
  LL_I2C_Disable(i2c_handle);

}

/* -- i2c_SHT2x_recv_data() --
 * Input: I2C_TypeDef *i2c_handle, uint8_t address, uint8_t *data, uint8_t number_of_data
 * Return: None
 * Description:
 *
 * Routine that reads data from the I2C bus specified by the input arguments.
 *
 * Read from the specified device (address) using the I2C peripheral (i2c_handle)
 * a specific number of bytes (num_of_data) which are stored on an array (uint8_t *data)
 *
 * */
static inline void i2c_recv_data(I2C_TypeDef *i2c_handle, uint8_t address, uint8_t *data, uint8_t num_of_data)
{
  uint8_t remaining_data = num_of_data;
  uint8_t cur = 0;

  // Enable the I2C peripheral //
  LL_I2C_Enable(i2c_handle);

  // Disable POS bit //
  LL_I2C_DisableBitPOS(i2c_handle);

  // Generate the start condition //
  LL_I2C_GenerateStartCondition(i2c_handle);

  // Wait for SB (start condition) flag to be set //
  while (LL_I2C_IsActiveFlag_SB(i2c_handle) == 0);

  // Write the I2C address to the DR register //
  LL_I2C_TransmitData8(i2c_handle, address);

  // Wait for the ADDR flag sent to be set //
  while (LL_I2C_IsActiveFlag_ADDR(i2c_handle) == 0);

  // Depending on number of data to be received //
  // different configuration //
  switch (num_of_data)
  {
    case 1:// 1 byte transmission //
    {
      LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);  // NACK for byte //
      LL_I2C_ClearFlag_ADDR(i2c_handle);		                // Clear the ADDR flag //
      LL_I2C_GenerateStopCondition(i2c_handle);		          // Generate the stop condition //
    }
    break;
    case 2:// 2 byte transmission //
    {
      LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);  // NACK for 2 bytes //
      LL_I2C_EnableBitPOS(i2c_handle);			                // Enable POS bit //
      LL_I2C_ClearFlag_ADDR(i2c_handle);		                // Clear the ADDR flag //
    }
    break;
    default:
    {
      LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_ACK);   // ACK for more than 2 bytes //
      LL_I2C_ClearFlag_ADDR(i2c_handle);		                // Clear the ADDR flag //
    }
    break;
  }

  // Transmission start //
  while (remaining_data != 0){
    switch (remaining_data){
      case 1:// 1 byte transmission //
      {
        while(LL_I2C_IsActiveFlag_RXNE(i2c_handle) == 0);    // Wait for RXNE flag to be set //

        data[cur] = LL_I2C_ReceiveData8(i2c_handle);
        remaining_data = 0;
      }
      break;
      case 2:// 2 byte transmission //
      {
        while(LL_I2C_IsActiveFlag_BTF(i2c_handle) == 0);	    // Wait for transfer complete flag //
        LL_I2C_GenerateStopCondition(i2c_handle);		          // Generate the stop condition //

        data[cur] = LL_I2C_ReceiveData8(i2c_handle);		      // Read the remaining 2 bytes //
        data[cur + 1]  = LL_I2C_ReceiveData8(i2c_handle);
        remaining_data = 0;
      }
      break;
      case 3:// 3 byte transmission //
      {
        while(LL_I2C_IsActiveFlag_BTF(i2c_handle) == 0);	    // Wait for transfer complete flag //
        LL_I2C_AcknowledgeNextData(i2c_handle, LL_I2C_NACK);
        data[cur] = LL_I2C_ReceiveData8(i2c_handle);

        while(LL_I2C_IsActiveFlag_BTF(i2c_handle) == 0);	    // Wait for transfer complete flag //
        LL_I2C_GenerateStopCondition(i2c_handle);		          // Generate the stop condition //

        data[cur + 1] = LL_I2C_ReceiveData8(i2c_handle);	    // Read the remaining 2 bytes //
        data[cur + 2] = LL_I2C_ReceiveData8(i2c_handle);
        remaining_data = 0;
      }
      break;
      default:
      {
        while(LL_I2C_IsActiveFlag_RXNE(i2c_handle) == 0);	    // Wait for transfer complete flag //

        data[cur] = LL_I2C_ReceiveData8(i2c_handle);
        cur++;
        remaining_data--;

        if (LL_I2C_IsActiveFlag_BTF(i2c_handle)){
          data[cur] = LL_I2C_ReceiveData8(i2c_handle);
          cur++;
          remaining_data--;
        }

      }
      break;
    }
  }

  // Disable the I2C peripheral //
  LL_I2C_Disable(i2c_handle);

}

#endif
