/* 
 * File:   BNO055.h
 * Author: Aaron Hunter
 * 
 * Software module to communicate with the IMU over I2C.
 * Provides access to each raw sensor axis along with raw temperature
 *
 * Created on July 7, 2022 
 */

#ifndef BNO055_H
#define	BNO055_H

#define PICO_I2C_SDA_PIN 16
#define PICO_I2C_SCL_PIN 17

#define SUCCESS 0
#define ERROR -1

#define TRUE 1
#define FALSE 0

/**
 * @Function BNO055_Init(void)

 * @return SUCCESS or ERROR
 * @brief  Initializes the BNO055 for usage. Sensors will be at Accel: 2g,Gyro:  250dps
 * @author Aaron Hunter */
int8_t BNO055_Init(void);

/**
 * @Function BNO055_ReadRaw(int16_t accel[3], int16_t mag[3], int16_t gyro[3], int16_t *temp)
 * @param pointer to accel[3] array of int16 words for  accelerometer axes
 * @param pointer to mag[3] array of int16 words for magnetomer axes
 * @param pointer to gyro[3] array of 1nt16 words for gyro axes
 * @param *temp pointer to temperature byte
 * @return None
 * @brief reads accel, mag, gyro, and temperature raw data
 * @author Aaron Hunter*/
void BNO055_ReadRaw(int16_t accel[3], int16_t mag[3], int16_t gyro[3], int8_t *temp);
    

#endif

