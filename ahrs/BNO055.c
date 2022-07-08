/* 
 * File:   BNO055.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on July 7, 2022
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "BNO055.h" // The header file for this source file. 

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

/* uncomment below to run test harness */
// #define BNO055_TEST
/** BNO055 Address A: when ADR (COM3) pin is tied to ground (default) **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B: when ADR (COM3) pin is tied to +3.3V **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)
/*Page numbers*/
#define BNO055_PAGE0 0
#define BNO055_PAGE1 1
/** Sensor configuration values: 
 * ACC_PWR_Mode <2:0> ACC_BW <2:0> ACC_Range <1:0>
 * 64Hz [GYR_Config_0]: xx110xxxb | 250 dps [GYR_Config_0]: xxxxx011b**/
#define ACC_CONFIG_PARAMS (0xC) //+/-2g, 62.5 Hz BW
#define MAG_CONFIG_PARAMS (0X1F)
#define GYRO_CONFIG_PARAMS_0 (0x33)
#define UNITS_PARAM (0x01)
#define P0_AXIS  (0x21)
#define P0_SIGN (0x04)
#define P1_AXIS  (0x24)
#define P1_SIGN (0x00)
#define P5_AXIS  (0x21)
#define P5_SIGN (0x01)

#define PICO_I2C_SDA_PIN 16
#define PICO_I2C_SCL_PIN 17

#define SUCCESS 0
#define ERROR -1

#define TRUE 1
#define FALSE 0


/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/


/*Register address copied from Adafruit Github
 * https://github.com/adafruit/Adafruit_BNO055/blob/master/Adafruit_BNO055.h    *
 */
static enum {
    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    BNO055_EULER_H_MSB_ADDR = 0X1B,
    BNO055_EULER_R_LSB_ADDR = 0X1C,
    BNO055_EULER_R_MSB_ADDR = 0X1D,
    BNO055_EULER_P_LSB_ADDR = 0X1E,
    BNO055_EULER_P_MSB_ADDR = 0X1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0X34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0X35,
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0X3B,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0X5B,
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0X61,
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
} BNO055_P0_REGISTERS;

static enum {
    /* PAGE1 REGISTER DEFINITION START*/
    /*sensor configurations*/
    BNO055_ACC_CONFIG = 0x08,
    BNO055_MAG_CONFIG = 0x09,
    BNO055_GYR_CONFIG_0 = 0x0A,
    BNO055_GYR_CONFIG_1 = 0x0B,
    BNO055_ACC_SLEEP_CONFIG = 0x0C,
    BNO055_GYR_SLEEP_CONFIG = 0x0D,
    BNO055_INT_MSK = 0x0F,
    BNO055_INT_EN = 0x10,
    BNO055_ACC_AM_THRES = 0x11,
    BNO055_ACC_INT_SETTINGS = 0x12,
    BNO055_ACC_HG_DURATION = 0x13,
    BNO055_ACC_HG_THRES = 0x14,
    BNO055_ACC_NM_THRES = 0x15,
    BNO055_ACC_NM_SET = 0x16,
    BNO055_GYR_INT_SETTINGS = 0x17,
    BNO055_GYR_HR_X_SET = 0x18,
    BNO055_GYR_DUR_X = 0x19,
    BNO055_GYR_HR_Y_SET = 0x1A,
    BNO055_GYR_DUR_Y = 0x1B,
    BNO055_GYR_HR_Z_SET = 0x1C,
    BNO055_GYR_DUR_Z = 0x1D,
    BNO055_GYR_AM_THRES = 0x1E,
    BNO055_GYR_AM_SET = 0x1F
} BNO055_P1_REGISTERS;

/** BNO055 power settings */
static enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
} BNO055_powermode;

/** Operation mode settings **/
static enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
} BNO055_opmode;

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/
// void DelayMicros(uint32_t microsec);
/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function BNO055_Init(void)

 * @return 0 if error, 1 if succeeded
 * @brief  Initializes the BNO055 for usage. Sensors will be at Accel: 2g,Gyro:  250dps
 * @author Aaron Hunter */
int8_t BNO055_Init(void) {
    /*TODO translate into Pico-ese*/

     uint8_t byteReturn = 0; //pointer to byte read by I2C
     uint8_t settings_buffer[2];
     uint8_t length = 2;
     int write_return = 0;
     int read_return = 0;

    /* Read chip ID to verify sensor connection */
    write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR, 1, true);  
    read_return = i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, &byteReturn, 1, false);// False - finished with bus
    if (byteReturn != BNO055_ID) {
        printf("Device returned %X, should return %X  \r\n", byteReturn, BNO055_ID);
        return (ERROR);
    } else {
     /* default state is in CONFIG_MODE. This is the only mode in which all the 
     * writable register map entries can be changed. (Exceptions from this rule
     *  are the interrupt registers (INT and INT_MSK) and the operation mode
     *  register (OPR_MODE), which can be modified in any operation mode.)*/
        /* TODO set GPIO to reset the BNO055 chip */

        /* Put device into configuration mode */
        settings_buffer[0] = BNO055_OPR_MODE_ADDR; // operation mode register
        settings_buffer[1] = OPERATION_MODE_CONFIG; // set to config mode

        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false);  
        sleep_ms(20); // switching to CONFIGMODE takes ~ 19 msec.  
        
        /*set the register page to page 1*/
        settings_buffer[0] = BNO055_PAGE_ID_ADDR; // change page 
        settings_buffer[1] = BNO055_PAGE1; // set to page 1
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false);  

        /* Set Sensor parameters */
        settings_buffer[0] = BNO055_ACC_CONFIG; // Accelerometer config register 
        settings_buffer[1] = ACC_CONFIG_PARAMS; // Accelerometer settings: +/- 2g, 62.5 Hz, normal
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false);  

        settings_buffer[0] = BNO055_MAG_CONFIG;
        settings_buffer[1] = MAG_CONFIG_PARAMS;
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false); 

        settings_buffer[0] = BNO055_GYR_CONFIG_0;
        settings_buffer[1] = GYRO_CONFIG_PARAMS_0;
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false); 

        /* verify settings */
        // settings_buffer[0] = BNO055_ACC_CONFIG; // Accelerometer config register 
        // write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, 1, true);  
        // read_return = i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, &settings_buffer[1], 3, false);// False - finished with bus
        // printf("%d read, start reg: %X, set_1 %X, set_2 %X, set_3 %X \r\n", read_return, settings_buffer[0], 
        // settings_buffer[1], settings_buffer[2], settings_buffer[3]);

        /*set the register page to page 0*/
        settings_buffer[0] = BNO055_PAGE_ID_ADDR; // change page 
        settings_buffer[1] = BNO055_PAGE0; // set to page 0
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false);  
        
        /*set units*/
        settings_buffer[0] = BNO055_UNIT_SEL_ADDR; // units register 
        settings_buffer[1] = UNITS_PARAM; // set units
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false);  
        
        /* Set operation mode to AMG */
        settings_buffer[0] = BNO055_OPR_MODE_ADDR; // operation mode register
        settings_buffer[1] = OPERATION_MODE_AMG; // AMG mode
        write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, settings_buffer, length, false);  
        return (SUCCESS);
    }

}

/**
 * @Function BNO055_ReadRaw(int16_t accel[3], int16_t mag[3], int16_t gyro[3], int16_t *temp)
 * @param accel[3] array of int16 words for  accelerometer axes
 * @param mag[3] array of int16 words for magnetomer axes
 * @param gyro[3] array of 1nt16 words for gyro axes
 * @param *temp pointer to temperature byte
 * @return None
 * @brief reads sensor axis as given by name
 * @author Aaron Hunter*/
void BNO055_ReadRaw(int16_t accel[3], int16_t mag[3], int16_t gyro[3], int8_t *temp) {
    uint8_t buffer[6];
    uint8_t reg_address;
    int8_t length = 6; // buffer length in bytes
    int write_return = 0;
    int read_return = 0;
    int index = 0;
    int array_length = 3;

    /* Accelerometer data*/
    reg_address = BNO055_ACCEL_DATA_X_LSB_ADDR;
    write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &reg_address, 1, true);  
    read_return = i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, buffer, length, false);
    /* parse accel data into array*/
    for(index = 0; index < array_length; index++){
        accel[index] = buffer[index*2 + 1] << 8 | buffer[index*2];
    }
    /* read the next 6 bytes for mag*/
    reg_address = BNO055_MAG_DATA_X_LSB_ADDR;
    write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &reg_address, 1, true);  
    read_return = i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, buffer, length, false);
    /* parse mag data into array*/
    for(index = 0; index < array_length; index++){
        mag[index] = buffer[index*2 + 1] << 8 | buffer[index*2];
    }
        /* read the next 6 bytes for gyro*/
    reg_address = BNO055_GYRO_DATA_X_LSB_ADDR;
    write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &reg_address, 1, true);  
    read_return = i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, buffer, length, false);
    /* parse gyro data into array*/
    for(index = 0; index < array_length; index++){
        gyro[index] = buffer[index*2 + 1] << 8 | buffer[index*2];
    }
    reg_address = BNO055_TEMP_ADDR;
    write_return = i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &reg_address, 1, true);  
    read_return = i2c_read_blocking(i2c_default, BNO055_ADDRESS_A, buffer, 1, false);
    /* chip temperature*/
    *temp = buffer[0];
}



/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/


#ifdef BNO055_TEST

int main(void) {
    uint8_t initResult;
    uint rate_return = 0;

    /* Data containers */
    int16_t accels[3] = {0};
    int16_t mags[3] = {0};
    int16_t gyros[3] = {0};
    int8_t temperature = 0;
    int period = 100; // repetition rate in msec


    /* init pico */
    stdio_init_all();
    printf("Welcome to the BNO055 test harness, compiled " __DATE__ " " __TIME__ "\r\n");
    printf("Sensor data will be streamed after initialization.\r\n");
    
    /* We use I2C0 on the SDA and SCL pico pins (16,17) */
    rate_return = i2c_init(i2c_default, 400 * 1000);
    printf("I2C rate set to %d \r\n", rate_return);
    gpio_set_function(PICO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C_SDA_PIN);
    gpio_pull_up(PICO_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_I2C_SDA_PIN, PICO_I2C_SCL_PIN, GPIO_FUNC_I2C));

    sleep_ms(1000);  //give chip time to settle
    initResult = BNO055_Init();
    if (initResult != SUCCESS) {
        printf("Initialization of IMU failed, stopping here.\r\n");
    } else {
        printf("Initialization succeeded!\r\n");
        while (1) {
            /* read sensors*/
            BNO055_ReadRaw(accels, mags, gyros, &temperature);
            /* print out data */
            printf("Accel: (%+6d, %+6d, %+6d)   ", accels[0], accels[1], accels[2]);
            printf("Mag: (%+6d, %+6d, %+6d)   ", mags[0], mags[1], mags[2]);
            printf("Gyro: (%+6d, %+6d, %+6d)   ", gyros[0], gyros[1], gyros[2]);
            printf("Temp: %+6d", temperature);
            printf("\r\n");
            sleep_ms(period); // wait for one period
        }
    }
    while (1);
}

#endif