/* 
 * File:   ahrs_main.c
 * Author: Aaron
 *
 * Created on June 21, 2022, 9:50 AM
 */

/*******************************************************************************
 * #includes                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "BNO055.h" // The header file for this source file. 
#include "ahrs_m_update.h"


/*******************************************************************************
 * #defines                                                                    *
 ******************************************************************************/
#define SUCCESS 0
#define ERROR -1

#define TRUE 1
#define FALSE 0

#define PERIOD 20000 //period in usec

#define MSZ 3
#define QSZ 4

#define DT 0.02 

/*******************************************************************************
 * Function Declarations                                                       *
 ******************************************************************************/

/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
void m_v_mult(double m[MSZ][MSZ], double v[MSZ], double v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
}

/**
 * @function lin_alg_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function extract_angles();
 * Extract Euler angles from the DCM
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param psi A pointer to return the Yaw angle in radians from -pi to pi
 * @param theta A pointer to return the Pitch angle in radians from -pi to pi
 * @param phi A pointer to return the Roll angle in radians from -pi to pi
 * @return SUCCESS or FAIL
 */
void extract_angles(double dcm[MSZ][MSZ], double euler[MSZ]) {
    const double pi_2 = M_PI / 2;
    euler[0] = atan2(dcm[1][0], dcm[0][0]); /* Yaw */
    if (dcm[2][0] > 1.0) {
        euler[1] = -pi_2;
    } else if (dcm[2][0] < -1.0) {
        euler[1] = pi_2;
    } else {
        euler[1] = -asin(dcm[2][0]); /* Pitch */
    }

    euler[2] = atan2(dcm[2][1], dcm[2][2]); /* Roll */

}

void main(void) {
    uint8_t initResult;
    uint rate_return = 0;
    uint8_t index = 0;
    uint8_t row = 0;
    uint8_t col = 0;
 
    uint32_t current_time; //current time in microsec
    uint32_t start_time; // starting time in microsec
    uint32_t update_start; // start time of update step
    uint32_t update_end;  // ending time of update step 


    double dt = DT; //integration time in sec
    /****** Filter gains  ***************************/
    double kp_a = 2.5; //Accelerometer proportional gain 
    double ki_a = 0.1; // Accelerometer integral gain
    double kp_m = 2.5; //Magnetometer proportional gain 
    double ki_m = 0.1; //Magnetometer integral gain
    /************************************************/
    /*rad <--> deg conversion constants*/
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;

    /********data arrays*******************************************************/
    // gryo, accelerometer, magnetometer raw and calibrated vectors
    /* Raw Data arrays */
    int16_t acc_raw[MSZ];
    int16_t mag_raw[MSZ];
    int16_t gyro_raw[MSZ];
    double acc_raw_double[MSZ];
    double mag_raw_double[MSZ];
    int8_t temperature = 0;
    /* Calibrated data arrays */
    double gyro_cal[MSZ];
    double acc_tmp[MSZ]; // intermediate array
    double acc_cal[MSZ];
    double mag_cal[MSZ];
    double mag_tmp[MSZ]; // intermediate array

    /****** Calibration parameters ********************************************/
    /*gyroscope scaling to rad/sec*/
    double gyro_scale = (250.0 / ((1 << 15) - 1)) * deg2rad;
    /*Accelerometer calibration matrix*/
    double A_acc[MSZ][MSZ] = {
        0.00100495454726146, -0.0000216880122750632, 0.0000133999325710038,
        -0.0000162926752704191, 0.000985380021967908, 0.00000666633241783684,
        0.0000125438947528585, 0.00000252521314262081, 0.000989136500881465
    };
    /*Accelerometer offsets*/
    double b_acc[MSZ] = {0.00605446075713124, 0.0531371565285396, 0.0243733439531166};

    /*Magnetometer calibration matrix*/
    double A_mag[MSZ][MSZ] = {0.00135496706593374, 0.0000116105133187656, -0.00000832065758415854,
        0.00000781613708114319, 0.00137913779319635, 0.00000538838213067807,
        -0.0000126033738192070, 0.00000739989830409341, 0.00140611428756101};
    /*Magnetometer offsets*/
    double b_mag[MSZ] = {0.480254990338941, -0.286077906618463, 0.528755499855987};
    /**************************************************************************/

    // attitude DCMs
    double r_minus[MSZ][MSZ] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double r_plus[MSZ][MSZ] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // gyro bias vectors
    double b_minus[MSZ] = {0, 0, 0};
    double b_plus[MSZ] = {0, 0, 0};
    double euler[MSZ] = {0, 0, 0};

    /*******************Inertial aiding vectors *******************************/
    // Earth's magnetic field inertial vector, normalized 
    // North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
    // converted into ENU format and normalized:
    double m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

    // gravity in inertial frame
    double a_i[MSZ] = {0, 0, 1.0};
    /**************************************************************************/

    /* init pico */
    stdio_init_all();
    printf("Mahoney Filter using DCMs and matrix exponential from Matlab Coder\r\n");
    /* We use I2C0 on the SDA and SCL pico pins (16,17) */
    rate_return = i2c_init(i2c_default, 400 * 1000);
    printf("I2C rate set to %d \r\n", rate_return);
    gpio_set_function(PICO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C_SDA_PIN);
    gpio_pull_up(PICO_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_I2C_SDA_PIN, PICO_I2C_SCL_PIN, GPIO_FUNC_I2C));

    sleep_ms(2000);  //give chip time to settle
    initResult = BNO055_Init();
    if (initResult != SUCCESS) {
        printf("Initialization of IMU failed, stopping here.\r\n");
    }else {
        printf("Initialization succeeded!\r\n");
        start_time = time_us_32();
        while (1) {
            current_time = time_us_32();
            if ((current_time - start_time) % PERIOD == 0) {
                start_time = current_time; // reset period counter
                /* read sensors*/
                BNO055_ReadRaw(acc_raw, mag_raw, gyro_raw, &temperature);
                /* convert uint16_t to double for compatibility with Matlab code */
                for(index = 0; index < MSZ; index++){
                    acc_raw_double[index] = (double)acc_raw[index];
                    mag_raw_double[index] = (double)mag_raw[index];
                    gyro_cal[index] = (double) gyro_raw[index] * gyro_scale * deg2rad; // scale and convert gyro data into rad/s
                }
                /* calibrate sensors */
                m_v_mult(A_acc, acc_raw_double, acc_tmp); // scale accelerometer data 
                v_v_add(acc_tmp, b_acc, acc_cal); // offset accelerometer data
                m_v_mult(A_mag, mag_raw_double, mag_tmp); // scale magnetometer data
                v_v_add(mag_tmp, b_mag, mag_cal); // offset magnetometer data

                update_start = time_us_32(); //get timing of routine
                ahrs_m_update(r_minus, b_minus, gyro_cal, mag_cal, acc_cal, m_i, a_i, dt,
                    kp_a, ki_a, kp_m, ki_m, r_plus, b_plus);
                update_end = time_us_32();
                /* extract euler angles from DCM */
                extract_angles(r_plus, euler);
                /* print out data */
                printf("%+3.1f, %3.1f, %3.1f, ", euler[0] * rad2deg, euler[1] * rad2deg, euler[2] * rad2deg);
                printf("%f, %f, %f, ", b_plus[0], b_plus[1], b_plus[2]);
                printf("%d \r\n", update_end - update_start);
                /* update b_minus and r_minus */
                for (row = 0; row < MSZ; row++) {
                    for (col = 0; col < MSZ; col++) {
                        r_minus[row][col] = r_plus[row][col];
                        b_minus[col] = b_plus[col];
                    }
                }
            }
        }
    }
}




