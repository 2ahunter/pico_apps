/* 
 * File:   ahrs_main.c
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
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "BNO055.h" // The header file for the IMU

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
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
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/


void cross(double u[MSZ], double v[MSZ], double w_out[MSZ]);
void q_mult(double q[QSZ], double p[QSZ], double r[QSZ]);
void quat2euler(double q[QSZ], double euler[MSZ]) ;
void q_rot_v_q(double v_i[MSZ], double q[QSZ], double v_b[MSZ]);
double q_norm(double q[QSZ]);
void v_scale(double s, double v[MSZ]);
void v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]);
void v_v_sub(double v1[MSZ], double v2[MSZ], double v_out[MSZ]);
void v_copy(double m_in[MSZ], double m_out[MSZ]);
void m_v_mult(double m[MSZ][MSZ], double v[MSZ], double v_out[MSZ]);
void ahrs_update(double q_minus[QSZ], double q_plus[QSZ], double bias_minus[MSZ],
        double bias_plus[MSZ], double gyros[MSZ], double mags[MSZ], double accels[MSZ],
        double mag_i[MSZ], double acc_i[MSZ], double dt);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/
/**
 * @function cross()
 * @param u A vector
 * @param v A vector
 * @return The cross product (sometimes called the outter product) of u and v
 */
void cross(double u[MSZ], double v[MSZ], double w_out[MSZ]) {
    w_out[0] = u[1] * v[2] - u[2] * v[1];
    w_out[1] = u[2] * v[0] - u[0] * v[2];
    w_out[2] = u[0] * v[1] - u[1] * v[0];
}

/**
 * @function q_mult()
 * Multiply two quaternions together
 * @param p A quaternion
 * @param q A quaternion
 * @param r The resulting quaternion
 */
void q_mult(double q[QSZ], double p[QSZ], double r[QSZ]) {
    r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    r[1] = p[1] * q[0] + p[0] * q[1] + p[3] * q[2] - p[2] * q[3];
    r[2] = p[2] * q[0] - p[3] * q[1] + p[0] * q[2] + p[1] * q[3];
    r[3] = p[3] * q[0] + p[2] * q[1] - p[1] * q[2] + p[0] * q[3];
}
/**
 * @function quat2euler()
 * @param q A quaternion
 * @param euler a vector of euler angles in [psi, theta, roll] order
 */
void quat2euler(double q[QSZ], double euler[MSZ]) {
    double q00 = q[0] * q[0];
    double q11 = q[1] * q[1];
    double q22 = q[2] * q[2];
    double q33 = q[3] * q[3];

    // psi
    euler[0] = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), ((q00 + q11 - q22 - q33)));
    // theta
    euler[1] = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // phi
    euler[2] = atan2(2.0 * (q[2] * q[3] + q[0] * q[1]), q00 - q11 - q22 + q33);
}

/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
void q_rot_v_q(double v_i[MSZ], double q[QSZ], double v_b[MSZ]) {
    double q_i[QSZ];
    double q_temp[QSZ];
    double q_conj[QSZ];
    double q_b[QSZ]; // container for inertial vector in body frame as pure quaternion

    /* calculate conjugate of q */
    q_conj[0] = q[0];
    q_conj[1] = -q[1];
    q_conj[2] = -q[2];
    q_conj[3] = -q[3];
    /* convert v_i to a pure quaternion --> q_i */
    q_i[0] = 0;
    q_i[1] = v_i[0];
    q_i[2] = v_i[1];
    q_i[3] = v_i[2];
    /* first quaternion product q_i by q --> q_temp */
    q_mult(q_i, q, q_temp);
    /* second quaternion product q_conj by q_temp -->q_b */
    q_mult(q_conj, q_temp, q_b);
    /* set v_b to imaginary part of q_b */
    v_b[0] = q_b[1];
    v_b[1] = q_b[2];
    v_b[2] = q_b[3];
}

/**
 * @function q_norm()
 * @param q A quaternion
 * @return The magnitude of the quaternion, q
 */
double q_norm(double q[QSZ]) {
    return ((double) sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
}

/**
 * @function v_scale()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 */
void v_scale(double s, double v[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v[row] *= s;
    }
}

/**
 * @function v_v_add()
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
 * @function v_v_sub()
 * Add a vector value to a vector
 * @param v1 Vector to subtract to another vector
 * @param v2 Vector to have a vector subtracted to it
 * @param v_out Vector as difference of two vectors
 */
void v_v_sub(double v1[MSZ], double v2[MSZ], double v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] - v2[row];
    }
}

void v_copy(double m_in[MSZ], double m_out[MSZ]) {
    int row;
    for (row = 0; row < MSZ; row++) {
        m_out[row] = m_in[row];
    }
}

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

void ahrs_update(double q_minus[QSZ], double q_plus[QSZ], double bias_minus[MSZ],
    double bias_plus[MSZ], double gyros[MSZ], double mags[MSZ], double accels[MSZ],
    double mag_i[MSZ], double acc_i[MSZ], double dt) {
    double kp_a = 2.5; //accelerometer proportional gain
    double ki_a = 0.05; // accelerometer integral gain
    double kp_m = 2.5; // magnetometer proportional gain
    double ki_m = 0.05; //magnetometer integral gain

    double acc_b[MSZ]; //estimated gravity vector in body frame
    double mag_b[MSZ]; //estimated magnetic field vector in body frame

    double gyro_cal[MSZ]; // gyros with bias correction
    double gyro_wfb[MSZ]; // gyro 'rate' after feedback
    double w_meas_ap[MSZ]; // accelerometer proportion correction rate
    double w_meas_mp[MSZ]; // magnetometer proportional correction rate
    double w_meas_ai[MSZ]; // accelerometer integral correction rate
    double w_meas_mi[MSZ]; // magnetometer integral correction rate

    double gyro_q_wfb[QSZ]; // temporary quaternion to hold feedback term
    double q_dot[QSZ]; // quaternion derivative
    double b_dot[MSZ]; // bias vector derivative
    double q_n;


    /*Accelerometer attitude calculations */
    q_rot_v_q(acc_i, q_minus, acc_b); //estimate gravity vector in body frame 
    cross(accels, acc_b, w_meas_ap); // calculate the accelerometer rate term
    v_copy(w_meas_ap, w_meas_ai); // make a copy for the integral term
    v_scale(kp_a, w_meas_ap); // calculate the accelerometer proportional feedback term 
    v_scale(ki_a, w_meas_ai); // calculate the accelerometer integral feedback term 

    /*Magnetometer attitude calculations*/
    q_rot_v_q(mag_i, q_minus, mag_b); //estimate magnetic field vector in body frame
    cross(mags, mag_b, w_meas_mp); // calculate the magnetometer rate term
    v_copy(w_meas_mp, w_meas_mi); //make a copy for the integral term
    v_scale(kp_m, w_meas_mp); // calculate the magnetometer proportional feedback term
    v_scale(ki_m, w_meas_mi); // calculate the magnetometer integral feedback term

    /*Gyro attitude contributions */
    v_v_sub(gyros, bias_minus, gyro_cal); //correct the gyros with the b_minus vector

    /* calculate total rate term gyro_wfb */
    v_v_add(w_meas_ap, w_meas_mp, gyro_wfb);
    v_v_add(gyro_cal, gyro_wfb, gyro_wfb);

    /* convert feedback term to a pure quaternion */
    gyro_q_wfb[0] = 0;
    gyro_q_wfb[1] = gyro_wfb[0];
    gyro_q_wfb[2] = gyro_wfb[1];
    gyro_q_wfb[3] = gyro_wfb[2];

    /* compute the quaternion derivative q_dot */
    q_mult(q_minus, gyro_q_wfb, q_dot);

    /* integrate term by term */
    q_plus[0] = q_minus[0] + 0.5 * q_dot[0] * dt;
    q_plus[1] = q_minus[1] + 0.5 * q_dot[1] * dt;
    q_plus[2] = q_minus[2] + 0.5 * q_dot[2] * dt;
    q_plus[3] = q_minus[3] + 0.5 * q_dot[3] * dt;

    // normalize the quaternion for stability
    q_n = q_norm(q_plus);
    q_plus[0] = q_plus[0] / q_n;
    q_plus[1] = q_plus[1] / q_n;
    q_plus[2] = q_plus[2] / q_n;
    q_plus[3] = q_plus[3] / q_n;

    // compute the integral of the bias term by term
    bias_plus[0] = bias_minus[0] - (w_meas_ai[0] + w_meas_mi[0]) * dt;
    bias_plus[1] = bias_minus[1] - (w_meas_ai[1] + w_meas_mi[1]) * dt;
    bias_plus[2] = bias_minus[2] - (w_meas_ai[2] + w_meas_mi[2]) * dt;
}

int main(void) {
    uint8_t initResult;
    uint rate_return = 0;
    uint8_t index = 0;
 
    uint32_t current_time; //current time in microsec
    uint32_t start_time; // starting time in microsec
    uint32_t update_start; // start time of update step
    uint32_t update_end;  // ending time of update step 

    /* Matrix and Quaternion arrays */
    const double dt = DT; // integration interval
    const double gyro_scale = 250.0 / (double) ((1 << 15) - 1);
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;
    /*Calibration matrices and offsets from tumble test */
    double A_acc[MSZ][MSZ] = {
        0.00100495454726146, -0.0000216880122750632, 0.0000133999325710038,
        -0.0000162926752704191, 0.000985380021967908, 0.00000666633241783684,
        0.0000125438947528585, 0.00000252521314262081, 0.000989136500881465
    };
    double b_acc[MSZ] = {0.00605446075713124, 0.0531371565285396, 0.0243733439531166};

    double A_mag[MSZ][MSZ] = {0.00135496706593374, 0.0000116105133187656, -0.00000832065758415854,
        0.00000781613708114319, 0.00137913779319635, 0.00000538838213067807,
        -0.0000126033738192070, 0.00000739989830409341, 0.00140611428756101};

    double b_mag[MSZ] = {0.480254990338941, -0.286077906618463, 0.528755499855987};

    // gravity inertial vector
    double a_i[MSZ] = {0, 0, 1.0};
    // Earth's magnetic field inertial vector, normalized 
    // North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
    // converted into ENU format and normalized:
    double m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

    // Euler angles
    double euler[MSZ] = {0, 0, 0};

    // attitude quaternions
    double q_minus[QSZ] = {1, 0, 0, 0};
    double q_plus[QSZ] = {1, 0, 0, 0};
    // gyro bias vector
    double b_minus[MSZ] = {0, 0, 0};
    double b_plus[MSZ] = {0, 0, 0};

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

    /* init pico */
    stdio_init_all();
    printf("AHRS evaluation application, compiled " __DATE__ " " __TIME__ "\r\n");
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

        start_time = time_us_32();
        while(1){
            current_time = time_us_32();
            if((current_time - start_time) % PERIOD == 0){
                start_time = current_time; // reset period counter
                /* read sensors*/
                BNO055_ReadRaw(acc_raw, mag_raw, gyro_raw, &temperature);
                /* convert uint16_t to double for compatibility with linear algebra library */
                for(index = 0; index < MSZ; index++){
                    acc_raw_double[index] = (double)acc_raw[index];
                    mag_raw_double[index] = (double)mag_raw[index];
                    gyro_cal[index] = (double) gyro_raw[index] * gyro_scale * deg2rad; // scale and convert gyro data into rad/s
                }
                /* calibrate sensors */
                m_v_mult(A_acc, acc_raw_double, acc_tmp); // scale and calibrate acc             
                v_v_add(acc_tmp, b_acc, acc_cal); // offset accelerometer data              
                m_v_mult(A_mag, mag_raw_double, mag_tmp); // scale magnetometer data                 
                v_v_add(mag_tmp, b_mag, mag_cal); // offset magnetometer data

                update_start = time_us_32();
                ahrs_update(q_minus, q_plus, b_minus, b_plus, gyro_cal, mag_cal, acc_cal, m_i, a_i, dt);
                update_end = time_us_32();
                quat2euler(q_plus, euler); // convert quaternion to euler angles

                /* print out data */
                printf("%+3.1f, %3.1f, %3.1f, ", euler[0] * rad2deg, euler[1] * rad2deg, euler[2] * rad2deg);
                printf("%1.3e, %1.3e, %1.3e,", b_plus[0], b_plus[1], b_plus[2]);
                printf("%d\r\n", update_end - update_start);
                /* update b_minus and q_minus */
                b_minus[0] = b_plus[0];
                b_minus[1] = b_plus[1];
                b_minus[2] = b_plus[2];
                q_minus[0] = q_plus[0];
                q_minus[1] = q_plus[1];
                q_minus[2] = q_plus[2];
                q_minus[3] = q_plus[3];
            }
        }
    }
}