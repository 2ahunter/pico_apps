/*
 * File:   lin_alg_float.c
 * Author: Pavlo Vlastos
 * Acknowledgements:
 *     Some code is based on CMPE13 (now CSE/E 13) and Chris Seruge's Matrix 
 *     libraries
 * Created on February 6, 2020, 11:28 AM
 */

/*******************************************************************************
 * #INCLUDES
 ******************************************************************************/
#include "lin_alg_float.h"
#include <math.h>
#include <stdio.h>

/*******************************************************************************
 * PRIVATE #DEFINES
 ******************************************************************************/
#define RES 0.0001

/*******************************************************************************
 * PRIVATE DATATYPES 
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE VARIABLES 
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

///**
// * @function correct()
// * Correct scaling and offset of a 3-component measurement by the accelerometer
// * or magnetometer
// * @param v_measure A vector array to hold accelerometer or magnetometer 
// *        [x y z]^T  measurement
// * @param a_est The scale factor matrix correction from tumble test. See 
// *        CalibrateEllipsoidData3D.m in repo
// * @param b_est The offset vector correction from tumble test. See 
// *        CalibrateEllipsoidData3D.m in repo
// * @param v_correct The corrected vector measurement
// * @return SUCCESS or ERROR
// */
//char correct(float v_measure[MSZ], float a_est[MSZ][MSZ], float b_est[MSZ],
//        float v_correct[MSZ]) {
//
//    lin_alg_m_v_mult(a_est, v_measure, v_correct);
//    lin_alg_v_v_add(v_correct, b_est, v_correct);
//
//}

/**
 * @function lin_alg_is_m_equal()
 * Compares to see if two matrices are equal to within the resolution (RES)
 * @param m1 A 3x3 matrix
 * @param m2 A 3x3 matrix
 * @return TRUE if matrices are equal, FALSE otherwise
 */
char lin_alg_is_m_equal(float m1[MSZ][MSZ], float m2[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            if (fabs(m1[row][col] - m2[row][col]) > RES) {
                return FALSE;
            }
        }
    }

    return TRUE;
}

/**
 * @function lin_alg_m_m_mult()
 * Multiplies two matricies together
 * @param m1 First matrix to be multiplied with second
 * @param m2 Second matrix to be multiplied with first
 * @param m_out The product of the two matrices.
 */
char lin_alg_m_m_mult(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]) {
    int row;
    int col;
    int i_sum;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = 0;
            for (i_sum = 0; i_sum < MSZ; i_sum++) {
                m_out[row][col] += (m1[row][i_sum] * m2[i_sum][col]);
            }
        }
    }
    return SUCCESS;
}

/**
 * @function lin_alg_m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
char lin_alg_m_v_mult(float m[MSZ][MSZ], float v[MSZ], float v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
    return SUCCESS;
}

/**
 * @function lin_alg_is_v_equal()
 * Compares to see if two vectors are equal to within the resolution (RES)
 * @param v1
 * @param v2
 * @return TRUE if equal and FALSE otherwise
 */
char lin_alg_is_v_equal(float v1[MSZ], float v2[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        if (fabs(v1[row] - v2[row]) > RES) {
            return FALSE;
        }
    }

    return TRUE;
}

/**
 * @function lin_alg_set_m()
 * @param matrixXX all respective entries to a 3x3 matrix
 * @param m The matrix for which each entry is set.
 */
void lin_alg_set_m(float matrix11, float matrix12, float matrix13,
        float matrix21, float matrix22, float matrix23, float matrix31,
        float matrix32, float matrix33, float m[MSZ][MSZ]) {

    m[0][0] = matrix11;
    m[0][1] = matrix12;
    m[0][2] = matrix13;

    m[1][0] = matrix21;
    m[1][1] = matrix22;
    m[1][2] = matrix23;

    m[2][0] = matrix31;
    m[2][1] = matrix32;
    m[2][2] = matrix33;
}

/**
 * @function lin_alg_set_v()
 * @param vectorX all respective entries to a 3x1 vector
 * @param v The vector for which each entry is set.
 */
void lin_alg_set_v(float vector1, float vector2, float vector3, float v[MSZ]) {
    v[0] = vector1;
    v[1] = vector2;
    v[2] = vector3;
}

/**
 * @function lin_alg_s_m_mult()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 * @param m_out Scaled matrix
 */
void lin_alg_s_m_mult(float s, float m[MSZ][MSZ], float m_out[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = s * m[row][col];
        }
    }
}

/**
 * @function lin_alg_m_scale()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
void lin_alg_m_scale(float s, float m[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m[row][col] *= s;
        }
    }
}

/**
 * @function lin_alg_s_v_mult()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 * @param v_out
 */
void lin_alg_s_v_mult(float s, float v[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = s * v[row];
    }
}

/**
 * @function lin_alg_v_scale()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 */
void lin_alg_v_scale(float s, float v[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v[row] *= s;
    }
}

/**
 * @function lin_alg_s_m_mult()
 * Add a scalar value to a matrix
 * @param s Scalar to add to a matrix
 * @param m Matrix to have a scalar added to each element
 * @param m_out Matrix with added scalar
 */
void lin_alg_s_m_add(float s, float m[MSZ][MSZ], float m_out[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = s + m[row][col];
        }
    }
}

/**
 * @function lin_alg_s_v_add()
 * Add a scalar value to a vector
 * @param s Scalar to add to a vector
 * @param v Vector to have a scalar added to each element
 * @param v_out Vector with added scalar
 */
void lin_alg_s_v_add(float s, float v[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = s + v[row];
    }
}

/**
 * @function lin_alg_m_m_add()
 * Add a matrix to a matrix
 * @param m1 Matrix to add to another matrix
 * @param m2 Matrix to have a matrix added to it
 * @param m_out Matrix as sum of two matrices
 */
void lin_alg_m_m_add(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]) {

    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = m1[row][col] + m2[row][col];
        }
    }
}

/**
 * @function lin_alg_m_m_sub()
 * Add a matrix to a matrix
 * @param m1 Matrix to subtract to another matrix
 * @param m2 Matrix to have a matrix subtracted to it
 * @param m_out Matrix as difference of two matrices
 */
void lin_alg_m_m_sub(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]) {

    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = m1[row][col] - m2[row][col];
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
void lin_alg_v_v_add(float v1[MSZ], float v2[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function lin_alg_v_v_sub()
 * Add a vector value to a vector
 * @param v1 Vector to subtract to another vector
 * @param v2 Vector to have a vector subtracted to it
 * @param v_out Vector as difference of two vectors
 */
void lin_alg_v_v_sub(float v1[MSZ], float v2[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] - v2[row];
    }
}

/**
 * @function lin_alg_m_trace()
 * Sums the diagonals of a matrix. This sum is known as the trace.
 * @param m The matrix to have its trace calculated
 * @return The trace of the matrix.
 */
float lin_alg_m_trace(float m[MSZ][MSZ]) {
    float trace = 0;
    int i;

    for (i = 0; i < MSZ; i++) {
        trace += m[i][i];
    }

    return trace;
}

/**
 * @function lin_alg_m_det()
 * @param m The matrix for which a determinant is to be calculated
 * @return The determinant of the matrix
 */
float lin_alg_m_det(float m[MSZ][MSZ]) {
    float det = 0;
    det = m[0][0] * ((m[1][1] * m[2][2]) - (m[1][2] * m[2][1]))
            - m[0][1] * ((m[1][0] * m[2][2]) - (m[1][2] * m[2][0]))
            + m[0][2] * ((m[1][0] * m[2][1]) - (m[1][1] * m[2][0]));
    return det;
}

/**
 * @function lin_alg_m_transpose()
 * Generates the transpose of a matrix
 * @param m The matrix to be transposed
 * @param m_out The transpose of m
 */
void lin_alg_m_transpose(float m[MSZ][MSZ], float m_out[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = m[col][row];
        }
    }
}

/**
 * @function lin_alg_skew_sym()
 * Generate the skew symmetric matrix of a (3x1) vector 
 * @param v A vector
 * @param m_out The resulting (3x3) skew matrix of v
 */
void lin_alg_skew_sym(float v[MSZ], float m_out[MSZ][MSZ]) {
    m_out[0][0] = 0.0;
    m_out[1][1] = 0.0;
    m_out[2][2] = 0.0;

    m_out[2][1] = v[0];
    m_out[2][0] = -v[1];
    m_out[1][0] = v[2];

    m_out[1][2] = -v[0];
    m_out[0][2] = v[1];
    m_out[0][1] = -v[2];
}

/**
 * @function lin_alg_anti_skew_pro()
 * Forms the antisymmetric projection matrix
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param dcm_t The transpose of the dcm
 * @param p_out The antisymmetric projection matrix
 */
void lin_alg_anti_sym_pro(float dcm[MSZ][MSZ], float dcm_t[MSZ][MSZ],
        float p_out[MSZ][MSZ]) {

    lin_alg_m_m_sub(dcm, dcm_t, p_out);

    lin_alg_m_scale(0.5, p_out);
}

/**
 * @function lin_alg_vex()
 * Change a skew matrix to a vector
 * @param m A skew matrix
 * @param v_out The corresponding vector
 */
void lin_alg_vex(float m[MSZ][MSZ], float v_out[MSZ]) {
    v_out[0] = m[2][1];
    v_out[1] = m[0][2];
    v_out[2] = m[1][0];
}

/**
 * @function lin_alg_v_norm()
 * Calculate the 2-norm, also known as the Euclidean length, or vector magnitude
 * @param v The vector for which the norm is to be calculated
 * @return The norm of v
 */
float lin_alg_v_norm(float v[MSZ]) {
    return ((float) sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2])));
}

/**
 * @function lin_alg_dot()
 * @param u A vector
 * @param v A vector
 * @return The dot product (sometimes called the inner product) of u and v
 */
float lin_alg_dot(float u[MSZ], float v[MSZ]) {
    return (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]);
}

/**
 * @function lin_alg_cross()
 * @param u A vector
 * @param v A vector
 * @return The cross product (sometimes called the outter product) of u and v
 */
void lin_alg_cross(float u[MSZ], float v[MSZ], float w_out[MSZ]) {
    w_out[0] = u[1] * v[2] - u[2] * v[1];
    w_out[1] = u[2] * v[0] - u[0] * v[2];
    w_out[2] = u[0] * v[1] - u[1] * v[0];
}

/**
 * @function lin_alg_angle_from_2vecs()
 * @param u A vector
 * @param v A vector
 * @return The angle between the two vectors
 */
float lin_alg_angle_from_2vecs(float u[MSZ], float v[MSZ]) {
    return acos(lin_alg_dot(u, v) / (lin_alg_v_norm(u) * lin_alg_v_norm(v)));
}

/**
 * @function lin_alg_gen_dcm();
 * Generates a Direction Cosine Matrix for a rotation about a given axis and a
 * given angle. See Jack B. Kuibers book: "Quaternions and Rotation Sequences"
 * @param angle The angle of rotation about the axis of rotation unit-vector
 * @param v The unit-vector representing the axis of rotation
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @return SUCCESS or FAIL
 */
char lin_alg_gen_dcm(float angle, float v[MSZ], float dcm[MSZ][MSZ]) {

    // Check if the axis vector is a unit-vector
    if (fabs(lin_alg_v_norm(v) - 1.0) > RES) {
        return ERROR;
    }

    float ca = cos(angle);
    float sa = sin(angle);

    dcm[0][0] = v[0] * v[0] + (v[1] * v[1] + v[2] * v[2]) * ca;
    dcm[0][1] = v[0] * v[1]*(1 - ca) - v[2] * sa;
    dcm[0][2] = v[0] * v[2]*(1 - ca) + v[1] * sa;

    dcm[1][0] = v[0] * v[1]*(1 - ca) + v[2] * sa;
    dcm[1][1] = v[1] * v[1] + (v[2] * v[2] + v[0] * v[0]) * ca;
    dcm[1][2] = v[1] * v[2]*(1 - ca) - v[0] * sa;

    dcm[2][0] = v[2] * v[0]*(1 - ca) - v[1] * sa;
    dcm[2][1] = v[1] * v[2]*(1 - ca) + v[0] * sa;
    dcm[2][2] = v[2] * v[2] + (v[0] * v[0] + v[1] * v[1]) * ca;

    return SUCCESS;
}

/**
 * @function lin_alg_gen_dcm_with_angles();
 * Generates a Direction Cosine Matrix for a rotation about a given axis and a
 * given angle. See Jack B. Kuibers book: "Quaternions and Rotation Sequences"
 * @param psi Yaw angle in radians from -pi to pi
 * @param theta Pitch angle in radians from -pi to pi
 * @param phi Roll angle in radians from -pi to pi
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 */
void lin_alg_gen_dcm_with_angles(float psi, float theta, float phi,
        float dcm[MSZ][MSZ]) {
    float c_psi = cos(psi);
    float c_theta = cos(theta);
    float c_phi = cos(phi);

    float s_psi = sin(psi);
    float s_theta = sin(theta);
    float s_phi = sin(phi);

    dcm[0][0] = c_psi*c_theta;
    dcm[0][1] = s_psi*c_theta;
    dcm[0][2] = -s_theta;

    dcm[1][0] = c_psi * s_theta * s_phi - s_psi * c_phi;
    dcm[1][1] = s_psi * s_theta * s_phi + c_psi * c_phi;
    dcm[1][2] = c_theta * s_phi;

    dcm[2][0] = c_psi * s_theta * c_phi + s_psi * s_phi;
    dcm[2][1] = s_psi * s_theta * c_phi - c_psi * s_phi;
    dcm[2][2] = c_theta * c_phi;
}

/**
 * @function lin_alg_extract_angles();
 * Extract Euler angles from the DCM
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param psi A pointer to return the Yaw angle in radians from -pi to pi
 * @param theta A pointer to return the Pitch angle in radians from -pi to pi
 * @param phi A pointer to return the Roll angle in radians from -pi to pi
 * @return SUCCESS or FAIL
 */
char lin_alg_extract_angles(float dcm[MSZ][MSZ], float *psi, float *theta,
        float *phi) {
    *psi = atan2(dcm[1][0], dcm[0][0]); /* Yaw */

    *theta = asin(-dcm[2][0]); /* Pitch */

    *phi = atan2(dcm[2][1], dcm[2][2]); /* Roll */

    return SUCCESS;
}

/**
 * @function lin_alg_set_q()
 * @param psi Yaw angle in radians from -pi to pi
 * @param theta Pitch angle in radians from -pi to pi
 * @param phi Roll angle in radians from -pi to pi
 * @param q_out
 */
void lin_alg_set_q(float psi, float theta, float phi, float q_out[QSZ]) {
    float c_psi_2 = cos(psi / 2.0);
    float c_theta_2 = cos(theta / 2.0);
    float c_phi_2 = cos(phi / 2.0);

    float s_psi_2 = sin(psi / 2.0);
    float s_theta_2 = sin(theta / 2.0);
    float s_phi_2 = sin(phi / 2.0);

    q_out[0] = (c_psi_2 * c_theta_2 * c_phi_2 + s_psi_2 * s_theta_2 * s_phi_2);
    q_out[1] = (c_psi_2 * c_theta_2 * s_phi_2 - s_psi_2 * s_theta_2 * c_phi_2);
    q_out[2] = (c_psi_2 * s_theta_2 * c_phi_2 + s_psi_2 * c_theta_2 * s_phi_2);
    q_out[3] = (s_psi_2 * c_theta_2 * c_phi_2 - c_psi_2 * s_theta_2 * s_phi_2);
}

/**
 * @function lin_alg_q_inv()
 * Form the complex conjugate 
 * @param q_in A quaternion
 * @param q_out The complex conjugate, or inverse of q_in
 */
void lin_alg_q_inv(float q_in[QSZ], float q_out[QSZ]) {
    q_out[0] = q_in[0];
    q_out[1] = -q_in[1];
    q_out[2] = -q_in[2];
    q_out[3] = -q_in[3];
}

/**
 * @function lin_alg_scale_q()
 * Scale a quaternion
 * @param s A scalar
 * @param q A quaternion
 */
void lin_alg_scale_q(float s, float q[QSZ]) {
    q[0] *= s;
    q[1] *= s;
    q[2] *= s;
    q[3] *= s;
}

/**
 * @function lin_alg_q_norm()
 * @param q A quaternion
 * @return The magnitude of the quaternion, q
 */
float lin_alg_q_norm(float q[QSZ]) {
    return ((float) sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
}

/**
 * @function lin_alg_q_mult()
 * Multiply two quaternions together
 * @param p A quaternion
 * @param q A quaternion
 * @param r The resulting quaternion
 */
void lin_alg_q_mult(float q[QSZ], float p[QSZ], float r[QSZ]) {
    r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    r[1] = p[1] * q[0] + p[0] * q[1] + p[3] * q[2] - p[2] * q[3];
    r[2] = p[2] * q[0] - p[3] * q[1] + p[0] * q[2] + p[1] * q[3];
    r[3] = p[3] * q[0] + p[2] * q[1] - p[1] * q[2] + p[0] * q[3];
}

/**
 * @function lin_alg_q2dcm()
 * Form a DCM from a quaternion
 * @param q A quaternion
 * @param dcm_out A direction cosine matrix (DCM)
 */
void lin_alg_q2dcm(float q[QSZ], float dcm_out[MSZ][MSZ]) {
    dcm_out[0][0] = 2.0 * q[0] * q[0] - 1.0 + 2.0 * q[1] * q[1];
    dcm_out[0][1] = 2.0 * q[1] * q[2] + 2.0 * q[0] * q[3];
    dcm_out[0][2] = 2.0 * q[1] * q[3] - 2.0 * q[0] * q[2];

    dcm_out[1][0] = 2.0 * q[1] * q[2] - 2.0 * q[0] * q[3];
    dcm_out[1][1] = 2.0 * q[0] * q[0] - 1.0 + 2.0 * q[2] * q[2];
    dcm_out[1][2] = 2.0 * q[2] * q[3] + 2.0 * q[0] * q[1];

    dcm_out[2][0] = 2.0 * q[1] * q[3] + 2.0 * q[0] * q[2];
    dcm_out[2][1] = 2.0 * q[2] * q[3] - 2.0 * q[0] * q[1];
    dcm_out[2][2] = 2.0 * q[0] * q[0] - 1.0 + 2.0 * q[3] * q[3];
}

/**
 * @function lin_alg_q2euler()
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param q A quaternion
 */
void lin_alg_q2euler(float q[QSZ], float *psi, float *theta, float *phi) {
    *psi = atan((2.0 * q[1] * q[2] + 2.0 * q[0] * q[3]) /
            ((2.0 * q[0] * q[0] + 2.0 * q[1] * q[1] - 1.0)));

    *theta = asin(-(2.0 * q[1] * q[3] - 2.0 * q[0] * q[2]));

    *phi = atan((2.0 * q[2] * q[3] + 2.0 * q[0] * q[1]) /
            (2.0 * q[0] * q[0] + 2.0 * q[3] * q[3] - 1.0));
}

/**
 * @function lin_alg_q2euler_abs()
 * Return absolute Euler angles
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param q A quaternion
 */
void lin_alg_q2euler_abs(float q[QSZ], float *psi, float *theta, float *phi) {
    *psi = atan2((2.0 * q[1] * q[2] + 2.0 * q[0] * q[3]),
            ((2.0 * q[0] * q[0] + 2.0 * q[1] * q[1] - 1.0)));

    *theta = asin(-(2.0 * q[1] * q[3] - 2.0 * q[0] * q[2]));

    *phi = atan2((2.0 * q[2] * q[3] + 2.0 * q[0] * q[1]),
            (2.0 * q[0] * q[0] + 2.0 * q[3] * q[3] - 1.0));
}

/**
 * @function lin_alg_rot_v_q()
 * Rotate a vector using quaternions
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param v_new
 */
void lin_alg_rot_v_q(float v[MSZ], float psi, float theta, float phi,
        float v_new[MSZ]) {
    float v_pure[QSZ];
    float v_temp[MSZ];
    float q[MSZ];
    float q_conj[MSZ];

    v_pure[0] = 0.0;
    v_pure[1] = v[0];
    v_pure[2] = v[1];
    v_pure[3] = v[2];

    lin_alg_set_q(psi, theta, phi, q);
    lin_alg_q_inv(q, q_conj);
    lin_alg_q_mult(q_conj, v_pure, v_temp);
    lin_alg_q_mult(v_temp, q, v_pure);

    v_new[0] = v_pure[1];
    v_new[1] = v_pure[2];
    v_new[2] = v_pure[3];
}

/**
 * @function lin_alg_m_print()
 * Print a matrix
 * @param matrix A matrix to print out
 */
void lin_alg_m_print(float matrix[MSZ][MSZ]) {
    int row;
    int col;

    printf("\r\n");
    for (row = 0; row < MSZ; row++) {
        printf("\r\n");
        for (col = 0; col < MSZ; col++) {
            printf("    % -9.4f", (double) matrix[row][col]);
        }
    }
    printf("\r\n");
}

/**
 * @function lin_alg_v_print()
 * Print a vector
 * @param vector A vector to print out
 */
void lin_alg_v_print(float vector[MSZ]) {
    int row;

    printf("\r\n");
    for (row = 0; row < MSZ; row++) {
        printf("\r\n    % -9.4f", (double) vector[row]);
    }
    printf("\r\n");
}