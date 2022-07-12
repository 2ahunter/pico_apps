/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ahrs_q_update.c
 *
 * Code generation for function 'ahrs_q_update'
 *
 */

/* Include files */
#include "ahrs_q_update.h"
#include <math.h>

/* Function Definitions */
/*
 * function [Qplus, Bplus] = ahrs_q_update(Qminus, Bminus, gyros, mags,...
 *     accels, magInertial, accelInertial, deltaT, Kp_a, Ki_a, Kp_m, Ki_m)
 */
void ahrs_q_update(const double Qminus[4], const double Bminus[3], const double
                   gyros[3], double mags[3], double accels[3], double
                   magInertial[3], double accelInertial[3], double deltaT,
                   double Kp_a, double Ki_a, double Kp_m, double Ki_m, double
                   Qplus[4], double Bplus[3])
{
  double dv[9];
  double gyroInputWithFeedback[3];
  double wmeas_m[3];
  double absxk;
  double q_conj_idx_2;
  double q_i_idx_3;
  double scale;
  double t;
  double y;
  int i;

  /*  function [Qplus, Bplus] = ahrs_q update((Qminus, Bminus, gyros, mags,... */
  /*  accels, magInertial, accelInertial, deltaT, Kp_a, Ki_a, Kp_m, Ki_m) */
  /*  */
  /*  Function to implement the full complementary estimate and integration of */
  /*  gyros for full attitude estimation using an accelerometer and */
  /*  magnetometer feedback. */
  /*  */
  /*  Inputs: Previous attitute quaternion (Qminus) */
  /*          Previous bias estimate (Bminus) */
  /*          Body Fixed Rotation rates ([p;q;r]) in rad/s (gyros) */
  /*          Magnetometer Readings in body coordinates (mags) */
  /*          Accelerometer Readings in body coordinates (accels) */
  /*          Inertial reference magnetic field (magInertial) */
  /*          Inertial reference gravity field (accelInertial) */
  /*          Time between samples (deltaT) in seconds */
  /*          Accelerometer proportional gain (Kp_a) */
  /*          Accelerometer integral gain (Ki_a) */
  /*          Magnetometer proportional gain (Kp_m) */
  /*          Magnetometer integral gain (Ki_m) */
  /*  */
  /*  Outputs: New attitude quaternion (Qplus) */
  /*           New Gyro Bias (Bplus) */
  /*  set inertial measurements to unit vectors */
  /* 'ahrs_q_update:28' accels = accels/norm(accels); */
  scale = 3.3121686421112381E-170;
  absxk = fabs(accels[0]);
  if (absxk > 3.3121686421112381E-170) {
    q_i_idx_3 = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    q_i_idx_3 = t * t;
  }

  absxk = fabs(accels[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  absxk = fabs(accels[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  q_i_idx_3 = scale * sqrt(q_i_idx_3);

  /* 'ahrs_q_update:29' mags = mags/norm(mags); */
  scale = 3.3121686421112381E-170;
  accels[0] /= q_i_idx_3;
  absxk = fabs(mags[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  accels[1] /= q_i_idx_3;
  absxk = fabs(mags[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  accels[2] /= q_i_idx_3;
  absxk = fabs(mags[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);

  /*  set inertial reference vectors to unit vectors */
  /* 'ahrs_q_update:32' magInertial = magInertial/norm(magInertial); */
  scale = 3.3121686421112381E-170;
  mags[0] /= y;
  absxk = fabs(magInertial[0]);
  if (absxk > 3.3121686421112381E-170) {
    q_i_idx_3 = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    q_i_idx_3 = t * t;
  }

  mags[1] /= y;
  absxk = fabs(magInertial[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  mags[2] /= y;
  absxk = fabs(magInertial[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  q_i_idx_3 = scale * sqrt(q_i_idx_3);

  /* 'ahrs_q_update:33' accelInertial = accelInertial/norm(accelInertial); */
  scale = 3.3121686421112381E-170;
  magInertial[0] /= q_i_idx_3;
  absxk = fabs(accelInertial[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  magInertial[1] /= q_i_idx_3;
  absxk = fabs(accelInertial[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  magInertial[2] /= q_i_idx_3;
  absxk = fabs(accelInertial[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  accelInertial[0] /= y;
  accelInertial[1] /= y;
  accelInertial[2] /= y;

  /*  rotate inertial vectors to body frame */
  /* 'ahrs_q_update:36' v_ai_est = q_rotate(Qminus,accelInertial); */
  /* 'ahrs_q_update:37' v_mi_est = q_rotate(Qminus,magInertial); */
  /*  subtract estimated gyro bias from gyro reading */
  /* 'ahrs_q_update:40' gyroInputWithBias = gyros - Bminus; */
  /*  accelerometer correction in the body frame */
  /* 'ahrs_q_update:42' wmeas_a = rcross(accels) * v_ai_est; */
  /*  function rx = rcross(r) */
  /*  forms the skew symmetric x-product matrix of a 3x1 vector */
  /* 'rcross:4' rx=[0    -r(3)  r(2); */
  /* 'rcross:5'     r(3)  0    -r(1); */
  /* 'rcross:6'    -r(2)  r(1)  0]; */
  /*  q_rotate(q,v_i) rotates the inertial aiding vector into the body frame */
  /*  through rotation quaternion q */
  /*  convert to quaternion */
  /* 'q_rotate:5' q_i = r2p(v_i); */
  /* 'r2p:2' P = [0,r(1),r(2),r(3)]'; */
  /*  first quaternion product q_inertial by q */
  /* 'q_rotate:7' q_i = q_mult(q_i,q); */
  /* returns the hamilton product of two quaternions */
  /*    returns the hamiton product of two quaternions q,p in the [qw,qx,qy,qz] */
  /*    convention */
  /* 'q_mult:5' pw = p(1); */
  /* 'q_mult:6' px = p(2); */
  /* 'q_mult:7' py = p(3); */
  /* 'q_mult:8' pz = p(4); */
  /* 'q_mult:9' qw = q(1); */
  /* 'q_mult:10' qx = q(2); */
  /* 'q_mult:11' qy = q(3); */
  /* 'q_mult:12' qz = q(4); */
  /* 'q_mult:14' pq = [(pw*qw - px*qx -py*qy - pz*qz); */
  /* 'q_mult:15'     (pw*qx + px*qw +py*qz - pz*qy); */
  /* 'q_mult:16'     (pw*qy - px*qz + py*qw + pz*qx); */
  /* 'q_mult:17'     (pw*qz + px*qy - py*qx + pz*qw)]; */
  scale = ((0.0 * Qminus[0] - accelInertial[0] * Qminus[1]) - accelInertial[1] *
           Qminus[2]) - accelInertial[2] * Qminus[3];
  absxk = ((0.0 * Qminus[1] + accelInertial[0] * Qminus[0]) + accelInertial[1] *
           Qminus[3]) - accelInertial[2] * Qminus[2];
  t = ((0.0 * Qminus[2] - accelInertial[0] * Qminus[3]) + accelInertial[1] *
       Qminus[0]) + accelInertial[2] * Qminus[1];
  q_i_idx_3 = ((0.0 * Qminus[3] + accelInertial[0] * Qminus[2]) - accelInertial
               [1] * Qminus[1]) + accelInertial[2] * Qminus[0];

  /*  second quaternion product Qminus_conj by q_temp */
  /* 'q_rotate:9' q_b = q_mult(q_conj(q),q_i); */
  /* q_conj() returns the quaternion conjugate */
  /*    returns the conjugate of a quaternion in  the [qw, qx, qy, qz]' */
  /*    convention */
  /* 'q_conj:6' q_conj = [q(1) -q(2) -q(3) -q(4)]'; */
  /* returns the hamilton product of two quaternions */
  /*    returns the hamiton product of two quaternions q,p in the [qw,qx,qy,qz] */
  /*    convention */
  /* 'q_mult:5' pw = p(1); */
  /* 'q_mult:6' px = p(2); */
  /* 'q_mult:7' py = p(3); */
  /* 'q_mult:8' pz = p(4); */
  /* 'q_mult:9' qw = q(1); */
  /* 'q_mult:10' qx = q(2); */
  /* 'q_mult:11' qy = q(3); */
  /* 'q_mult:12' qz = q(4); */
  /* 'q_mult:14' pq = [(pw*qw - px*qx -py*qy - pz*qz); */
  /* 'q_mult:15'     (pw*qx + px*qw +py*qz - pz*qy); */
  /* 'q_mult:16'     (pw*qy - px*qz + py*qw + pz*qx); */
  /* 'q_mult:17'     (pw*qz + px*qy - py*qx + pz*qw)]; */
  /* 'q_rotate:10' v_b = q_b(2:end); */
  y = ((Qminus[0] * absxk + -Qminus[1] * scale) + -Qminus[2] * q_i_idx_3) -
    -Qminus[3] * t;
  q_conj_idx_2 = ((Qminus[0] * t - -Qminus[1] * q_i_idx_3) + -Qminus[2] * scale)
    + -Qminus[3] * absxk;
  scale = ((Qminus[0] * q_i_idx_3 + -Qminus[1] * t) - -Qminus[2] * absxk) +
    -Qminus[3] * scale;
  dv[0] = 0.0;
  dv[3] = -accels[2];
  dv[6] = accels[1];
  dv[1] = accels[2];
  dv[4] = 0.0;
  dv[7] = -accels[0];
  dv[2] = -accels[1];
  dv[5] = accels[0];
  dv[8] = 0.0;
  for (i = 0; i < 3; i++) {
    Bplus[i] = (dv[i] * y + dv[i + 3] * q_conj_idx_2) + dv[i + 6] * scale;
  }

  /*  magnetometer correction in the body frame */
  /* 'ahrs_q_update:44' wmeas_m = rcross(mags) * v_mi_est; */
  /*  function rx = rcross(r) */
  /*  forms the skew symmetric x-product matrix of a 3x1 vector */
  /* 'rcross:4' rx=[0    -r(3)  r(2); */
  /* 'rcross:5'     r(3)  0    -r(1); */
  /* 'rcross:6'    -r(2)  r(1)  0]; */
  /*  q_rotate(q,v_i) rotates the inertial aiding vector into the body frame */
  /*  through rotation quaternion q */
  /*  convert to quaternion */
  /* 'q_rotate:5' q_i = r2p(v_i); */
  /* 'r2p:2' P = [0,r(1),r(2),r(3)]'; */
  /*  first quaternion product q_inertial by q */
  /* 'q_rotate:7' q_i = q_mult(q_i,q); */
  /* returns the hamilton product of two quaternions */
  /*    returns the hamiton product of two quaternions q,p in the [qw,qx,qy,qz] */
  /*    convention */
  /* 'q_mult:5' pw = p(1); */
  /* 'q_mult:6' px = p(2); */
  /* 'q_mult:7' py = p(3); */
  /* 'q_mult:8' pz = p(4); */
  /* 'q_mult:9' qw = q(1); */
  /* 'q_mult:10' qx = q(2); */
  /* 'q_mult:11' qy = q(3); */
  /* 'q_mult:12' qz = q(4); */
  /* 'q_mult:14' pq = [(pw*qw - px*qx -py*qy - pz*qz); */
  /* 'q_mult:15'     (pw*qx + px*qw +py*qz - pz*qy); */
  /* 'q_mult:16'     (pw*qy - px*qz + py*qw + pz*qx); */
  /* 'q_mult:17'     (pw*qz + px*qy - py*qx + pz*qw)]; */
  scale = ((0.0 * Qminus[0] - magInertial[0] * Qminus[1]) - magInertial[1] *
           Qminus[2]) - magInertial[2] * Qminus[3];
  absxk = ((0.0 * Qminus[1] + magInertial[0] * Qminus[0]) + magInertial[1] *
           Qminus[3]) - magInertial[2] * Qminus[2];
  t = ((0.0 * Qminus[2] - magInertial[0] * Qminus[3]) + magInertial[1] * Qminus
       [0]) + magInertial[2] * Qminus[1];
  q_i_idx_3 = ((0.0 * Qminus[3] + magInertial[0] * Qminus[2]) - magInertial[1] *
               Qminus[1]) + magInertial[2] * Qminus[0];

  /*  second quaternion product Qminus_conj by q_temp */
  /* 'q_rotate:9' q_b = q_mult(q_conj(q),q_i); */
  /* q_conj() returns the quaternion conjugate */
  /*    returns the conjugate of a quaternion in  the [qw, qx, qy, qz]' */
  /*    convention */
  /* 'q_conj:6' q_conj = [q(1) -q(2) -q(3) -q(4)]'; */
  /* returns the hamilton product of two quaternions */
  /*    returns the hamiton product of two quaternions q,p in the [qw,qx,qy,qz] */
  /*    convention */
  /* 'q_mult:5' pw = p(1); */
  /* 'q_mult:6' px = p(2); */
  /* 'q_mult:7' py = p(3); */
  /* 'q_mult:8' pz = p(4); */
  /* 'q_mult:9' qw = q(1); */
  /* 'q_mult:10' qx = q(2); */
  /* 'q_mult:11' qy = q(3); */
  /* 'q_mult:12' qz = q(4); */
  /* 'q_mult:14' pq = [(pw*qw - px*qx -py*qy - pz*qz); */
  /* 'q_mult:15'     (pw*qx + px*qw +py*qz - pz*qy); */
  /* 'q_mult:16'     (pw*qy - px*qz + py*qw + pz*qx); */
  /* 'q_mult:17'     (pw*qz + px*qy - py*qx + pz*qw)]; */
  /* 'q_rotate:10' v_b = q_b(2:end); */
  y = ((Qminus[0] * absxk + -Qminus[1] * scale) + -Qminus[2] * q_i_idx_3) -
    -Qminus[3] * t;
  q_conj_idx_2 = ((Qminus[0] * t - -Qminus[1] * q_i_idx_3) + -Qminus[2] * scale)
    + -Qminus[3] * absxk;
  scale = ((Qminus[0] * q_i_idx_3 + -Qminus[1] * t) - -Qminus[2] * absxk) +
    -Qminus[3] * scale;
  dv[0] = 0.0;
  dv[3] = -mags[2];
  dv[6] = mags[1];
  dv[1] = mags[2];
  dv[4] = 0.0;
  dv[7] = -mags[0];
  dv[2] = -mags[1];
  dv[5] = mags[0];
  dv[8] = 0.0;

  /*  Error rate term  */
  /* 'ahrs_q_update:47' gyroInputWithFeedback = gyroInputWithBias + Kp_a*wmeas_a + Kp_m*wmeas_m; */
  for (i = 0; i < 3; i++) {
    absxk = (dv[i] * y + dv[i + 3] * q_conj_idx_2) + dv[i + 6] * scale;
    wmeas_m[i] = absxk;
    gyroInputWithFeedback[i] = ((gyros[i] - Bminus[i]) + Kp_a * Bplus[i]) + Kp_m
      * absxk;
  }

  /*  convert error term to pure quaternion */
  /* 'ahrs_q_update:49' p = [0; gyroInputWithFeedback(1); gyroInputWithFeedback(2); gyroInputWithFeedback(3)]; */
  /*  calculate the quaternion attitude derivative */
  /* 'ahrs_q_update:52' Qdot = 1/2 * q_mult(Qminus, p); */
  /* returns the hamilton product of two quaternions */
  /*    returns the hamiton product of two quaternions q,p in the [qw,qx,qy,qz] */
  /*    convention */
  /* 'q_mult:5' pw = p(1); */
  /* 'q_mult:6' px = p(2); */
  /* 'q_mult:7' py = p(3); */
  /* 'q_mult:8' pz = p(4); */
  /* 'q_mult:9' qw = q(1); */
  /* 'q_mult:10' qx = q(2); */
  /* 'q_mult:11' qy = q(3); */
  /* 'q_mult:12' qz = q(4); */
  /* 'q_mult:14' pq = [(pw*qw - px*qx -py*qy - pz*qz); */
  /* 'q_mult:15'     (pw*qx + px*qw +py*qz - pz*qy); */
  /* 'q_mult:16'     (pw*qy - px*qz + py*qw + pz*qx); */
  /* 'q_mult:17'     (pw*qz + px*qy - py*qx + pz*qw)]; */
  /*  integrate and normalize the new estimate of attitude */
  /* 'ahrs_q_update:54' Qplus = Qminus + Qdot * deltaT; */
  Qplus[0] = Qminus[0] + 0.5 * (((Qminus[0] * 0.0 - Qminus[1] *
    gyroInputWithFeedback[0]) - Qminus[2] * gyroInputWithFeedback[1]) - Qminus[3]
    * gyroInputWithFeedback[2]) * deltaT;
  Qplus[1] = Qminus[1] + 0.5 * (((Qminus[0] * gyroInputWithFeedback[0] + Qminus
    [1] * 0.0) + Qminus[2] * gyroInputWithFeedback[2]) - Qminus[3] *
    gyroInputWithFeedback[1]) * deltaT;
  Qplus[2] = Qminus[2] + 0.5 * (((Qminus[0] * gyroInputWithFeedback[1] - Qminus
    [1] * gyroInputWithFeedback[2]) + Qminus[2] * 0.0) + Qminus[3] *
    gyroInputWithFeedback[0]) * deltaT;
  Qplus[3] = Qminus[3] + 0.5 * (((Qminus[0] * gyroInputWithFeedback[2] + Qminus
    [1] * gyroInputWithFeedback[1]) - Qminus[2] * gyroInputWithFeedback[0]) +
    Qminus[3] * 0.0) * deltaT;

  /* 'ahrs_q_update:55' Qplus = Qplus/norm(Qplus); */
  scale = 3.3121686421112381E-170;
  absxk = fabs(Qplus[0]);
  if (absxk > 3.3121686421112381E-170) {
    q_i_idx_3 = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    q_i_idx_3 = t * t;
  }

  absxk = fabs(Qplus[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  absxk = fabs(Qplus[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  absxk = fabs(Qplus[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q_i_idx_3 = q_i_idx_3 * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    q_i_idx_3 += t * t;
  }

  q_i_idx_3 = scale * sqrt(q_i_idx_3);
  Qplus[0] /= q_i_idx_3;
  Qplus[1] /= q_i_idx_3;
  Qplus[2] /= q_i_idx_3;
  Qplus[3] /= q_i_idx_3;

  /*  calculate the bias estimate derivative */
  /* 'ahrs_q_update:58' Bdot = - Ki_a*wmeas_a - Ki_m*wmeas_m; */
  /*  integrate the bias derivative for the new bias estimate */
  /* 'ahrs_q_update:60' Bplus = Bminus + Bdot*deltaT; */
  Bplus[0] = Bminus[0] + (-Ki_a * Bplus[0] - Ki_m * wmeas_m[0]) * deltaT;
  Bplus[1] = Bminus[1] + (-Ki_a * Bplus[1] - Ki_m * wmeas_m[1]) * deltaT;
  Bplus[2] = Bminus[2] + (-Ki_a * Bplus[2] - Ki_m * wmeas_m[2]) * deltaT;
}

/* End of code generation (ahrs_q_update.c) */