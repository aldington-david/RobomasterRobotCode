//
// Created by Ken_n on 2022/10/7.
//

#include "calibrate_ukf.h"
#include "matrix.h"
#include "ahrs_ukf.h"

/*
TRICAL_init:
Initializes `instance`. Must be called prior to any other TRICAL procedures
taking `instance` as a parameter.

If called on an instance that has already been initialized, TRICAL_init will
reset that instance to its default state.
*/
void TRICAL_init(TRICAL_instance_t *instance) {

    memset(instance, 0, sizeof(TRICAL_instance_t));

    instance->field_norm = 1.0f;
    instance->measurement_noise = 1e-6f;

    /*
    Set the state covariance diagonal to a small value, so that we can run the
    Cholesky decomposition without blowing up
    */
    unsigned int i;
    for (i = 0; i < TRICAL_STATE_DIM * TRICAL_STATE_DIM;
         i += (TRICAL_STATE_DIM + 1)) {
        instance->state_covariance[i] = 1e-2f;
    }
}

/*
TRICAL_reset:
Resets the state and state covariance of `instance`.
*/
void TRICAL_reset(TRICAL_instance_t *instance) {

    memset(instance->state, 0, sizeof(instance->state));
    memset(instance->state_covariance, 0, sizeof(instance->state_covariance));

    /*
    Set the state covariance diagonal to a small value, so that we can run the
    Cholesky decomposition without blowing up
    */
    unsigned int i;
    for (i = 0; i < TRICAL_STATE_DIM * TRICAL_STATE_DIM;
         i += (TRICAL_STATE_DIM + 1)) {
        instance->state_covariance[i] = 1e-2f;
    }
}

/*
TRICAL_norm_set:
Sets the expected field norm (magnitude) of `instance` to `norm`.
*/
void TRICAL_norm_set(TRICAL_instance_t *instance, float norm) {

    instance->field_norm = norm;
}

/*
TRICAL_norm_get:
Returns the expected field norm (magnitude) of `instance`.
*/
float TRICAL_norm_get(TRICAL_instance_t *instance) {

    return instance->field_norm;
}

/*
TRICAL_noise_set:
Sets the standard deviation in measurement supplied to `instance` to `noise`.
*/
void TRICAL_noise_set(TRICAL_instance_t *instance, float noise) {

    instance->measurement_noise = noise;
}

/*
TRICAL_noise_get:
Returns the standard deviation in measurements supplied to `instance`.
*/
float TRICAL_noise_get(TRICAL_instance_t *instance) {

    return instance->measurement_noise;
}

/*
TRICAL_measurement_count_get:
Returns the number of measurements previously provided to `instance` via
TRICAL_estimate_update.
*/
unsigned int TRICAL_measurement_count_get(TRICAL_instance_t *instance) {

    return instance->measurement_count;
}

/*
TRICAL_estimate_update
Updates the calibration estimate of `instance` based on the new data in
`measurement`, and the current field direction estimate `reference_field`.
Call this function with each reading you receive from your sensor.
*/
void TRICAL_estimate_update(TRICAL_instance_t *instance,
                            float measurement[3], float reference_field[3]) {

    _trical_filter_iterate(instance, measurement, reference_field);
    instance->measurement_count++;
}

/*
TRICAL_estimate_get
Copies the calibration bias and scale esimates of `instance` to
`bias_estimate` and `scale_estimate` respectively. A new calibration estimate
will be available after every call to TRICAL_estimate_update.
*/
void TRICAL_estimate_get(TRICAL_instance_t *restrict instance,
                         float bias_estimate[3], float scale_estimate[9]) {

    /* Copy bias estimate from state[0:3] to the destination vector */
    memcpy(bias_estimate, instance->state, 3 * sizeof(float));

    /* Copy the scale estimate to the destination matrix */
    memcpy(scale_estimate, &instance->state[3], 9 * sizeof(float));
}

/*
TRICAL_estimate_get_ext
Same as TRICAL_estimate_get, but additionally copies the bias and scale
estimate variances to `bias_estimate_variance` and `scale_estimate_variance`.
*/
void TRICAL_estimate_get_ext(TRICAL_instance_t *restrict instance,
                             float bias_estimate[3], float scale_estimate[9],
                             float bias_estimate_variance[3], float scale_estimate_variance[9]) {
    TRICAL_estimate_get(instance, bias_estimate, scale_estimate);

    /* A bit of paranoia to avoid potential undefined behaviour */

    /* Copy bias estimate covariance from the state covariance diagonal */
    bias_estimate_variance[0] = instance->state_covariance[0 * 12 + 0];
    bias_estimate_variance[1] = instance->state_covariance[1 * 12 + 1];
    bias_estimate_variance[2] = instance->state_covariance[2 * 12 + 2];

    /* Now copy scale estimate covariance. */
    scale_estimate_variance[0] = instance->state_covariance[3 * 12 + 3];
    scale_estimate_variance[1] = instance->state_covariance[4 * 12 + 4];
    scale_estimate_variance[2] = instance->state_covariance[5 * 12 + 5];

    scale_estimate_variance[3] = instance->state_covariance[6 * 12 + 6];
    scale_estimate_variance[4] = instance->state_covariance[7 * 12 + 7];
    scale_estimate_variance[5] = instance->state_covariance[8 * 12 + 8];

    scale_estimate_variance[6] = instance->state_covariance[9 * 12 + 9];
    scale_estimate_variance[7] = instance->state_covariance[10 * 12 + 10];
    scale_estimate_variance[8] = instance->state_covariance[11 * 12 + 11];
}

/*
TRICAL_measurement_calibrate
Calibrates `measurement` based on the current calibration estimates, and
copies the result to `calibrated_measurement`. The `measurement` and
`calibrated_measurement` parameters may be pointers to the same vector.

DO NOT pass the calibrated measurement into TRICAL_estimate_update, as it
needs the raw measurement values to work.
*/
void TRICAL_measurement_calibrate(TRICAL_instance_t *restrict instance,
                                  float measurement[3], float calibrated_measurement[3]) {

    /* Pass off to the internal function */
    _trical_measurement_calibrate(instance->state, measurement,
                                  calibrated_measurement);
}

/*
A bit about the UKF formulation in this file: main references are
[1]: http://www.acsu.buffalo.edu/~johnc/mag_cal05.pdf
[2]: http://malcolmdshuster.com/Pub_2002c_J_scale_scan.pdf

Since we don't know the attitude matrix, we use scalar values for measurements
and scalar measurement noise.

The relevant equations are (2a) and (2b) in [2]. Basically, the measurement
value is:

z = -Bt x (2D + D^2) x B + 2Bt x (I + D) x b - |b|^2

where B is the 3-axis measurement vector, and H is the reference field (making
|H| the norm of the reference field, which we know). Here, t is used as the
transpose operator.

The measurement noise is a scalar:

v = 2[(I + D) x B - b]t x E - |E|^2

where B is the 3-axis measurement vector, b is the 3-axis bias estimate,
E is the measurement noise, I is the 3x3 identity matrix, and D is the 3x3
scale estimate.

The implementation follows the general approach of https://github.com/sfwa/ukf
however there a number of simplifications due to the restricted problem
domain.

Since there is no process model, the apriori mean is the same as the state at
the start of the update step, and W' is just the set of sigma points. That in
turn means that there's no need to calculate propagated state covariance from
W'; it's the same as the covariance was at the start of the update step.

The scalar measurement model also simplifies a lot of the cross-correlation
and Kalman gain calculation.

Like the full SFWA UKF, and unlike the papers upon which this approach is
based, we're actually using the scaled unscented Kalman filter. The main
differences are in the choices of scaling parameters, including alpha, beta
and kappa. Given the relatively small dimensionality of the filter it's
probably not strictly necessary to use the scaled formulation, but we'd
already implemented that so it seemed easier to continue with that approach.
*/

/*
_trical_measurement_reduce
Reduces `measurement` to a scalar value based on the calibration estimate in
`state` and the current field direction estimate `field` (if no absolute
orientation calibration is required, the same vector can be supplied for
`measurement` and `field`).

FIXME: It's not clear to me why the measurement model should be as in (2a)
above, when we can just apply the current calibration estimate to the
measurement and take its magnitude. Look into this further if the below
approach doesn't work.
*/
float _trical_measurement_reduce(float state[TRICAL_STATE_DIM], float
measurement[3], float field[3]) {
    float temp[3];
    _trical_measurement_calibrate(state, measurement, temp);

    return fsqrt(fabsf(temp[0] * field[0] + temp[1] * field[1] +
                       temp[2] * field[2]));
}

/*
_trical_measurement_calibrate
Calibrates `measurement` based on the calibration estimate in `state` and
copies the result to `calibrated_measurement`. The `measurement` and
`calibrated_measurement` parameters may be pointers to the same vector.

Implements
B' = (I_{3x3} + D)B - b

where B' is the calibrated measurement, I_{3x3} is the 3x3 identity matrix,
D is the scale calibration matrix, B is the raw measurement, and b is the bias
vector.
*/
void _trical_measurement_calibrate(float state[TRICAL_STATE_DIM],
                                   float measurement[3], float calibrated_measurement[3]) {

    float v[3], *restrict s = state, *restrict c = calibrated_measurement;
    v[0] = measurement[0] - s[0];
    v[1] = measurement[1] - s[1];
    v[2] = measurement[2] - s[2];

    /* 3x3 matrix multiply */
    c[0] = v[0] * (s[3] + 1.0f) + v[1] * s[4] + v[2] * s[5];
    c[1] = v[0] * s[6] + v[1] * (s[7] + 1.0f) + v[2] * s[8];
    c[2] = v[0] * s[9] + v[1] * s[10] + v[2] * (s[11] + 1.0f);
}

/*
_trical_filter_iterate
Generates a new calibration estimate for `instance` incorporating the raw
sensor readings in `measurement`.
*/
void _trical_filter_iterate(TRICAL_instance_t *instance,
                            float measurement[3], float field[3]) {
    unsigned int i, j, k, l, col;

    float *restrict covariance = instance->state_covariance;
    float *restrict state = instance->state;

    /*
    LLT decomposition on state covariance matrix, with result multiplied by
    TRICAL_DIM_PLUS_LAMBDA
    */
    float covariance_llt[TRICAL_STATE_DIM * TRICAL_STATE_DIM];
    memset(covariance_llt, 0, sizeof(covariance_llt));
    matrix_cholesky_decomp_scale_f(
            TRICAL_STATE_DIM, covariance_llt, covariance, TRICAL_DIM_PLUS_LAMBDA);



    /*
    Generate the sigma points, and use them as the basis of the measurement
    estimates
    */
    float temp_sigma[TRICAL_STATE_DIM], temp;
    float measurement_estimates[TRICAL_NUM_SIGMA], measurement_estimate_mean;

    measurement_estimate_mean = 0.0f;

    /*
    Handle central sigma point -- process the measurement based on the current
    state vector
    */
    measurement_estimates[0] = _trical_measurement_reduce(state, measurement,
                                                          field);

//#pragma MUST_ITERATE(12, 12);
    for (i = 0, col = 0; i < TRICAL_STATE_DIM; i++, col += TRICAL_STATE_DIM) {
        /*
        Handle the positive sigma point -- perturb the state vector based on
        the current column of the covariance matrix, and process the
        measurement based on the resulting state estimate
        */
//#pragma MUST_ITERATE(12, 12);
        for (k = col, l = 0; l < TRICAL_STATE_DIM; k++, l++) {
            temp_sigma[l] = state[l] + covariance_llt[k];
        }
        measurement_estimates[i + 1] =
                _trical_measurement_reduce(temp_sigma, measurement, field);

        /* Handle the negative sigma point -- mirror of the above */
//#pragma MUST_ITERATE(12, 12);
        for (k = col, l = 0; l < TRICAL_STATE_DIM; k++, l++) {
            temp_sigma[l] = state[l] - covariance_llt[k];
        }
        measurement_estimates[i + 1 + TRICAL_STATE_DIM] =
                _trical_measurement_reduce(temp_sigma, measurement, field);

        /* Calculate the measurement estimate sum as we go */
        temp = measurement_estimates[i + 1] +
               measurement_estimates[i + 1 + TRICAL_STATE_DIM];
        measurement_estimate_mean += temp;
    }

    measurement_estimate_mean = measurement_estimate_mean * TRICAL_SIGMA_WMI +
                                measurement_estimates[0] * TRICAL_SIGMA_WM0;

    /*
    Convert estimates to deviation from mean (so measurement_estimates
    effectively becomes Z').

    While we're at it, calculate the measurement estimate covariance (which
    is a scalar quantity).
    */
    float measurement_estimate_covariance = 0.0f;

//#pragma MUST_ITERATE(25, 25);
    for (i = 0; i < TRICAL_NUM_SIGMA; i++) {
        measurement_estimates[i] -= measurement_estimate_mean;

        temp = measurement_estimates[i] * measurement_estimates[i];
        measurement_estimate_covariance += temp;
    }



    /* Add the sensor noise to the measurement estimate covariance */
    temp = instance->measurement_noise * instance->measurement_noise;
    measurement_estimate_covariance += temp;

    /* Calculate cross-correlation matrix (1 x TRICAL_STATE_DIM) */
    float cross_correlation[TRICAL_STATE_DIM];
    memset(cross_correlation, 0, sizeof(cross_correlation));

    /*
    Calculate the innovation (difference between the expected value, i.e. the
    field norm, and the measurement estimate mean).
    */
    float innovation;
    innovation = instance->field_norm - measurement_estimate_mean;

    /* Iterate over sigma points, two at a time */
//#pragma MUST_ITERATE(12, 12);
    for (i = 0; i < TRICAL_STATE_DIM; i++) {
        /* Iterate over the cross-correlation matrix */
//#pragma MUST_ITERATE(12, 12);
        for (j = 0; j < TRICAL_STATE_DIM; j++) {
            /*
            We're regenerating the sigma points as we go, so that we don't
            need to store W'.
            */
            temp = measurement_estimates[i + 1] *
                   (state[j] + covariance_llt[i * TRICAL_STATE_DIM + j]);
            cross_correlation[j] += temp;

            temp = measurement_estimates[i + 1 + TRICAL_STATE_DIM] *
                   (state[j] - covariance_llt[i * TRICAL_STATE_DIM + j]);
            cross_correlation[j] += temp;
        }
    }

    /*
    Scale the results of the previous step, and add in the scaled central
    sigma point
    */
//#pragma MUST_ITERATE(12, 12);
    for (j = 0; j < TRICAL_STATE_DIM; j++) {
        temp = TRICAL_SIGMA_WC0 * measurement_estimates[0] * state[j];
        cross_correlation[j] = TRICAL_SIGMA_WCI * cross_correlation[j] + temp;
    }


    /*
    Update the state -- since the measurement is a scalar, we can calculate
    the Kalman gain and update in a single pass.
    */
    float kalman_gain;
    temp = recip(measurement_estimate_covariance);
//#pragma MUST_ITERATE(12, 12);
    for (i = 0; i < TRICAL_STATE_DIM; i++) {
        kalman_gain = cross_correlation[i] * temp;
        state[i] += kalman_gain * innovation;
    }



    /*
    Update the state covariance:
    covariance = covariance - kalman gain * measurement estimate covariance *
                 (transpose of kalman gain)

    Since kalman gain is a 1 x TRICAL_STATE_DIM matrix, multiplying by its
    transpose is just a vector outer product accumulating onto state
    covariance.

    And, of course, since kalman gain is cross correlation *
    (1 / measurement estimate covariance), and we multiply by measurement
    estimate covariance during the outer product, we can skip that whole step
    and just use cross correlation instead.
    */
//#pragma MUST_ITERATE(12, 12)
    for (i = 0; i < TRICAL_STATE_DIM; i++) {
        temp = -cross_correlation[i];

//#pragma MUST_ITERATE(12, 12)
        for (j = 0; j < TRICAL_STATE_DIM; j++) {
            covariance[i * TRICAL_STATE_DIM + j] +=
                    temp * cross_correlation[j];
        }
    }


}

static void matrix_cholesky_decomp_scale_f(unsigned int dim, float L[],
                                           const float A[], const float mul) {

    /*
    9x9:
    900 mult
    72 div
    9 sqrt
    */

    unsigned int i, j, kn, in, jn;
    for (i = 0, in = 0; i < dim; i++, in += dim) {
        L[i + 0] = (i == 0) ? fsqrt(A[i + in]*mul) : recip(L[0]) * (A[i]*mul);

        for (j = 1, jn = dim; j <= i; j++, jn += dim) {
            float s = 0;
//#pragma MUST_ITERATE(1,9)
            for (kn = 0; kn < j*dim; kn += dim) {
                s += L[i + kn] * L[j + kn];
            }

            L[i + jn] = (i == j) ? fsqrt(A[i + in]*mul - s) :
                        recip(L[j + jn]) * (A[i + jn]*mul - s);
        }
    }
}

void realtime_mag_cali(matrix_f32_t quaternion_attitude) {
    static matrix_f32_t last_quaternion_attitude;
    static float32_t body_wmm_filed[3];
    Matrix_vinit(&last_quaternion_attitude);
    Matrix_vCopy(&last_quaternion_attitude, &last_quaternion_attitude);
    /*
If the current attitude is too close to the attitude at which this
TRICAL instance was last updated, skip calibration this time
*/
    float delta_angle = quaternion_quaternion_angle_f(quaternion_attitude.arm_matrix.pData,
                                                      last_quaternion_attitude.arm_matrix.pData);
    if (delta_angle < 3.0 * PI / 180.0) {
        return;
    }
    quaternion_vector3_multiply_f(body_wmm_filed,quaternion_attitude.arm_matrix.pData,IMU_MAG_B0_data);

}

float quaternion_quaternion_angle_f(const float q1[4], const float q2[4]) {
    float qdot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    return acosf(2.0f * qdot - 1.0f);
}

float quaternion_vector3_multiply_f(float result[3], const float q[4], const float v[3]) {
    /*
    Multiply a quaternion by a vector (i.e. transform a vectory by a
    quaternion)

    v' = q * v * conjugate(q), or:
    t = 2 * cross(q.xyz, v)
    v' = v + q.w * t + cross(q.xyz, t)

    http://molecularmusings.wordpress.com/2013/05/24/a-faster-quaternion-vector-multiplication/
    */

    float rx, ry, rz, tx, ty, tz;

    tx = q[2] * v[2];
    ty = q[3] * v[0];
    tx -= q[3] * v[1];
    ty -= q[1] * v[2];
    tz = q[1] * v[1];
    ty *= 2.0f;
    tz -= q[2] * v[0];
    tx *= 2.0f;
    tz *= 2.0f;

    rx = v[0];
    rx += q[0] * tx;
    rx += q[2] * tz;
    rx -= q[3] * ty;
    result[1] = rx;

    ry = v[1];
    ry += q[0] * ty;
    ry += q[3] * tx;
    ry -= q[1] * tz;
    result[2] = ry;

    rz = v[2];
    rz += q[0] * tz;
    rz -= q[2] * tx;
    rz += q[1] * ty;
    result[3] = rz;
}