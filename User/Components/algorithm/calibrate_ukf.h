//
// Created by Ken_n on 2022/10/7.
//

#ifndef ROBOMASTERROBOTCODE_CALIBRATE_UKF_H
#define ROBOMASTERROBOTCODE_CALIBRATE_UKF_H

#include "matrix.h"

#define X 0
#define Y 1
#define Z 2
#define W 3

#ifndef absval
#define absval(x) ((x) < 0 ? -x : x)
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef M_PI
#define M_PI ((real_t)3.14159265358979323846)
#define M_PI_2 (M_PI * 0.5)
#define M_PI_4 (M_PI * 0.25)
#endif

#define sqrt_inv(x) (1.0f / (float)sqrt((x)))
#define divide(a, b) ((a) / (b))
#define recip(a) (1.0f / (a))
#define fsqrt(a) (float)sqrtf((a))

#define TRICAL_STATE_DIM 12
typedef struct {
    float field_norm;
    float measurement_noise;

    float state[TRICAL_STATE_DIM];
    float state_covariance[TRICAL_STATE_DIM * TRICAL_STATE_DIM];
    unsigned int measurement_count;
} TRICAL_instance_t;

/*
TRICAL_init:
Initializes `instance`. Must be called prior to any other TRICAL procedures
taking `instance` as a parameter.

If called on an instance that has already been initialized, TRICAL_init will
reset that instance to its default state.
*/
void TRICAL_init(TRICAL_instance_t *instance);

/*
TRICAL_reset:
Resets the state and state covariance of `instance`.
*/
void TRICAL_reset(TRICAL_instance_t *instance);

/*
TRICAL_norm_set:
Sets the expected field norm (magnitude) of `instance` to `norm`. If `norm`
differs from the instance's current field norm, all estimates are multiplied
by the ratio of new norm:old norm.
*/
void TRICAL_norm_set(TRICAL_instance_t *instance, float norm);

/*
TRICAL_norm_get:
Returns the expected field norm (magnitude) of `instance`.
*/
float TRICAL_norm_get(TRICAL_instance_t *instance);

/*
TRICAL_noise_set:
Sets the standard deviation in measurement supplied to `instance` to `noise`.
*/
void TRICAL_noise_set(TRICAL_instance_t *instance, float noise);

/*
TRICAL_noise_get:
Returns the standard deviation in measurements supplied to `instance`.
*/
float TRICAL_noise_get(TRICAL_instance_t *instance);

/*
TRICAL_measurement_count_get:
Returns the number of measurements previously provided to `instance` via
TRICAL_estimate_update.
*/
unsigned int TRICAL_measurement_count_get(TRICAL_instance_t *instance);

/*
TRICAL_estimate_update
Updates the calibration estimate of `instance` based on the new data in
`measurement`, and the current field direction estimate `reference_field`.
Call this function with each reading you receive from your sensor.
*/
void TRICAL_estimate_update(TRICAL_instance_t *instance,
                            float measurement[3], float reference_field[3]);

/*
TRICAL_estimate_get
Copies the calibration bias and scale esimates of `instance` to
`bias_estimate` and `scale_estimate` respectively. A new calibration estimate
will be available after every call to TRICAL_estimate_update.
*/
void TRICAL_estimate_get(TRICAL_instance_t *instance, float bias_estimate[3],
                         float scale_estimate[9]);

/*
TRICAL_estimate_get_ext
Same as TRICAL_estimate_get, but additionally copies the bias and scale
estimate variances to `bias_estimate_variance` and `scale_estimate_variance`.
*/
void TRICAL_estimate_get_ext(TRICAL_instance_t *instance,
                             float bias_estimate[3], float scale_estimate[9],
                             float bias_estimate_variance[3], float scale_estimate_variance[9]);

/*
TRICAL_measurement_calibrate
Calibrates `measurement` based on the current calibration estimates, and
copies the result to `calibrated_measurement`.

DO NOT pass the calibrated measurement into TRICAL_estimate_update, as it
needs the raw measurement values to work.
*/
void TRICAL_measurement_calibrate(TRICAL_instance_t *instance,
                                  float measurement[3], float calibrated_measurement[3]);


/* Unscented Kalman filter sigma point and scaling parameters. */
#define TRICAL_NUM_SIGMA (2 * TRICAL_STATE_DIM + 1)

#define TRICAL_ALPHA_2 (1.0f)
#define TRICAL_BETA (0.0f)
#define TRICAL_KAPPA (1.0f)
#define TRICAL_LAMBDA (TRICAL_ALPHA_2 * (TRICAL_STATE_DIM + TRICAL_KAPPA) - \
                       TRICAL_STATE_DIM)
#define TRICAL_DIM_PLUS_LAMBDA (TRICAL_ALPHA_2 * \
                                (TRICAL_STATE_DIM + TRICAL_KAPPA))

#define TRICAL_SIGMA_WM0 (TRICAL_LAMBDA / TRICAL_DIM_PLUS_LAMBDA)
#define TRICAL_SIGMA_WC0 (TRICAL_SIGMA_WM0 + \
                          (1.0f - TRICAL_ALPHA_2 + TRICAL_BETA))
#define TRICAL_SIGMA_WMI (1.0f / (2.0f * TRICAL_DIM_PLUS_LAMBDA))
#define TRICAL_SIGMA_WCI (TRICAL_SIGMA_WMI)

/* Internal function prototypes */

/*
_trical_measurement_reduce
Reduces `measurement` to a scalar value based on the calibration estimate in
`state`.
*/
float _trical_measurement_reduce(float state[TRICAL_STATE_DIM], float
measurement[3], float field[3]);

/*
_trical_measurement_calibrate
Calibrates `measurement` based on the calibration estimate in `state` and
copies the result to `calibrated_measurement`. The `measurement` and
`calibrated_measurement` parameters may be pointers to the same vector.
*/
void _trical_measurement_calibrate(float state[TRICAL_STATE_DIM],
                                   float measurement[3], float calibrated_measurement[3]);

/*
_trical_filter_iterate
Generates a new calibration estimate for `instance` incorporating the raw
sensor readings in `measurement`.
*/
void _trical_filter_iterate(TRICAL_instance_t *instance,
                            float measurement[3], float field[3]);

static void matrix_cholesky_decomp_scale_f(unsigned int dim, float L[],
                                           const float A[], const float mul);

#endif //ROBOMASTERROBOTCODE_CALIBRATE_UKF_H
