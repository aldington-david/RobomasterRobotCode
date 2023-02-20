//
// Created by Ken_n on 2022/9/27.
//
#include "arm_math.h"
#include "matrix.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "printf.h"

void Matrix_nodata_creat_f32(matrix_f32_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, InitZero _init) {
    matrix_op->arm_matrix.numRows = _i16row;
    matrix_op->arm_matrix.numCols = _i16col;
    matrix_op->is_valid = false;
    matrix_op->p2Data = (float32_t **) pvPortMalloc(
            sizeof(float32_t *) * _i16row + sizeof(float32_t) * _i16col * _i16row);
    matrix_op->arm_matrix.pData = (float32_t *) (matrix_op->p2Data + _i16row);
    if (matrix_op->p2Data != NULL) {
        for (uint16_t i = 0; i < _i16row; i++) {
            matrix_op->p2Data[i] = (matrix_op->arm_matrix.pData + _i16col * i);
        }
        matrix_op->is_valid = true;
    }
    if (_init == InitMatWithZero) {
        Matrix_vSetHomogen_f32(matrix_op, 0.0f);
    }
}
void Matrix_nodata_creat_f64(matrix_f64_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, InitZero _init) {
    matrix_op->arm_matrix.numRows = _i16row;
    matrix_op->arm_matrix.numCols = _i16col;
    matrix_op->is_valid = false;
    matrix_op->p2Data = (float64_t **) pvPortMalloc(
            sizeof(float64_t *) * _i16row + sizeof(float64_t) * _i16col * _i16row);
    matrix_op->arm_matrix.pData = (float64_t *) (matrix_op->p2Data + _i16row);
    if (matrix_op->p2Data != NULL) {
        for (uint16_t i = 0; i < _i16row; i++) {
            matrix_op->p2Data[i] = (matrix_op->arm_matrix.pData + _i16col * i);
        }
        matrix_op->is_valid = true;
    }
    if (_init == InitMatWithZero) {
        Matrix_vSetHomogen_f64(matrix_op, 0.0);
    }
}

void Matrix_data_creat_f32(matrix_f32_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, float32_t *initData,
                           InitZero _init) {
    matrix_op->arm_matrix.numRows = _i16row;
    matrix_op->arm_matrix.numCols = _i16col;
    matrix_op->is_valid = false;
    matrix_op->p2Data = (float32_t **) pvPortMalloc(
            sizeof(float32_t *) * _i16row + sizeof(float32_t) * _i16col * _i16row);
    matrix_op->arm_matrix.pData = (float32_t *) (matrix_op->p2Data + _i16row);
    if (matrix_op->p2Data != NULL) {
        for (uint16_t i = 0; i < _i16row; i++) {
            matrix_op->p2Data[i] = (matrix_op->arm_matrix.pData + _i16col * i);
        }
        matrix_op->is_valid = true;
    }
    if (_init == InitMatWithZero) {
        Matrix_vSetHomogen_f32(matrix_op, 0.0f);
    }
    for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
        for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
            (matrix_op->p2Data)[_i][_j] = *initData;
            initData++;
        }
    }
}
void Matrix_data_creat_f64(matrix_f64_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, float64_t *initData,
                           InitZero _init) {
    matrix_op->arm_matrix.numRows = _i16row;
    matrix_op->arm_matrix.numCols = _i16col;
    matrix_op->is_valid = false;
    matrix_op->p2Data = (float64_t **) pvPortMalloc(
            sizeof(float64_t *) * _i16row + sizeof(float64_t) * _i16col * _i16row);
    matrix_op->arm_matrix.pData = (float64_t *) (matrix_op->p2Data + _i16row);
    if (matrix_op->p2Data != NULL) {
        for (uint16_t i = 0; i < _i16row; i++) {
            matrix_op->p2Data[i] = (matrix_op->arm_matrix.pData + _i16col * i);
        }
        matrix_op->is_valid = true;
    }
    if (_init == InitMatWithZero) {
        Matrix_vSetHomogen_f64(matrix_op, 0.0);
    }
    for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
        for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
            (matrix_op->p2Data)[_i][_j] = *initData;
            initData++;
        }
    }
}

bool Matrix_bMatrixIsValid_f32(matrix_f32_t *matrix_op) {
    if ((matrix_op->arm_matrix.pData != NULL) && (matrix_op->p2Data != NULL) && (matrix_op->is_valid) &&
        (matrix_op->arm_matrix.numRows <= MATRIX_MAXIMUM_SIZE) &&
        (matrix_op->arm_matrix.numCols <= MATRIX_MAXIMUM_SIZE)) {
        return true;
    } else {
        return false;
    }

}
bool Matrix_bMatrixIsValid_f64(matrix_f64_t *matrix_op) {
    if ((matrix_op->arm_matrix.pData != NULL) && (matrix_op->p2Data != NULL) && (matrix_op->is_valid) &&
        (matrix_op->arm_matrix.numRows <= MATRIX_MAXIMUM_SIZE) &&
        (matrix_op->arm_matrix.numCols <= MATRIX_MAXIMUM_SIZE)) {
        return true;
    } else {
        return false;
    }

}

void Matrix_vSetMatrixInvalid_f32(matrix_f32_t *matrix_op) {
    matrix_op->arm_matrix.numRows = -1;
    matrix_op->arm_matrix.numCols = -1;
    if (matrix_op->is_valid) {
        vPortFree(matrix_op->p2Data);
    }
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    matrix_op->p2Data = NULL;
}
void Matrix_vSetMatrixInvalid_f64(matrix_f64_t *matrix_op) {
    matrix_op->arm_matrix.numRows = -1;
    matrix_op->arm_matrix.numCols = -1;
    if (matrix_op->is_valid) {
        vPortFree(matrix_op->p2Data);
    }
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    matrix_op->p2Data = NULL;
}

bool Matrix_bMatrixIsSquare_f32(matrix_f32_t *matrix_op) {
    return (matrix_op->arm_matrix.numRows == matrix_op->arm_matrix.numCols);
}
bool Matrix_bMatrixIsSquare_f64(matrix_f64_t *matrix_op) {
    return (matrix_op->arm_matrix.numRows == matrix_op->arm_matrix.numCols);
}

uint16_t Matrix_i16getRow_f32(matrix_f32_t *matrix_op) {
    return matrix_op->arm_matrix.numRows;
}
uint16_t Matrix_i16getRow_f64(matrix_f64_t *matrix_op) {
    return matrix_op->arm_matrix.numRows;
}

uint16_t Matrix_i16getCol_f32(matrix_f32_t *matrix_op) {
    return matrix_op->arm_matrix.numCols;
}
uint16_t Matrix_i16getCol_f64(matrix_f64_t *matrix_op) {
    return matrix_op->arm_matrix.numCols;
}

void Matrix_vassignment_f32(matrix_f32_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, const float32_t _val) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        (matrix_op->p2Data)[_i16row - 1][_i16col - 1] = _val;
    }
}
void Matrix_vassignment_f64(matrix_f64_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, const float64_t _val) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        (matrix_op->p2Data)[_i16row - 1][_i16col - 1] = _val;
    }
}

bool Matrix_bMatrixIsEqual_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R) {
    if (Matrix_bMatrixIsValid_f32(matrix_L) && Matrix_bMatrixIsValid_f32(matrix_R)) {
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            return false;
        }
        for (uint16_t _i = 0; _i < matrix_L->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_L->arm_matrix.numCols; _j++) {
                if (fabsf((matrix_L->p2Data)[_i][_j] - (matrix_R->p2Data)[_i][_j]) >
                    (float_prec_ZERO)) {
                    return false;
                }
            }
        }
        return true;
    } else {
        return false;
    }
}
bool Matrix_bMatrixIsEqual_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R) {
    if (Matrix_bMatrixIsValid_f64(matrix_L) && Matrix_bMatrixIsValid_f64(matrix_R)) {
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            return false;
        }
        for (uint16_t _i = 0; _i < matrix_L->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_L->arm_matrix.numCols; _j++) {
                if (fabs((matrix_L->p2Data)[_i][_j] - (matrix_R->p2Data)[_i][_j]) >
                    (float_prec_ZERO)) {
                    return false;
                }
            }
        }
        return true;
    } else {
        return false;
    }
}

void Matrix_vadd_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_L) && Matrix_bMatrixIsValid_f32(matrix_R)) {
        if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_nodata_creat_f32(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols,
                                    InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_L->arm_matrix.numCols) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
            Matrix_nodata_creat_f32(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_add_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}
void Matrix_vadd_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_L) && Matrix_bMatrixIsValid_f64(matrix_R)) {
        if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_nodata_creat_f64(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols,
                                    InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_L->arm_matrix.numCols) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
            Matrix_nodata_creat_f64(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_add_f64(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}

extern void Matrix_vsub_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_L) && Matrix_bMatrixIsValid_f32(matrix_R)) {
        if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_nodata_creat_f32(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols,
                                    InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_L->arm_matrix.numCols) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
            Matrix_nodata_creat_f32(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_sub_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}
extern void Matrix_vsub_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_L) && Matrix_bMatrixIsValid_f64(matrix_R)) {
        if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_nodata_creat_f64(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols,
                                    InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_L->arm_matrix.numCols) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
            Matrix_nodata_creat_f64(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_sub_f64(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}

void Matrix_vGetNegative_f32(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        arm_mat_scale_f32(&matrix_op->arm_matrix, -1.0f, &matrix_op->arm_matrix);
    }
}
void Matrix_vGetNegative_f64(matrix_f64_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        arm_mat_scale_f64(&matrix_op->arm_matrix, -1.0, &matrix_op->arm_matrix);
    }
}

void Matrix_vmult_nsame_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_L) && Matrix_bMatrixIsValid_f32(matrix_R)) {
        if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_nodata_creat_f32(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
            Matrix_nodata_creat_f32(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numRows) ||
            (matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
            (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_mult_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}
void Matrix_vmult_nsame_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_L) && Matrix_bMatrixIsValid_f64(matrix_R)) {
        if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_nodata_creat_f64(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
            Matrix_nodata_creat_f64(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                    InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numRows) ||
            (matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
            (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_mult_f64(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}

void Matrix_vRoundingElementToZero_f32(matrix_f32_t *matrix_op, const uint16_t _i, const uint16_t _j) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        if (fabsf((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
            (matrix_op->p2Data)[_i][_j] = 0.0f;
        }
    }
}
void Matrix_vRoundingElementToZero_f64(matrix_f64_t *matrix_op, const uint16_t _i, const uint16_t _j) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        if (fabs((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
            (matrix_op->p2Data)[_i][_j] = 0.0f;
        }
    }
}

void Matrix_vRoundingMatrixToZero_f32(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                if (fabsf((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
                    (matrix_op->p2Data)[_i][_j] = 0.0f;
                }
            }
        }
    }
}
void Matrix_vRoundingMatrixToZero_f64(matrix_f64_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                if (fabs((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
                    (matrix_op->p2Data)[_i][_j] = 0.0f;
                }
            }
        }
    }
}

void Matrix_vSetToZero_f32(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        Matrix_vSetHomogen_f32(matrix_op, 0.0f);
    }
}
void Matrix_vSetToZero_f64(matrix_f64_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        Matrix_vSetHomogen_f64(matrix_op, 0.0);
    }
}

void Matrix_vSetRandom_f32(matrix_f32_t *matrix_op, const int32_t _maxRand, const int32_t _minRand) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = (float) ((rand() % (_maxRand - _minRand + 1)) + _minRand);
            }
        }
    }
}
void Matrix_vSetRandom_f64(matrix_f64_t *matrix_op, const int32_t _maxRand, const int32_t _minRand) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = (float) ((rand() % (_maxRand - _minRand + 1)) + _minRand);
            }
        }
    }
}

void Matrix_vSetDiag_f32(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                if (_i == _j) {
                    (matrix_op->p2Data)[_i][_j] = _val;
                } else {
                    (matrix_op->p2Data)[_i][_j] = 0.0f;
                }
            }
        }
    }
}
void Matrix_vSetDiag_f64(matrix_f64_t *matrix_op, const float64_t _val) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                if (_i == _j) {
                    (matrix_op->p2Data)[_i][_j] = _val;
                } else {
                    (matrix_op->p2Data)[_i][_j] = 0.0;
                }
            }
        }
    }
}

void Matrix_vSetIdentity_f32(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        Matrix_vSetDiag_f32(matrix_op, 1.0f);
    }
}
void Matrix_vSetIdentity_f64(matrix_f64_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        Matrix_vSetDiag_f64(matrix_op, 1.0);
    }
}

void Matrix_vInsertVector_f32(matrix_f32_t *matrix_op, matrix_f32_t *_vector, const uint16_t _posColumn,
                              matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(_vector)) {

        if (matrix_result != matrix_op) {
            Matrix_vCopy_f32(matrix_op, matrix_result);
        }
        if ((_vector->arm_matrix.numRows > matrix_op->arm_matrix.numRows) ||
            (_vector->arm_matrix.numCols + _posColumn > matrix_op->arm_matrix.numCols)) {
            /* Return false */
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < _vector->arm_matrix.numRows; _i++) {
                (matrix_result->p2Data)[_i][_posColumn] = (_vector->p2Data)[_i][0];
            }
        }
    }
}
void Matrix_vInsertVector_f64(matrix_f64_t *matrix_op, matrix_f64_t *_vector, const uint16_t _posColumn,
                              matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(_vector)) {

        if (matrix_result != matrix_op) {
            Matrix_vCopy_f64(matrix_op, matrix_result);
        }
        if ((_vector->arm_matrix.numRows > matrix_op->arm_matrix.numRows) ||
            (_vector->arm_matrix.numCols + _posColumn > matrix_op->arm_matrix.numCols)) {
            /* Return false */
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < _vector->arm_matrix.numRows; _i++) {
                (matrix_result->p2Data)[_i][_posColumn] = (_vector->p2Data)[_i][0];
            }
        }
    }
}

void Matrix_vInsertAllSubMatrix_f32(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const uint16_t _posRow,
                                    const uint16_t _posColumn, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy_f32(matrix_op, matrix_result);
        }
        if (((_subMatrix->arm_matrix.numRows + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_subMatrix->arm_matrix.numCols + _posColumn) > matrix_op->arm_matrix.numCols)) {
            /* Return false */
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        for (uint16_t _i = 0; _i < _subMatrix->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < _subMatrix->arm_matrix.numCols; _j++) {
                (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
            }
        }
    }
}
void Matrix_vInsertAllSubMatrix_f64(matrix_f64_t *matrix_op, matrix_f64_t *_subMatrix, const uint16_t _posRow,
                                    const uint16_t _posColumn, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy_f64(matrix_op, matrix_result);
        }
        if (((_subMatrix->arm_matrix.numRows + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_subMatrix->arm_matrix.numCols + _posColumn) > matrix_op->arm_matrix.numCols)) {
            /* Return false */
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        for (uint16_t _i = 0; _i < _subMatrix->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < _subMatrix->arm_matrix.numCols; _j++) {
                (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
            }
        }
    }
}

void Matrix_vInsertPartSubMatrix_fixed_f32(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const uint16_t _posRow,
                                           const uint16_t _posColumn,
                                           const uint16_t _lenRow,
                                           const uint16_t _lenColumn, matrix_f32_t
                                       *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy_f32(matrix_op, matrix_result);
        }
        if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
            (_lenRow > _subMatrix->arm_matrix.numRows) || (_lenColumn > _subMatrix->arm_matrix.numCols)) {
/* Return false */
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < _lenRow; _i++) {
                for (uint16_t _j = 0; _j < _lenColumn; _j++) {
                    (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
                }
            }
        }
    }
}
void Matrix_vInsertPartSubMatrix_fixed_f64(matrix_f64_t *matrix_op, matrix_f64_t *_subMatrix, const uint16_t _posRow,
                                           const uint16_t _posColumn,
                                           const uint16_t _lenRow,
                                           const uint16_t _lenColumn, matrix_f64_t
                                           *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy_f64(matrix_op, matrix_result);
        }
        if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
            (_lenRow > _subMatrix->arm_matrix.numRows) || (_lenColumn > _subMatrix->arm_matrix.numCols)) {
/* Return false */
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < _lenRow; _i++) {
                for (uint16_t _j = 0; _j < _lenColumn; _j++) {
                    (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
                }
            }
        }
    }
}
void Matrix_vInsertPartSubMatrix_unstuck_f32(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const uint16_t _posRow,
                                             const uint16_t _posColumn, const uint16_t _posRowSub,
                                             const uint16_t _posColumnSub,
                                             const uint16_t _lenRow, const uint16_t _lenColumn,
                                             matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy_f32(matrix_op, matrix_result);
        }
        if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
            ((_posRowSub + _lenRow) > _subMatrix->arm_matrix.numRows) ||
            ((_posColumnSub + _lenColumn) > _subMatrix->arm_matrix.numCols)) {
/* Return false */
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < _lenRow; _i++) {
                for (uint16_t _j = 0; _j < _lenColumn; _j++) {
                    (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[
                            _posRowSub + _i][_posColumnSub + _j];
                }
            }
        }
    }
}
void Matrix_vInsertPartSubMatrix_unstuck_f64(matrix_f64_t *matrix_op, matrix_f64_t *_subMatrix, const uint16_t _posRow,
                                             const uint16_t _posColumn, const uint16_t _posRowSub,
                                             const uint16_t _posColumnSub,
                                             const uint16_t _lenRow, const uint16_t _lenColumn,
                                             matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy_f64(matrix_op, matrix_result);
        }
        if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
            ((_posRowSub + _lenRow) > _subMatrix->arm_matrix.numRows) ||
            ((_posColumnSub + _lenColumn) > _subMatrix->arm_matrix.numCols)) {
/* Return false */
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < _lenRow; _i++) {
                for (uint16_t _j = 0; _j < _lenColumn; _j++) {
                    (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[
                            _posRowSub + _i][_posColumnSub + _j];
                }
            }
        }
    }
}

void Matrix_vTranspose_nsame_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_result)) {
        Matrix_vSetMatrixInvalid_f32(matrix_result);
        Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numCols, matrix_op->arm_matrix.numRows,
                                InitMatWithZero);
    } else {
        Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numCols, matrix_op->arm_matrix.numRows,
                                InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(matrix_result)) {
        arm_mat_trans_f32(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}
void Matrix_vTranspose_nsame_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_result)) {
        Matrix_vSetMatrixInvalid_f64(matrix_result);
        Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numCols, matrix_op->arm_matrix.numRows,
                                InitMatWithZero);
    } else {
        Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numCols, matrix_op->arm_matrix.numRows,
                                InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(matrix_result)) {
        arm_mat_trans_f64(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}

bool Matrix_bNormVector_f32(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        float _normM = 0.0f;
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                _normM = _normM + ((matrix_op->p2Data)[_i][_j] * (matrix_op->p2Data)[_i][_j]);
            }
        }

        if (_normM < float_prec_ZERO) {
            return false;
        }
        /* Rounding to zero to avoid case where sqrt(0-) */
        if (fabsf(_normM) < float_prec_ZERO) {
            _normM = 0.0f;
        }
        _normM = sqrtf(_normM);
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] /= _normM;
            }
        }
        return true;
    } else {
        return false;
    }
}
bool Matrix_bNormVector_f64(matrix_f64_t *matrix_op) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        float64_t _normM = 0.0;
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                _normM = _normM + ((matrix_op->p2Data)[_i][_j] * (matrix_op->p2Data)[_i][_j]);
            }
        }

        if (_normM < float_prec_ZERO) {
            return false;
        }
        /* Rounding to zero to avoid case where sqrt(0-) */
        if (fabs(_normM) < float_prec_ZERO) {
            _normM = 0.0f;
        }
        _normM = sqrt(_normM);
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] /= _normM;
            }
        }
        return true;
    } else {
        return false;
    }
}

void Matrix_vInverse_nsame_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
        Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                InitMatWithZero);
    } else {
        Matrix_vSetMatrixInvalid_f32(matrix_result);
        Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(matrix_result)) {
        arm_mat_inverse_f32(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}
void Matrix_vInverse_nsame_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
        Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                InitMatWithZero);
    } else {
        Matrix_vSetMatrixInvalid_f64(matrix_result);
        Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(matrix_result)) {
        arm_mat_inverse_f64(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}

bool Matrix_bMatrixIsPositiveDefinite_f32(matrix_f32_t *matrix_op, bool checkPosSemidefinite) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        bool _posDef, _posSemiDef;
        matrix_f32_t _temp;
        Matrix_vinit_f32(&_temp);
        Matrix_vCopy_f32(matrix_op, &_temp);

        /* Gauss Elimination... */
        for (uint16_t _j = 0; _j < (_temp.arm_matrix.numRows) - 1; _j++) {
            for (uint16_t _i = _j + 1; _i < _temp.arm_matrix.numRows; _i++) {
                if (fabsf(_temp.p2Data[_j][_j]) < float_prec_ZERO) {
                    /* Q: Do we still need to check this?
                     * A: idk, It's 3 AM here.
                     *
                     * NOTE TO FUTURE SELF: Confirm it!
                     */
                    return false;
                }

                float _tempfloat = _temp.p2Data[_i][_j] / _temp.p2Data[_j][_j];

                for (uint16_t _k = 0; _k < _temp.arm_matrix.numCols; _k++) {
                    _temp.p2Data[_i][_k] -= (_temp.p2Data[_j][_k] * _tempfloat);
                    Matrix_vRoundingElementToZero_f32(&_temp, _i, _k);
                }

            }
        }

        _posDef = true;
        _posSemiDef = true;
        for (uint16_t _i = 0; _i < _temp.arm_matrix.numRows; _i++) {
            if (_temp.p2Data[_i][_i] < float_prec_ZERO) {      /* false if less than 0+ (zero included) */
                _posDef = false;
            }
            if (_temp.p2Data[_i][_i] < -float_prec_ZERO) {     /* false if less than 0- (zero is not included) */
                _posSemiDef = false;
            }
        }

        Matrix_vSetMatrixInvalid_f32(&_temp);

        if (checkPosSemidefinite) {
            return _posSemiDef;
        } else {
            return _posDef;
        }
    } else {
        return false;
    }
}
bool Matrix_bMatrixIsPositiveDefinite_f64(matrix_f64_t *matrix_op, bool checkPosSemidefinite) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        bool _posDef, _posSemiDef;
        matrix_f64_t _temp;
        Matrix_vinit_f64(&_temp);
        Matrix_vCopy_f64(matrix_op, &_temp);

        /* Gauss Elimination... */
        for (uint16_t _j = 0; _j < (_temp.arm_matrix.numRows) - 1; _j++) {
            for (uint16_t _i = _j + 1; _i < _temp.arm_matrix.numRows; _i++) {
                if (fabs(_temp.p2Data[_j][_j]) < float_prec_ZERO) {
                    /* Q: Do we still need to check this?
                     * A: idk, It's 3 AM here.
                     *
                     * NOTE TO FUTURE SELF: Confirm it!
                     */
                    return false;
                }

                float64_t _tempfloat = _temp.p2Data[_i][_j] / _temp.p2Data[_j][_j];

                for (uint16_t _k = 0; _k < _temp.arm_matrix.numCols; _k++) {
                    _temp.p2Data[_i][_k] -= (_temp.p2Data[_j][_k] * _tempfloat);
                    Matrix_vRoundingElementToZero_f64(&_temp, _i, _k);
                }

            }
        }

        _posDef = true;
        _posSemiDef = true;
        for (uint16_t _i = 0; _i < _temp.arm_matrix.numRows; _i++) {
            if (_temp.p2Data[_i][_i] < float_prec_ZERO) {      /* false if less than 0+ (zero included) */
                _posDef = false;
            }
            if (_temp.p2Data[_i][_i] < -float_prec_ZERO) {     /* false if less than 0- (zero is not included) */
                _posSemiDef = false;
            }
        }

        Matrix_vSetMatrixInvalid_f64(&_temp);

        if (checkPosSemidefinite) {
            return _posSemiDef;
        } else {
            return _posDef;
        }
    } else {
        return false;
    }
}

void Matrix_vGetDiagonalEntries_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        } else {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
            Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        }
        if (matrix_op->arm_matrix.numRows != matrix_op->arm_matrix.numCols) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                matrix_result->p2Data[_i][0] = (matrix_op->p2Data)[_i][_i];
            }
        }
    }
}
void Matrix_vGetDiagonalEntries_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        } else {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
            Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        }
        if (matrix_op->arm_matrix.numRows != matrix_op->arm_matrix.numCols) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                matrix_result->p2Data[_i][0] = (matrix_op->p2Data)[_i][_i];
            }
        }
    }
}

void Matrix_vCholeskyDec_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
        Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(matrix_result)) {
        arm_mat_cholesky_f32(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}
void Matrix_vCholeskyDec_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
        Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(matrix_result)) {
        arm_mat_cholesky_f64(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}

void
Matrix_vHouseholderTransformQR_f32(matrix_f32_t *matrix_op, const uint16_t _rowTransform, const uint16_t _columnTransform,
                                   matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        float32_t _tempFloat;
        float32_t _xLen;
        float32_t _x1;
        float32_t _u1;
        float32_t _vLen2;

        if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_nodata_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numRows,
                                    InitMatWithZero);
        }
        matrix_f32_t _vectTemp;
        Matrix_vinit_f32(&_vectTemp);
        Matrix_nodata_creat_f32(&_vectTemp, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        if ((_rowTransform >= matrix_op->arm_matrix.numRows) ||
            (_columnTransform >= matrix_op->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }

        /* Until here:
         *
         * _xLen    = ||x||            = sqrt(x1^2 + x2^2 + .. + xn^2)
         * _vLen2   = ||u||^2 - (u1^2) = x2^2 + .. + xn^2
         * _vectTemp= [0 0 0 .. x1=0 x2 x3 .. xn]'
         */
        _x1 = (matrix_op->p2Data)[_rowTransform][_columnTransform];
        _xLen = _x1 * _x1;
        _vLen2 = 0.0f;
        for (uint16_t _i = _rowTransform + 1; _i < matrix_op->arm_matrix.numRows; _i++) {
            _vectTemp.p2Data[_i][0] = (matrix_op->p2Data)[_i][_columnTransform];

            _tempFloat = _vectTemp.p2Data[_i][0] * _vectTemp.p2Data[_i][0];
            _xLen += _tempFloat;
            _vLen2 += _tempFloat;
        }
        _xLen = sqrtf(_xLen);

        /* u1    = x1+(-sign(x1))*xLen */
        if (_x1 < 0.0) {
            _u1 = _x1 + _xLen;
        } else {
            _u1 = _x1 - _xLen;
        }


        /* Solve vlen2 & tempHH */
        _vLen2 += (_u1 * _u1);
        _vectTemp.p2Data[_rowTransform][0] = _u1;

        if (fabsf(_vLen2) < float_prec_ZERO) {
            /* x vector is collinear with basis vector e, return result = I */
            Matrix_vSetIdentity_f32(matrix_result);
        } else {
            if (Matrix_bMatrixIsValid_f32(matrix_result)) {
                /* P = -2*(u1*u1')/v_len2 + I */
                /* PR TODO: We can do many optimization here */
                for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                    _tempFloat = _vectTemp.p2Data[_i][0];
                    if (fabsf(_tempFloat) > float_prec_ZERO) {
                        for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numRows; _j++) {
                            if (fabsf(_vectTemp.p2Data[_j][0]) > float_prec_ZERO) {
                                matrix_result->p2Data[_i][_j] =
                                        _vectTemp.p2Data[_j][0] * _tempFloat * (-2.0f / _vLen2);//optimize_not_check
                            }
                        }
                    }
                    matrix_result->p2Data[_i][_i] = matrix_result->p2Data[_i][_i] + 1.0f;
                }
            }
        }
        Matrix_vSetMatrixInvalid_f32(&_vectTemp);
    }
}
void
Matrix_vHouseholderTransformQR_f64(matrix_f64_t *matrix_op, const uint16_t _rowTransform, const uint16_t _columnTransform,
                                   matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        float64_t _tempFloat;
        float64_t _xLen;
        float64_t _x1;
        float64_t _u1;
        float64_t _vLen2;

        if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_nodata_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numRows,
                                    InitMatWithZero);
        }
        matrix_f64_t _vectTemp;
        Matrix_vinit_f64(&_vectTemp);
        Matrix_nodata_creat_f64(&_vectTemp, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        if ((_rowTransform >= matrix_op->arm_matrix.numRows) ||
            (_columnTransform >= matrix_op->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }

        /* Until here:
         *
         * _xLen    = ||x||            = sqrt(x1^2 + x2^2 + .. + xn^2)
         * _vLen2   = ||u||^2 - (u1^2) = x2^2 + .. + xn^2
         * _vectTemp= [0 0 0 .. x1=0 x2 x3 .. xn]'
         */
        _x1 = (matrix_op->p2Data)[_rowTransform][_columnTransform];
        _xLen = _x1 * _x1;
        _vLen2 = 0.0f;
        for (uint16_t _i = _rowTransform + 1; _i < matrix_op->arm_matrix.numRows; _i++) {
            _vectTemp.p2Data[_i][0] = (matrix_op->p2Data)[_i][_columnTransform];

            _tempFloat = _vectTemp.p2Data[_i][0] * _vectTemp.p2Data[_i][0];
            _xLen += _tempFloat;
            _vLen2 += _tempFloat;
        }
        _xLen = sqrt(_xLen);

        /* u1    = x1+(-sign(x1))*xLen */
        if (_x1 < 0.0) {
            _u1 = _x1 + _xLen;
        } else {
            _u1 = _x1 - _xLen;
        }


        /* Solve vlen2 & tempHH */
        _vLen2 += (_u1 * _u1);
        _vectTemp.p2Data[_rowTransform][0] = _u1;

        if (fabs(_vLen2) < float_prec_ZERO) {
            /* x vector is collinear with basis vector e, return result = I */
            Matrix_vSetIdentity_f64(matrix_result);
        } else {
            if (Matrix_bMatrixIsValid_f64(matrix_result)) {
                /* P = -2*(u1*u1')/v_len2 + I */
                /* PR TODO: We can do many optimization here */
                for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                    _tempFloat = _vectTemp.p2Data[_i][0];
                    if (fabs(_tempFloat) > float_prec_ZERO) {
                        for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numRows; _j++) {
                            if (fabs(_vectTemp.p2Data[_j][0]) > float_prec_ZERO) {
                                matrix_result->p2Data[_i][_j] =
                                        _vectTemp.p2Data[_j][0] * _tempFloat * (-2.0f / _vLen2);//optimize_not_check
                            }
                        }
                    }
                    matrix_result->p2Data[_i][_i] = matrix_result->p2Data[_i][_i] + 1.0f;
                }
            }
        }
        Matrix_vSetMatrixInvalid_f64(&_vectTemp);
    }
}

bool Matrix_bQRDec_f32(matrix_f32_t *matrix_op, matrix_f32_t *Qt, matrix_f32_t *R) {
    if (Matrix_bMatrixIsValid_f32(matrix_op) && Matrix_bMatrixIsValid_f32(Qt)) {
        matrix_f32_t Qn;
        matrix_f32_t _temp;
        Matrix_vinit_f32(&_temp);
        Matrix_nodata_creat_f32(&Qn, Qt->arm_matrix.numRows, Qt->arm_matrix.numCols, InitMatWithZero);
        if ((matrix_op->arm_matrix.numRows < matrix_op->arm_matrix.numCols) || (!Matrix_bMatrixIsSquare_f32(Qt)) ||
            (Qt->arm_matrix.numRows != matrix_op->arm_matrix.numRows) ||
            (R->arm_matrix.numRows != Qt->arm_matrix.numRows) ||
            (R->arm_matrix.numCols != matrix_op->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f32(Qt);
            Matrix_vSetMatrixInvalid_f32(R);
            return false;
        }
        R = matrix_op;
        Matrix_vSetIdentity_f32(Qt);
        for (uint16_t _i = 0;
             (_i < (matrix_op->arm_matrix.numRows - 1)) && (_i < matrix_op->arm_matrix.numCols - 1); _i++) {
            Matrix_vHouseholderTransformQR_f32(R, _i, _i, &Qn);
            if (!Matrix_bMatrixIsValid_f32(&Qn)) {
                Matrix_vSetMatrixInvalid_f32(Qt);
                Matrix_vSetMatrixInvalid_f32(R);
                return false;
            }
            Matrix_vmult_nsame_f32(&Qn, Qt, &_temp);
            Matrix_vMove_f32(&_temp, Qt);
            Matrix_vmult_nsame_f32(&Qn, R, &_temp);
            Matrix_vMove_f32(&_temp, R);
        }
        Matrix_vRoundingMatrixToZero_f32(Qt);
        /* R.RoundingMatrixToZero(); */
        return true;
    } else {
        return false;
    }
}
bool Matrix_bQRDec_f64(matrix_f64_t *matrix_op, matrix_f64_t *Qt, matrix_f64_t *R) {
    if (Matrix_bMatrixIsValid_f64(matrix_op) && Matrix_bMatrixIsValid_f64(Qt)) {
        matrix_f64_t Qn;
        matrix_f64_t _temp;
        Matrix_vinit_f64(&_temp);
        Matrix_nodata_creat_f64(&Qn, Qt->arm_matrix.numRows, Qt->arm_matrix.numCols, InitMatWithZero);
        if ((matrix_op->arm_matrix.numRows < matrix_op->arm_matrix.numCols) || (!Matrix_bMatrixIsSquare_f64(Qt)) ||
            (Qt->arm_matrix.numRows != matrix_op->arm_matrix.numRows) ||
            (R->arm_matrix.numRows != Qt->arm_matrix.numRows) ||
            (R->arm_matrix.numCols != matrix_op->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid_f64(Qt);
            Matrix_vSetMatrixInvalid_f64(R);
            return false;
        }
        R = matrix_op;
        Matrix_vSetIdentity_f64(Qt);
        for (uint16_t _i = 0;
             (_i < (matrix_op->arm_matrix.numRows - 1)) && (_i < matrix_op->arm_matrix.numCols - 1); _i++) {
            Matrix_vHouseholderTransformQR_f64(R, _i, _i, &Qn);
            if (!Matrix_bMatrixIsValid_f64(&Qn)) {
                Matrix_vSetMatrixInvalid_f64(Qt);
                Matrix_vSetMatrixInvalid_f64(R);
                return false;
            }
            Matrix_vmult_nsame_f64(&Qn, Qt, &_temp);
            Matrix_vMove_f64(&_temp, Qt);
            Matrix_vmult_nsame_f64(&Qn, R, &_temp);
            Matrix_vMove_f64(&_temp, R);
        }
        Matrix_vRoundingMatrixToZero_f64(Qt);
        /* R.RoundingMatrixToZero(); */
        return true;
    } else {
        return false;
    }
}

void Matrix_vBackSubtitution_f32(matrix_f32_t *upper_tri_A, matrix_f32_t *matrix_B, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
        Matrix_nodata_creat_f32(matrix_result, upper_tri_A->arm_matrix.numRows, 1, InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f32(upper_tri_A) && Matrix_bMatrixIsValid_f32(matrix_B) &&
        Matrix_bMatrixIsValid_f32(matrix_result)) {
        arm_mat_solve_upper_triangular_f32(&upper_tri_A->arm_matrix, &matrix_B->arm_matrix,
                                           &matrix_result->arm_matrix);
    }
}
void Matrix_vBackSubtitution_f64(matrix_f64_t *upper_tri_A, matrix_f64_t *matrix_B, matrix_f64_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
        Matrix_nodata_creat_f64(matrix_result, upper_tri_A->arm_matrix.numRows, 1, InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f64(upper_tri_A) && Matrix_bMatrixIsValid_f64(matrix_B) &&
        Matrix_bMatrixIsValid_f64(matrix_result)) {
        arm_mat_solve_upper_triangular_f64(&upper_tri_A->arm_matrix, &matrix_B->arm_matrix,
                                           &matrix_result->arm_matrix);
    }
}

void Matrix_vForwardSubtitution_f32(matrix_f32_t *lower_tri_A, matrix_f32_t *matrix_B, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f32(matrix_result)) {
        Matrix_nodata_creat_f32(matrix_result, lower_tri_A->arm_matrix.numRows, 1, InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f32(lower_tri_A) && Matrix_bMatrixIsValid_f32(matrix_B) &&
        Matrix_bMatrixIsValid_f32(matrix_result)) {
        arm_mat_solve_lower_triangular_f32(&lower_tri_A->arm_matrix, &matrix_B->arm_matrix,
                                           &matrix_result->arm_matrix);
    }
}
void Matrix_vForwardSubtitution_f64(matrix_f64_t *lower_tri_A, matrix_f64_t *matrix_B, matrix_f64_t *matrix_result) {
    if (!Matrix_bMatrixIsValid_f64(matrix_result)) {
        Matrix_nodata_creat_f64(matrix_result, lower_tri_A->arm_matrix.numRows, 1, InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid_f64(lower_tri_A) && Matrix_bMatrixIsValid_f64(matrix_B) &&
        Matrix_bMatrixIsValid_f64(matrix_result)) {
        arm_mat_solve_lower_triangular_f64(&lower_tri_A->arm_matrix, &matrix_B->arm_matrix,
                                           &matrix_result->arm_matrix);
    }
}

void Matrix_vRoundingadd_f32(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                matrix_op->p2Data[_i][_j] = matrix_op->p2Data[_i][_j] + _val;
            }
        }
    }
}
void Matrix_vRoundingadd_f64(matrix_f64_t *matrix_op, const float64_t _val) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                matrix_op->p2Data[_i][_j] = matrix_op->p2Data[_i][_j] + _val;
            }
        }
    }
}

void Matrix_vscale_f32(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        arm_mat_scale_f32(&matrix_op->arm_matrix, _val, &matrix_op->arm_matrix);
    }
}
void Matrix_vscale_f64(matrix_f64_t *matrix_op, const float64_t _val) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        arm_mat_scale_f64(&matrix_op->arm_matrix, _val, &matrix_op->arm_matrix);
    }
}

void Matrix_vRoundingsub_f32(matrix_f32_t *matrix_op, const float32_t _val, bool minuend__val) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        if (minuend__val) {
            for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                    matrix_op->p2Data[_i][_j] = _val - matrix_op->p2Data[_i][_j];
                }
            }
        } else {
            for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                    matrix_op->p2Data[_i][_j] = matrix_op->p2Data[_i][_j] - _val;
                }
            }
        }
    }
}
void Matrix_vRoundingsub_f64(matrix_f64_t *matrix_op, const float64_t _val, bool minuend__val) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        if (minuend__val) {
            for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                    matrix_op->p2Data[_i][_j] = _val - matrix_op->p2Data[_i][_j];
                }
            }
        } else {
            for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                    matrix_op->p2Data[_i][_j] = matrix_op->p2Data[_i][_j] - _val;
                }
            }
        }
    }
}

void Matrix_vCopy_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        if (Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
            Matrix_data_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                  matrix_op->arm_matrix.pData, NoInitMatZero);
        } else {
            Matrix_data_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                  matrix_op->arm_matrix.pData, NoInitMatZero);
        }
    }
}
void Matrix_vCopy_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        if (Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
            Matrix_data_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                  matrix_op->arm_matrix.pData, NoInitMatZero);
        } else {
            Matrix_data_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                                  matrix_op->arm_matrix.pData, NoInitMatZero);
        }
    }
}

void Matrix_vMove_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        if (Matrix_bMatrixIsValid_f32(matrix_result)) {
            Matrix_vSetMatrixInvalid_f32(matrix_result);
        }
        Matrix_data_creat_f32(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                              matrix_op->arm_matrix.pData, NoInitMatZero);
        Matrix_vSetMatrixInvalid_f32(matrix_op);
    }
}
void Matrix_vMove_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        if (Matrix_bMatrixIsValid_f64(matrix_result)) {
            Matrix_vSetMatrixInvalid_f64(matrix_result);
        }
        Matrix_data_creat_f64(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                              matrix_op->arm_matrix.pData, NoInitMatZero);
        Matrix_vSetMatrixInvalid_f64(matrix_op);
    }
}

void Matrix_vSetHomogen_f32(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = _val;
            }
        }
    }
}
void Matrix_vSetHomogen_f64(matrix_f64_t *matrix_op, const float64_t _val) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        for (uint16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (uint16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = _val;
            }
        }
    }
}

void Matrix_print_f32(matrix_f32_t *matrix_op, PrintWay printway) {
    if (Matrix_bMatrixIsValid_f32(matrix_op)) {
        char print_buf[256];
        int len = 0;
        if (printway == Matrix) {
            for (uint16_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                len += sprintf((print_buf + len), "[");
                for (uint16_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    len += sprintf((print_buf + len), "%8.3f", (matrix_op->p2Data)[i][j]);
                }
                len += sprintf((print_buf + len), "]");
            }
            len += sprintf((print_buf + len), ";\r\n");
            SEGGER_RTT_SetTerminal(0);
            SEGGER_RTT_WriteString(0, print_buf);
        } else if (printway == Linear) {
            for (uint16_t i = 0; i < (matrix_op->arm_matrix.numRows * matrix_op->arm_matrix.numCols); i++) {
                SEGGER_RTT_printf(0, "[%d]%.3f\r\n", i, (matrix_op->arm_matrix.pData)[i]);
            }
        } else if (printway == Linear_2D) {
            for (uint16_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                for (uint16_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    SEGGER_RTT_printf(0, "[%d][%d]%.3f\r\n", i, j, (matrix_op->p2Data)[i][j]);
                }
            }
        }
    }
}
void Matrix_print_f64(matrix_f64_t *matrix_op, PrintWay printway) {
    if (Matrix_bMatrixIsValid_f64(matrix_op)) {
        char print_buf[256];
        int len = 0;
        if (printway == Matrix) {
            for (uint16_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                len += sprintf((print_buf + len), "[");
                for (uint16_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    len += sprintf((print_buf + len), "%8.3f", (matrix_op->p2Data)[i][j]);
                }
                len += sprintf((print_buf + len), "]");
            }
            len += sprintf((print_buf + len), ";\r\n");
            SEGGER_RTT_SetTerminal(0);
            SEGGER_RTT_WriteString(0, print_buf);
        } else if (printway == Linear) {
            for (uint16_t i = 0; i < (matrix_op->arm_matrix.numRows * matrix_op->arm_matrix.numCols); i++) {
                SEGGER_RTT_printf(0, "[%d]%.3f\r\n", i, (matrix_op->arm_matrix.pData)[i]);
            }
        } else if (printway == Linear_2D) {
            for (uint16_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                for (uint16_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    SEGGER_RTT_printf(0, "[%d][%d]%.3f\r\n", i, j, (matrix_op->p2Data)[i][j]);
                }
            }
        }
    }
}

void Matrix_vinit_f32(matrix_f32_t *matrix_op) {
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    matrix_op->p2Data = NULL;
}
void Matrix_vinit_f64(matrix_f64_t *matrix_op) {
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    matrix_op->p2Data = NULL;
}


/* self add arm matrix calculation for the future replacement*/
arm_status arm_mat_add_f64(
        const arm_matrix_instance_f64 * pSrcA,
        const arm_matrix_instance_f64 * pSrcB,
        arm_matrix_instance_f64 * pDst)
{
    float64_t *pInA = pSrcA->pData;                /* input data matrix pointer A */
    float64_t *pInB = pSrcB->pData;                /* input data matrix pointer B */
    float64_t *pOut = pDst->pData;                 /* output data matrix pointer */

    uint32_t numSamples;                           /* total number of elements in the matrix */
    uint32_t blkCnt;                               /* loop counters */
    arm_status status;                             /* status of matrix addition */

#ifdef ARM_MATH_MATRIX_CHECK

    /* Check for matrix mismatch condition */
    if ((pSrcA->numRows != pSrcB->numRows) ||
        (pSrcA->numCols != pSrcB->numCols) ||
        (pSrcA->numRows != pDst->numRows)  ||
        (pSrcA->numCols != pDst->numCols)    )
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else

#endif /* #ifdef ARM_MATH_MATRIX_CHECK */

    {
        /* Total number of samples in input matrix */
        numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;

#if defined (ARM_MATH_LOOPUNROLL)

        /* Loop unrolling: Compute 4 outputs at a time */
        blkCnt = numSamples >> 2U;

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) + B(m,n) */

            /* Add and store result in destination buffer. */
            *pOut++ = *pInA++ + *pInB++;

            *pOut++ = *pInA++ + *pInB++;

            *pOut++ = *pInA++ + *pInB++;

            *pOut++ = *pInA++ + *pInB++;

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Loop unrolling: Compute remaining outputs */
        blkCnt = numSamples % 0x4U;

#else

        /* Initialize blkCnt with number of samples */
    blkCnt = numSamples;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) + B(m,n) */

            /* Add and store result in destination buffer. */
            *pOut++ = *pInA++ + *pInB++;

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
    }

    /* Return to application */
    return (status);
}

arm_status arm_mat_scale_f64(
        const arm_matrix_instance_f64 * pSrc,
        float64_t                 scale,
        arm_matrix_instance_f64 * pDst)
{
    float64_t *pIn = pSrc->pData;                  /* Input data matrix pointer */
    float64_t *pOut = pDst->pData;                 /* Output data matrix pointer */
    uint32_t numSamples;                           /* Total number of elements in the matrix */
    uint32_t blkCnt;                               /* Loop counters */
    arm_status status;                             /* Status of matrix scaling */

#ifdef ARM_MATH_MATRIX_CHECK

    /* Check for matrix mismatch condition */
    if ((pSrc->numRows != pDst->numRows) ||
        (pSrc->numCols != pDst->numCols)   )
    {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
    }
    else

#endif /* #ifdef ARM_MATH_MATRIX_CHECK */

    {
        /* Total number of samples in input matrix */
        numSamples = (uint32_t) pSrc->numRows * pSrc->numCols;

#if defined (ARM_MATH_LOOPUNROLL)

        /* Loop unrolling: Compute 4 outputs at a time */
        blkCnt = numSamples >> 2U;

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) * scale */

            /* Scale and store result in destination buffer. */
            *pOut++ = (*pIn++) * scale;
            *pOut++ = (*pIn++) * scale;
            *pOut++ = (*pIn++) * scale;
            *pOut++ = (*pIn++) * scale;

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Loop unrolling: Compute remaining outputs */
        blkCnt = numSamples % 0x4U;

#else

        /* Initialize blkCnt with number of samples */
    blkCnt = numSamples;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

        while (blkCnt > 0U)
        {
            /* C(m,n) = A(m,n) * scale */

            /* Scale and store result in destination buffer. */
            *pOut++ = (*pIn++) * scale;

            /* Decrement loop counter */
            blkCnt--;
        }

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
    }

    /* Return to application */
    return (status);
}



