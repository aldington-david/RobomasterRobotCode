//
// Created by Ken_n on 2022/9/27.
//
#include "arm_math.h"
#include "matrix.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "printf.h"

void Matrix_nodata_creat(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, InitZero _init) {
    matrix_op->arm_matrix.numRows = _i16row;
    matrix_op->arm_matrix.numCols = _i16col;
    matrix_op->is_valid = false;
    matrix_op->p2Data = (float32_t **) pvPortMalloc(
            sizeof(float32_t *) * _i16row + sizeof(float32_t) * _i16col * _i16row);
    matrix_op->arm_matrix.pData = (float32_t *) (matrix_op->p2Data + _i16row);
    if (matrix_op->p2Data != NULL) {
        for (int16_t i = 0; i < _i16row; i++) {
            matrix_op->p2Data[i] = (matrix_op->arm_matrix.pData + _i16col * i);
        }
        matrix_op->is_valid = true;
    }
    if (_init == InitMatWithZero) {
        Matrix_vSetHomogen(matrix_op, 0.0f);
    }
}

void Matrix_data_creat(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, float32_t *initData,
                       InitZero _init) {
    matrix_op->arm_matrix.numRows = _i16row;
    matrix_op->arm_matrix.numCols = _i16col;
    matrix_op->is_valid = false;
    matrix_op->p2Data = (float32_t **) pvPortMalloc(
            sizeof(float32_t *) * _i16row + sizeof(float32_t) * _i16col * _i16row);
    matrix_op->arm_matrix.pData = (float32_t *) (matrix_op->p2Data + _i16row);
    if (matrix_op->p2Data != NULL) {
        for (int16_t i = 0; i < _i16row; i++) {
            matrix_op->p2Data[i] = (matrix_op->arm_matrix.pData + _i16col * i);
        }
        matrix_op->is_valid = true;
    }
    if (_init == InitMatWithZero) {
        Matrix_vSetHomogen(matrix_op, 0.0f);
    }
    for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
        for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
            (matrix_op->p2Data)[_i][_j] = *initData;
            initData++;
        }
    }
}

bool Matrix_bMatrixIsValid(matrix_f32_t *matrix_op) {
    if ((matrix_op->arm_matrix.pData != NULL) && (matrix_op->p2Data != NULL) && (matrix_op->is_valid) &&
        (matrix_op->arm_matrix.numRows <= MATRIX_MAXIMUM_SIZE) &&
        (matrix_op->arm_matrix.numCols <= MATRIX_MAXIMUM_SIZE)) {
        return true;
    } else {
        return false;
    }

}

void Matrix_vSetMatrixInvalid(matrix_f32_t *matrix_op) {
    matrix_op->arm_matrix.numRows = -1;
    matrix_op->arm_matrix.numCols = -1;
    if (matrix_op->is_valid) {
        vPortFree(matrix_op->p2Data);
    }
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    matrix_op->p2Data = NULL;
}

bool Matrix_bMatrixIsSquare(matrix_f32_t *matrix_op) {
    return (matrix_op->arm_matrix.numRows == matrix_op->arm_matrix.numCols);
}

int16_t Matrix_i16getRow(matrix_f32_t *matrix_op) {
    return matrix_op->arm_matrix.numRows;
}

int16_t Matrix_i16getCol(matrix_f32_t *matrix_op) {
    return matrix_op->arm_matrix.numCols;
}

void Matrix_vassignment(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, const float32_t _val) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        (matrix_op->p2Data)[_i16row - 1][_i16col - 1] = _val;
    }
}

bool Matrix_bMatrixIsEqual(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R) {
    if (Matrix_bMatrixIsValid(matrix_L) && Matrix_bMatrixIsValid(matrix_R)) {
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            return false;
        }
        for (int16_t _i = 0; _i < matrix_L->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_L->arm_matrix.numCols; _j++) {
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

extern void Matrix_vadd(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_L) && Matrix_bMatrixIsValid(matrix_R)) {
        if (!Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols,
                                InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_L->arm_matrix.numCols) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
            Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_add_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}

extern void Matrix_vsub(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_L) && Matrix_bMatrixIsValid(matrix_R)) {
        if (!Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols,
                                InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_L->arm_matrix.numCols) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
            Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
            (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_sub_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}

void Matrix_vGetNegative(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        arm_mat_scale_f32(&matrix_op->arm_matrix, -1.0f, &matrix_op->arm_matrix);
    }
}

void Matrix_vmult_nsame(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_L) && Matrix_bMatrixIsValid(matrix_R)) {
        if (!Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                InitMatWithZero);
        } else if ((matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
                   (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
            Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols,
                                InitMatWithZero);
        }
        if ((matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numRows) ||
            (matrix_result->arm_matrix.numRows != matrix_L->arm_matrix.numRows) ||
            (matrix_result->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
            arm_mat_mult_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
        }
    }
}

void Matrix_vRoundingElementToZero(matrix_f32_t *matrix_op, const int16_t _i, const int16_t _j) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        if (fabsf((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
            (matrix_op->p2Data)[_i][_j] = 0.0f;
        }
    }
}

void Matrix_vRoundingMatrixToZero(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                if (fabsf((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
                    (matrix_op->p2Data)[_i][_j] = 0.0f;
                }
            }
        }
    }
}

void Matrix_vSetToZero(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        Matrix_vSetHomogen(matrix_op, 0.0f);
    }
}

void Matrix_vSetRandom(matrix_f32_t *matrix_op, const int32_t _maxRand, const int32_t _minRand) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = (float) ((rand() % (_maxRand - _minRand + 1)) + _minRand);
            }
        }
    }
}

void Matrix_vSetDiag(matrix_f32_t *matrix_op, const float _val) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                if (_i == _j) {
                    (matrix_op->p2Data)[_i][_j] = _val;
                } else {
                    (matrix_op->p2Data)[_i][_j] = 0.0f;
                }
            }
        }
    }
}

void Matrix_vSetIdentity(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        Matrix_vSetDiag(matrix_op, 1.0f);
    }
}

void Matrix_vInsertVector(matrix_f32_t *matrix_op, matrix_f32_t *_vector, const int16_t _posColumn,
                          matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(_vector)) {

        if (matrix_result != matrix_op) {
            Matrix_vCopy(matrix_op, matrix_result);
        }
        if ((_vector->arm_matrix.numRows > matrix_op->arm_matrix.numRows) ||
            (_vector->arm_matrix.numCols + _posColumn > matrix_op->arm_matrix.numCols)) {
            /* Return false */
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (int16_t _i = 0; _i < _vector->arm_matrix.numRows; _i++) {
                (matrix_result->p2Data)[_i][_posColumn] = (_vector->p2Data)[_i][0];
            }
        }
    }
}

void Matrix_vInsertAllSubMatrix(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                const int16_t _posColumn, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy(matrix_op, matrix_result);
        }
        if (((_subMatrix->arm_matrix.numRows + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_subMatrix->arm_matrix.numCols + _posColumn) > matrix_op->arm_matrix.numCols)) {
            /* Return false */
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        for (int16_t _i = 0; _i < _subMatrix->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < _subMatrix->arm_matrix.numCols; _j++) {
                (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
            }
        }
    }
}

void Matrix_vInsertPartSubMatrix_fixed(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                       const int16_t _posColumn,
                                       const int16_t _lenRow,
                                       const int16_t _lenColumn, matrix_f32_t
                                       *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy(matrix_op, matrix_result);
        }
        if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
            (_lenRow > _subMatrix->arm_matrix.numRows) || (_lenColumn > _subMatrix->arm_matrix.numCols)) {
/* Return false */
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (int16_t _i = 0; _i < _lenRow; _i++) {
                for (int16_t _j = 0; _j < _lenColumn; _j++) {
                    (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
                }
            }
        }
    }
}

void Matrix_vInsertPartSubMatrix_unstuck(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                         const int16_t _posColumn, const int16_t _posRowSub,
                                         const int16_t _posColumnSub,
                                         const int16_t _lenRow, const int16_t _lenColumn,
                                         matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(_subMatrix)) {
        if (matrix_result != matrix_op) {
            Matrix_vCopy(matrix_op, matrix_result);
        }
        if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
            ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
            ((_posRowSub + _lenRow) > _subMatrix->arm_matrix.numRows) ||
            ((_posColumnSub + _lenColumn) > _subMatrix->arm_matrix.numCols)) {
/* Return false */
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (int16_t _i = 0; _i < _lenRow; _i++) {
                for (int16_t _j = 0; _j < _lenColumn; _j++) {
                    (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[
                            _posRowSub + _i][_posColumnSub + _j];
                }
            }
        }
    }
}

void Matrix_vTranspose_nsame(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_result)) {
        Matrix_vSetMatrixInvalid(matrix_result);
        Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numCols, matrix_op->arm_matrix.numRows,
                            InitMatWithZero);
    } else {
        Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numCols, matrix_op->arm_matrix.numRows,
                            InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(matrix_result)) {
        arm_mat_trans_f32(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}

bool Matrix_bNormVector(matrix_f32_t *matrix_op) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        float _normM = 0.0f;
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
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
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] /= _normM;
            }
        }
        return true;
    } else {
        return false;
    }
}

void Matrix_vInverse_nsame(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid(matrix_result)) {
        Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                            InitMatWithZero);
    } else {
        Matrix_vSetMatrixInvalid(matrix_result);
        Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                            InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(matrix_result)) {
        arm_mat_inverse_f32(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}

bool Matrix_bMatrixIsPositiveDefinite(matrix_f32_t *matrix_op, bool checkPosSemidefinite) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        bool _posDef, _posSemiDef;
        matrix_f32_t _temp;
        Matrix_vinit(&_temp);
        Matrix_vCopy(matrix_op, &_temp);

        /* Gauss Elimination... */
        for (int16_t _j = 0; _j < (_temp.arm_matrix.numRows) - 1; _j++) {
            for (int16_t _i = _j + 1; _i < _temp.arm_matrix.numRows; _i++) {
                if (fabsf(_temp.p2Data[_j][_j]) < float_prec_ZERO) {
                    /* Q: Do we still need to check this?
                     * A: idk, It's 3 AM here.
                     *
                     * NOTE TO FUTURE SELF: Confirm it!
                     */
                    return false;
                }

                float _tempfloat = _temp.p2Data[_i][_j] / _temp.p2Data[_j][_j];

                for (int16_t _k = 0; _k < _temp.arm_matrix.numCols; _k++) {
                    _temp.p2Data[_i][_k] -= (_temp.p2Data[_j][_k] * _tempfloat);
                    Matrix_vRoundingElementToZero(&_temp, _i, _k);
                }

            }
        }

        _posDef = true;
        _posSemiDef = true;
        for (int16_t _i = 0; _i < _temp.arm_matrix.numRows; _i++) {
            if (_temp.p2Data[_i][_i] < float_prec_ZERO) {      /* false if less than 0+ (zero included) */
                _posDef = false;
            }
            if (_temp.p2Data[_i][_i] < -float_prec_ZERO) {     /* false if less than 0- (zero is not included) */
                _posSemiDef = false;
            }
        }

        Matrix_vSetMatrixInvalid(&_temp);

        if (checkPosSemidefinite) {
            return _posSemiDef;
        } else {
            return _posDef;
        }
    } else {
        return false;
    }
}

void Matrix_vGetDiagonalEntries(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        if (!Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        } else {
            Matrix_vSetMatrixInvalid(matrix_result);
            Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        }
        if (matrix_op->arm_matrix.numRows != matrix_op->arm_matrix.numCols) {
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        if (matrix_result->is_valid) {
            for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                matrix_result->p2Data[_i][0] = (matrix_op->p2Data)[_i][_i];
            }
        }
    }
}

void Matrix_vCholeskyDec(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid(matrix_result)) {
        Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                            InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(matrix_result)) {
        arm_mat_cholesky_f32(&matrix_op->arm_matrix, &matrix_result->arm_matrix);
    }
}

void
Matrix_vHouseholderTransformQR(matrix_f32_t *matrix_op, const int16_t _rowTransform, const int16_t _columnTransform,
                               matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        float32_t _tempFloat;
        float32_t _xLen;
        float32_t _x1;
        float32_t _u1;
        float32_t _vLen2;

        if (!Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_nodata_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numRows,
                                InitMatWithZero);
        }
        matrix_f32_t _vectTemp;
        Matrix_vinit(&_vectTemp);
        Matrix_nodata_creat(&_vectTemp, matrix_op->arm_matrix.numRows, 1, InitMatWithZero);
        if ((_rowTransform >= matrix_op->arm_matrix.numRows) ||
            (_columnTransform >= matrix_op->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(matrix_result);
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
        for (int16_t _i = _rowTransform + 1; _i < matrix_op->arm_matrix.numRows; _i++) {
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
            Matrix_vSetIdentity(matrix_result);
        } else {
            if (Matrix_bMatrixIsValid(matrix_result)) {
                /* P = -2*(u1*u1')/v_len2 + I */
                /* PR TODO: We can do many optimization here */
                for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                    _tempFloat = _vectTemp.p2Data[_i][0];
                    if (fabsf(_tempFloat) > float_prec_ZERO) {
                        for (int16_t _j = 0; _j < matrix_op->arm_matrix.numRows; _j++) {
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
        Matrix_vSetMatrixInvalid(&_vectTemp);
    }
}

bool Matrix_bQRDec(matrix_f32_t *matrix_op, matrix_f32_t *Qt, matrix_f32_t *R) {
    if (Matrix_bMatrixIsValid(matrix_op) && Matrix_bMatrixIsValid(Qt)) {
        matrix_f32_t Qn;
        matrix_f32_t _temp;
        Matrix_vinit(&_temp);
        Matrix_nodata_creat(&Qn, Qt->arm_matrix.numRows, Qt->arm_matrix.numCols, InitMatWithZero);
        if ((matrix_op->arm_matrix.numRows < matrix_op->arm_matrix.numCols) || (!Matrix_bMatrixIsSquare(Qt)) ||
            (Qt->arm_matrix.numRows != matrix_op->arm_matrix.numRows) ||
            (R->arm_matrix.numRows != Qt->arm_matrix.numRows) ||
            (R->arm_matrix.numCols != matrix_op->arm_matrix.numCols)) {
            Matrix_vSetMatrixInvalid(Qt);
            Matrix_vSetMatrixInvalid(R);
            return false;
        }
        R = matrix_op;
        Matrix_vSetIdentity(Qt);
        for (int16_t _i = 0;
             (_i < (matrix_op->arm_matrix.numRows - 1)) && (_i < matrix_op->arm_matrix.numCols - 1); _i++) {
            Matrix_vHouseholderTransformQR(R, _i, _i, &Qn);
            if (!Matrix_bMatrixIsValid(&Qn)) {
                Matrix_vSetMatrixInvalid(Qt);
                Matrix_vSetMatrixInvalid(R);
                return false;
            }
            Matrix_vmult_nsame(&Qn, Qt, &_temp);
            Matrix_vMove(&_temp, Qt);
            Matrix_vmult_nsame(&Qn, R, &_temp);
            Matrix_vMove(&_temp, R);
        }
        Matrix_vRoundingMatrixToZero(Qt);
        /* R.RoundingMatrixToZero(); */
        return true;
    } else {
        return false;
    }
}

void Matrix_vBackSubtitution(matrix_f32_t *upper_tri_A, matrix_f32_t *matrix_B, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid(matrix_result)) {
        Matrix_nodata_creat(matrix_result, upper_tri_A->arm_matrix.numRows, 1, InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid(upper_tri_A) && Matrix_bMatrixIsValid(matrix_B) &&
        Matrix_bMatrixIsValid(matrix_result)) {
        arm_mat_solve_upper_triangular_f32(&upper_tri_A->arm_matrix, &matrix_B->arm_matrix,
                                           &matrix_result->arm_matrix);
    }
}

void Matrix_vForwardSubtitution(matrix_f32_t *lower_tri_A, matrix_f32_t *matrix_B, matrix_f32_t *matrix_result) {
    if (!Matrix_bMatrixIsValid(matrix_result)) {
        Matrix_nodata_creat(matrix_result, lower_tri_A->arm_matrix.numRows, 1, InitMatWithZero);
    }
    if (Matrix_bMatrixIsValid(lower_tri_A) && Matrix_bMatrixIsValid(matrix_B) &&
        Matrix_bMatrixIsValid(matrix_result)) {
        arm_mat_solve_lower_triangular_f32(&lower_tri_A->arm_matrix, &matrix_B->arm_matrix,
                                           &matrix_result->arm_matrix);
    }
}

void Matrix_vRoundingadd(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                matrix_op->p2Data[_i][_j] = matrix_op->p2Data[_i][_j] + _val;
            }
        }
    }
}

void Matrix_vscale(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        arm_mat_scale_f32(&matrix_op->arm_matrix, _val, &matrix_op->arm_matrix);
    }
}

void Matrix_vRoundingsub(matrix_f32_t *matrix_op, const float32_t _val, bool minuend__val) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        if (minuend__val) {
            for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                    matrix_op->p2Data[_i][_j] = _val - matrix_op->p2Data[_i][_j];
                }
            }
        } else {
            for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
                for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                    matrix_op->p2Data[_i][_j] = matrix_op->p2Data[_i][_j] - _val;
                }
            }
        }
    }
}

void Matrix_vCopy(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        if (Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_vSetMatrixInvalid(matrix_result);
            Matrix_data_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                              matrix_op->arm_matrix.pData, NoInitMatZero);
        } else {
            Matrix_data_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                              matrix_op->arm_matrix.pData, NoInitMatZero);
        }
    }
}

void Matrix_vMove(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        if (Matrix_bMatrixIsValid(matrix_result)) {
            Matrix_vSetMatrixInvalid(matrix_result);
        }
        Matrix_data_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                          matrix_op->arm_matrix.pData, NoInitMatZero);
        Matrix_vSetMatrixInvalid(matrix_op);
    }
}

void Matrix_vSetHomogen(matrix_f32_t *matrix_op, const float32_t _val) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = _val;
            }
        }
    }
}

void Matrix_print(matrix_f32_t *matrix_op, PrintWay printway) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        char print_buf[256];
        int len = 0;
        if (printway == Matrix) {
            for (int8_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                len += sprintf((print_buf + len), "[");
                for (int8_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    len += sprintf((print_buf + len), "%8.3f", (matrix_op->p2Data)[i][j]);
                }
                len += sprintf((print_buf + len), "]");
            }
            len += sprintf((print_buf + len), ";\r\n");
            SEGGER_RTT_SetTerminal(0);
            SEGGER_RTT_WriteString(0, print_buf);
        } else if (printway == Linear) {
            for (int8_t i = 0; i < (matrix_op->arm_matrix.numRows * matrix_op->arm_matrix.numCols); i++) {
                SEGGER_RTT_printf(0, "[%d]%.3f\r\n", i, (matrix_op->arm_matrix.pData)[i]);
            }
        } else if (printway == Linear_2D) {
            for (int8_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                for (int8_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    SEGGER_RTT_printf(0, "[%d][%d]%.3f\r\n", i, j, (matrix_op->p2Data)[i][j]);
                }
            }
        }
    }
}

void Matrix_vinit(matrix_f32_t *matrix_op) {
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    matrix_op->p2Data = NULL;
}



