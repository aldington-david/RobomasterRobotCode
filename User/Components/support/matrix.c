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
    if ((matrix_op->is_valid) && (matrix_op->arm_matrix.numRows <= MATRIX_MAXIMUM_SIZE) &&
        (matrix_op->arm_matrix.numCols <= MATRIX_MAXIMUM_SIZE)) {
        return true;
    } else {
        return false;
    }

}

void Matrix_vSetMatrixInvalid(matrix_f32_t *matrix_op) {
    matrix_op->arm_matrix.numRows = -1;
    matrix_op->arm_matrix.numCols = -1;
    matrix_op->is_valid = false;
    matrix_op->arm_matrix.pData = NULL;
    vPortFree(matrix_op->p2Data);

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

void Matrix_assignment(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, const float32_t _val) {
    if (matrix_op->is_valid) {
        (matrix_op->p2Data)[_i16row - 1][_i16col - 1] = _val;
    }
}

bool Matrix_bMatrixIsEqual(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R) {
    if (matrix_L->is_valid && matrix_R->is_valid) {
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
    if (matrix_result->p2Data == NULL) {
        Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols, InitMatWithZero);
    }
    if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
        (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
        Matrix_vSetMatrixInvalid(matrix_result);
    }
    if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
        arm_mat_add_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
    }
}

extern void Matrix_vsub(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (matrix_result->p2Data == NULL) {
        Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_L->arm_matrix.numCols, InitMatWithZero);
    }
    if ((matrix_L->arm_matrix.numRows != matrix_R->arm_matrix.numRows) ||
        (matrix_L->arm_matrix.numCols != matrix_R->arm_matrix.numCols)) {
        Matrix_vSetMatrixInvalid(matrix_result);
    }
    if (matrix_L->is_valid && matrix_R->is_valid && matrix_result->is_valid) {
        arm_mat_sub_f32(&matrix_L->arm_matrix, &matrix_R->arm_matrix, &matrix_result->arm_matrix);
    }
}

void Matrix_vGetNegative(matrix_f32_t *matrix_op) {
    if (matrix_op->is_valid) {
        arm_mat_scale_f32(&matrix_op->arm_matrix, -1.0f, &matrix_op->arm_matrix);
    }
}

void Matrix_vmult(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result) {
    if (matrix_result->p2Data == NULL) {
        Matrix_nodata_creat(matrix_result, matrix_L->arm_matrix.numRows, matrix_R->arm_matrix.numCols, InitMatWithZero);
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

void Matrix_vRoundingElementToZero(matrix_f32_t *matrix_op, const int16_t _i, const int16_t _j) {
    if (fabsf((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
        (matrix_op->p2Data)[_i][_j] = 0.0f;
    }
}

void Matrix_vRoundingMatrixToZero(matrix_f32_t *matrix_op) {
    for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
        for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
            if (fabsf((matrix_op->p2Data)[_i][_j]) < float_prec_ZERO) {
                (matrix_op->p2Data)[_i][_j] = 0.0f;
            }
        }
    }
}

void Matrix_vSetToZero(matrix_f32_t *matrix_op) {
    Matrix_vSetHomogen(matrix_op, 0.0f);
}

void Matrix_vSetRandom(matrix_f32_t *matrix_op, const int32_t _maxRand, const int32_t _minRand) {
    for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
        for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
            (matrix_op->p2Data)[_i][_j] = (float) ((rand() % (_maxRand - _minRand + 1)) + _minRand);
        }
    }
}

void Matrix_vSetDiag(matrix_f32_t *matrix_op, const float _val) {
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

void Matrix_vSetIdentity(matrix_f32_t *matrix_op) {
    Matrix_vSetDiag(matrix_op, 1.0f);
}

void Matrix_vInsertVector(matrix_f32_t *matrix_op, matrix_f32_t *_vector, const int16_t _posColumn,
                          matrix_f32_t *matrix_result) {
    if (matrix_result != matrix_op) {
        Matrix_vCopy(matrix_op, matrix_result);
    }
    if ((_vector->arm_matrix.numRows > matrix_op->arm_matrix.numRows) ||
        (_vector->arm_matrix.numCols + _posColumn > matrix_op->arm_matrix.numCols)) {
        /* Return false */
        Matrix_vSetMatrixInvalid(matrix_result);
    }
    for (int16_t _i = 0; _i < _vector->arm_matrix.numRows; _i++) {
        (matrix_result->p2Data)[_i][_posColumn] = (_vector->p2Data)[_i][0];
    }
}

void Matrix_vInsertAllSubMatrix(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                const int16_t _posColumn,
                                const int16_t _lenRow,
                                const int16_t _lenColumn, matrix_f32_t
                                *matrix_result) {
    if (matrix_result != matrix_op) {
        Matrix_vCopy(matrix_op, matrix_result);
    }
    if (((_lenRow + _posRow) > matrix_op->arm_matrix.numRows) ||
        ((_lenColumn + _posColumn) > matrix_op->arm_matrix.numCols) ||
        (_lenRow > _subMatrix->arm_matrix.numRows) || (_lenColumn > _subMatrix->arm_matrix.numCols)) {
/* Return false */
        Matrix_vSetMatrixInvalid(matrix_result);
    }
    for (int16_t _i = 0; _i < _lenRow; _i++) {
        for (int16_t _j = 0; _j < _lenColumn; _j++) {
            (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[_i][_j];
        }
    }
}

void Matrix_vInsertPartSubMatrix(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                 const int16_t _posColumn, const int16_t _posRowSub, const int16_t _posColumnSub,
                                 const int16_t _lenRow, const int16_t _lenColumn,
                                 matrix_f32_t *matrix_result) {
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
    for (int16_t _i = 0; _i < _lenRow; _i++) {
        for (int16_t _j = 0; _j < _lenColumn; _j++) {
            (matrix_result->p2Data)[_i + _posRow][_j + _posColumn] = (_subMatrix->p2Data)[
                    _posRowSub + _i][_posColumnSub + _j];
        }
    }
}

void Matrix_vCopy(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result) {
    Matrix_data_creat(matrix_result, matrix_op->arm_matrix.numRows, matrix_op->arm_matrix.numCols,
                      matrix_op->arm_matrix.pData, NoInitMatZero);
}

void Matrix_vSetHomogen(matrix_f32_t *matrix_op, const float32_t _val) {
    if (matrix_op->is_valid) {
        for (int16_t _i = 0; _i < matrix_op->arm_matrix.numRows; _i++) {
            for (int16_t _j = 0; _j < matrix_op->arm_matrix.numCols; _j++) {
                (matrix_op->p2Data)[_i][_j] = _val;
            }
        }
    }
}

void Matrix_print(matrix_f32_t *matrix_op, PrintWay printway) {
    if (Matrix_bMatrixIsValid(matrix_op)) {
        char print_buf[64];
        int len = 0;
        if (printway == Matrix) {
            for (int8_t i = 0; i < matrix_op->arm_matrix.numRows; i++) {
                SEGGER_RTT_WriteString(0, "[");
                for (int8_t j = 0; j < matrix_op->arm_matrix.numCols; j++) {
                    len += sprintf((print_buf + len), "%3.3f ", (matrix_op->p2Data)[i][j]);
                    SEGGER_RTT_SetTerminal(0);
                    SEGGER_RTT_WriteString(0, print_buf);
                }
                SEGGER_RTT_WriteString(0, "]\r\n");
            }
        } else if (printway == Linear) {
            for (int8_t i = 0; i < (matrix_op->arm_matrix.numRows * matrix_op->arm_matrix.numCols); i++) {
                SEGGER_RTT_printf(0, "[%d]%.3f\r\n", i, (matrix_op->p2Data)[i]);
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



