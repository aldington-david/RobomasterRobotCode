//
// Created by Ken_n on 2022/9/27.
//

#ifndef ROBOMASTERROBOTCODE_MATRIX_H
#define ROBOMASTERROBOTCODE_MATRIX_H

#include <stdbool.h>
#include "struct_typedef.h"
#include "matrix_functions.h"

#define float_prec          (float)
#define float_prec_ZERO     (1e-7f)
#define float_prec_ZERO_ECO (1e-5f)
//will_move_to_other_place
/* State Space dimension */
#define SS_X_LEN    (4)
#define SS_Z_LEN    (6)
#define SS_U_LEN    (3)
#define SS_DT_MILIS (20)                            /* 10 ms */
#define SS_DT       float(SS_DT_MILIS/1000.)   /* Sampling time */



/* Change this size based on the biggest matrix you will use */
#define MATRIX_MAXIMUM_SIZE     (SS_Z_LEN*2+1)
//will_move_to_other_place


typedef enum {
    InitMatWithZero,    /* Initialize matrix with zero */
    NoInitMatZero
} InitZero;

typedef enum {
    Matrix,    /* Initialize matrix with zero */
    Linear,
    Linear_2D,
} PrintWay;

typedef struct {
    arm_matrix_instance_f32 arm_matrix;
    float32_t **p2Data;
    bool is_valid;
} matrix_f32_t;

extern void Matrix_nodata_creat(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, InitZero _init);

extern void
Matrix_data_creat(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, float32_t *initData,
                  InitZero _init);

extern void Matrix_vSetHomogen(matrix_f32_t *matrix_op, const float32_t _val);

/**
  * @brief          Check whether the matrix is valid or not.
  * Index is for buffer if there's some internal rouge code with 1 index buffer overflow.
  * @param[out]     matrix_op:矩阵指针
  * @retval         bool
  */
extern bool Matrix_bMatrixIsValid(matrix_f32_t *matrix_op);

extern void Matrix_vSetMatrixInvalid(matrix_f32_t *matrix_op);

extern bool Matrix_bMatrixIsSquare(matrix_f32_t *matrix_op);

extern int16_t Matrix_i16getRow(matrix_f32_t *matrix_op);

extern int16_t Matrix_i16getCol(matrix_f32_t *matrix_op);

extern bool Matrix_bMatrixIsEqual(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R);

extern void Matrix_vadd(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result);

extern void Matrix_vsub(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result);

extern void Matrix_vGetNegative(matrix_f32_t *matrix_op);

extern void Matrix_vmult(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result);

extern void Matrix_vRoundingElementToZero(matrix_f32_t *matrix_op, const int16_t _i, const int16_t _j);

extern void Matrix_vRoundingMatrixToZero(matrix_f32_t *matrix_op);

extern void Matrix_vSetToZero(matrix_f32_t *matrix_op);

extern void Matrix_vSetRandom(matrix_f32_t *matrix_op, const int32_t _maxRand, const int32_t _minRand);

extern void Matrix_vSetDiag(matrix_f32_t *matrix_op, const float _val);

extern void Matrix_vSetIdentity(matrix_f32_t *matrix_op);

/* Insert vector into matrix at _posColumn position
 * Example: A = Matrix 3x3, B = Vector 3x1
 *
 *  C = A.InsertVector(B, 1);
 *
 *  A = [A00  A01  A02]     B = [B00]
 *      [A10  A11  A12]         [B10]
 *      [A20  A21  A22]         [B20]
 *
 *  C = [A00  B00  A02]
 *      [A10  B10  A12]
 *      [A20  B20  A22]
 */
extern void Matrix_vInsertVector(matrix_f32_t *matrix_op, matrix_f32_t *_vector, const int16_t _posColumn,
                                 matrix_f32_t *matrix_result);

/* Insert submatrix into matrix at _posRow & _posColumn position
 * Example: A = Matrix 4x4, B = Matrix 2x3
 *
 *  C = A.InsertSubMatrix(B, 1, 1);
 *
 *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
 *      [A10  A11  A12  A13]        [B10  B11  B12]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 *
 *
 *  C = [A00  A01  A02  A03]
 *      [A10  B00  B01  B02]
 *      [A20  B10  B11  B12]
 *      [A30  A31  A32  A33]
 */
extern void Matrix_vInsertAllSubMatrix(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                       const int16_t _posColumn, const int16_t _lenRow, const int16_t _lenColumn,
                                       matrix_f32_t *matrix_result);

/* Insert the _lenRow & _lenColumn submatrix, start from _posRowSub & _posColumnSub submatrix;
 *  into matrix at the matrix's _posRow and _posColumn position.
 *
 * Example: A = Matrix 4x4, B = Matrix 2x3
 *
 *  C = A.InsertSubMatrix(B, 1, 1, 0, 1, 1, 2);
 *
 *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
 *      [A10  A11  A12  A13]        [B10  B11  B12]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 *
 *
 *  C = [A00  A01  A02  A03]
 *      [A10  B01  B02  A13]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 */

extern void Matrix_vInsertPartSubMatrix(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const int16_t _posRow,
                                        const int16_t _posColumn, const int16_t _posRowSub, const int16_t _posColumnSub,
                                        const int16_t _lenRow, const int16_t _lenColumn,
                                        matrix_f32_t *matrix_result);

extern void Matrix_vCopy(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void
Matrix_vassignment(matrix_f32_t *matrix_op, const int16_t _i16row, const int16_t _i16col, const float32_t _val);
extern void Matrix_print(matrix_f32_t *matrix_op,PrintWay printway);

#endif //ROBOMASTERROBOTCODE_MATRIX_H
