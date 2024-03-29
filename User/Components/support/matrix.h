//
// Created by Ken_n on 2022/9/27.
//

#ifndef ROBOMASTERROBOTCODE_MATRIX_H
#define ROBOMASTERROBOTCODE_MATRIX_H

#include <stdbool.h>
#include "struct_typedef.h"
#include "arm_math.h"

#define float_prec          (float)
#define float_prec_ZERO     (1e-7f)
#define float_prec_ZERO_ECO (1e-5f)
//will_move_to_other_place
/* State Space dimension */
#define SS_X_LEN    (4)
#define SS_Z_LEN    (6)
#define SS_U_LEN    (3)
#define SS_DT_MILIS (5)                            /* 5 ms */
#define SS_DT       (SS_DT_MILIS/1000.0f)   /* Sampling time */



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

typedef struct {
    arm_matrix_instance_f64 arm_matrix;
    float64_t **p2Data;
    bool is_valid;
} matrix_f64_t;

extern void
Matrix_nodata_creat_f32(matrix_f32_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, InitZero _init);

extern void
Matrix_nodata_creat_f64(matrix_f64_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, InitZero _init);


extern void
Matrix_data_creat_f32(matrix_f32_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, float32_t *initData,
                      InitZero _init);

extern void
Matrix_data_creat_f64(matrix_f64_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, float64_t *initData,
                      InitZero _init);

extern void Matrix_vSetHomogen_f32(matrix_f32_t *matrix_op, const float32_t _val);

extern void Matrix_vSetHomogen_f64(matrix_f64_t *matrix_op, const float64_t _val);

/**
  * @brief          Check whether the matrix is valid or not.
  * Index is for buffer if there's some internal rouge code with 1 index buffer overflow.
  * @param[out]     matrix_op:矩阵指针
  * @retval         bool
  */
extern bool Matrix_bMatrixIsValid_f32(matrix_f32_t *matrix_op);

extern bool Matrix_bMatrixIsValid_f64(matrix_f64_t *matrix_op);

extern void Matrix_vSetMatrixInvalid_f32(matrix_f32_t *matrix_op);

extern void Matrix_vSetMatrixInvalid_f64(matrix_f64_t *matrix_op);

extern bool Matrix_bMatrixIsSquare_f32(matrix_f32_t *matrix_op);

extern bool Matrix_bMatrixIsSquare_f64(matrix_f64_t *matrix_op);

extern uint16_t Matrix_i16getRow_f32(matrix_f32_t *matrix_op);

extern uint16_t Matrix_i16getRow_f64(matrix_f64_t *matrix_op);

extern uint16_t Matrix_i16getCol_f32(matrix_f32_t *matrix_op);

extern uint16_t Matrix_i16getCol_f64(matrix_f64_t *matrix_op);

extern bool Matrix_bMatrixIsEqual_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R);

extern bool Matrix_bMatrixIsEqual_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R);

/*Will overwrite matrix_resualt*/
extern void Matrix_vadd_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result);

extern void Matrix_vadd_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R, matrix_f64_t *matrix_result);

/*Will overwrite matrix_resualt*/
extern void Matrix_vsub_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result);

extern void Matrix_vsub_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R, matrix_f64_t *matrix_result);

extern void Matrix_vGetNegative_f32(matrix_f32_t *matrix_op);

extern void Matrix_vGetNegative_f64(matrix_f64_t *matrix_op);

extern void Matrix_vmult_nsame_f32(matrix_f32_t *matrix_L, matrix_f32_t *matrix_R, matrix_f32_t *matrix_result);

extern void Matrix_vmult_nsame_f64(matrix_f64_t *matrix_L, matrix_f64_t *matrix_R, matrix_f64_t *matrix_result);

extern void Matrix_vRoundingElementToZero_f32(matrix_f32_t *matrix_op, const uint16_t _i, const uint16_t _j);

extern void Matrix_vRoundingElementToZero_f64(matrix_f64_t *matrix_op, const uint16_t _i, const uint16_t _j);

extern void Matrix_vRoundingMatrixToZero_f32(matrix_f32_t *matrix_op);

extern void Matrix_vRoundingMatrixToZero_f64(matrix_f64_t *matrix_op);

extern void Matrix_vSetToZero_f32(matrix_f32_t *matrix_op);

extern void Matrix_vSetToZero_f64(matrix_f64_t *matrix_op);

extern void Matrix_vSetRandom_f32(matrix_f32_t *matrix_op, const int32_t _maxRand, const int32_t _minRand);

extern void Matrix_vSetRandom_f64(matrix_f64_t *matrix_op, const int32_t _maxRand, const int32_t _minRand);

extern void Matrix_vSetDiag_f32(matrix_f32_t *matrix_op, const float32_t _val);

extern void Matrix_vSetDiag_f64(matrix_f64_t *matrix_op, const float64_t _val);

extern void Matrix_vSetIdentity_f32(matrix_f32_t *matrix_op);

extern void Matrix_vSetIdentity_f64(matrix_f64_t *matrix_op);

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
extern void Matrix_vInsertVector_f32(matrix_f32_t *matrix_op, matrix_f32_t *_vector, const uint16_t _posColumn,
                                     matrix_f32_t *matrix_result);

extern void Matrix_vInsertVector_f64(matrix_f64_t *matrix_op, matrix_f64_t *_vector, const uint16_t _posColumn,
                                     matrix_f64_t *matrix_result);

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
extern void Matrix_vInsertAllSubMatrix_f32(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const uint16_t _posRow,
                                           const uint16_t _posColumn, matrix_f32_t *matrix_result);

extern void Matrix_vInsertAllSubMatrix_f64(matrix_f64_t *matrix_op, matrix_f64_t *_subMatrix, const uint16_t _posRow,
                                           const uint16_t _posColumn, matrix_f64_t *matrix_result);

/* Insert the first _lenRow-th and first _lenColumn-th submatrix into matrix; at the matrix's _posRow and _posColumn position.
 * Example: A = Matrix 4x4, B = Matrix 2x3
 *
 *  C = A.InsertSubMatrix(B, 1, 1, 2, 2);
 *
 *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
 *      [A10  A11  A12  A13]        [B10  B11  B12]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 *
 *
 *  C = [A00  A01  A02  A03]
 *      [A10  B00  B01  A13]
 *      [A20  B10  B11  A23]
 *      [A30  A31  A32  A33]
 */
extern void
Matrix_vInsertPartSubMatrix_fixed_f32(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const uint16_t _posRow,
                                      const uint16_t _posColumn, const uint16_t _lenRow, const uint16_t _lenColumn,
                                      matrix_f32_t *matrix_result);

extern void
Matrix_vInsertPartSubMatrix_fixed_f64(matrix_f64_t *matrix_op, matrix_f64_t *_subMatrix, const uint16_t _posRow,
                                      const uint16_t _posColumn, const uint16_t _lenRow, const uint16_t _lenColumn,
                                      matrix_f64_t *matrix_result);

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

extern void
Matrix_vInsertPartSubMatrix_unstuck_f32(matrix_f32_t *matrix_op, matrix_f32_t *_subMatrix, const uint16_t _posRow,
                                        const uint16_t _posColumn, const uint16_t _posRowSub, const uint16_t _posColumnSub,
                                        const uint16_t _lenRow, const uint16_t _lenColumn,
                                        matrix_f32_t *matrix_result);

extern void
Matrix_vInsertPartSubMatrix_unstuck_f64(matrix_f64_t *matrix_op, matrix_f64_t *_subMatrix, const uint16_t _posRow,
                                        const uint16_t _posColumn, const uint16_t _posRowSub, const uint16_t _posColumnSub,
                                        const uint16_t _lenRow, const uint16_t _lenColumn,
                                        matrix_f64_t *matrix_result);

extern void Matrix_vTranspose_nsame_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void Matrix_vTranspose_nsame_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result);

extern bool Matrix_bNormVector_f32(matrix_f32_t *matrix_op);

extern bool Matrix_bNormVector_f64(matrix_f64_t *matrix_op);

/* Invers operation using Gauss-Jordan algorithm */
extern void Matrix_vInverse_nsame_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void Matrix_vInverse_nsame_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result);

/* Use elemtary row operation to reduce the matrix into upper triangular form (like in the first phase of gauss-jordan algorithm).
 *
 * Useful if we want to check the matrix as positive definite or not (can be used before calling CholeskyDec function).
 */
extern bool Matrix_bMatrixIsPositiveDefinite_f32(matrix_f32_t *matrix_op, bool checkPosSemidefinite);

extern bool Matrix_bMatrixIsPositiveDefinite_f64(matrix_f64_t *matrix_op, bool checkPosSemidefinite);

/* For square matrix 'this' with size MxM, return vector Mx1 with entries corresponding with diagonal entries of 'this'.
 *  Example:    this = [a11 a12 a13]
 *                     [a21 a22 a23]
 *                     [a31 a32 a33]
 *
 * out = this.GetDiagonalEntries() = [a11]
 *                                   [a22]
 *                                   [a33]
 */
extern void Matrix_vGetDiagonalEntries_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void Matrix_vGetDiagonalEntries_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result);

/* Do the Cholesky Decomposition using Cholesky-Crout algorithm.
 *
 *      A = L*L'     ; A = real, positive definite, and symmetry MxM matrix
 *
 *      L = A.CholeskyDec();
 *
 *      CATATAN! NOTE! The symmetry property is not checked at the beginning to lower
 *          the computation cost. The processing is being done on the lower triangular
 *          component of _A. Then it is assumed the upper triangular is inherently
 *          equal to the lower end.
 *          (as a side note, Scilab & MATLAB is using Lapack routines DPOTRF that process
 *           the upper triangular of _A. The result should be equal mathematically if A
 *           is symmetry).
 */
extern void Matrix_vCholeskyDec_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void Matrix_vCholeskyDec_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result);

/* Do the Householder Transformation for QR Decomposition operation.
 *              out = HouseholderTransformQR(A, i, j)
 */
extern void
Matrix_vHouseholderTransformQR_f32(matrix_f32_t *matrix_op, const uint16_t _rowTransform, const uint16_t _columnTransform,
                                   matrix_f32_t *matrix_result);

extern void
Matrix_vHouseholderTransformQR_f64(matrix_f64_t *matrix_op, const uint16_t _rowTransform, const uint16_t _columnTransform,
                                   matrix_f64_t *matrix_result);

/* Do the QR Decomposition for matrix using Householder Transformation.
 *                      A = Q*R
 *
 * PERHATIAN! CAUTION! The matrix calculated by this function return Q' and R (Q transpose and R).
 *  Because QR Decomposition usually used to calculate solution for least-squares equation (that
 *  need Q'), we don't do the transpose of Q inside this routine to lower the computation cost).
 *
 * Example of using QRDec to solve least-squares:
 *                      Ax = b
 *                   (QR)x = b
 *                      Rx = Q'b    --> Afterward use back-subtitution to solve x
 */
extern bool Matrix_bQRDec_f32(matrix_f32_t *matrix_op, matrix_f32_t *Qt, matrix_f32_t *R);

extern bool Matrix_bQRDec_f64(matrix_f64_t *matrix_op, matrix_f64_t *Qt, matrix_f64_t *R);

/* Do the back-subtitution opeartion for upper triangular matrix A & column matrix B to solve x:
 *                      Ax = B
 *
 * x = BackSubtitution(&A, &B);
 *
 * CATATAN! NOTE! To lower the computation cost, we don't check that A is a upper triangular
 *  matrix (it's assumed that user already make sure before calling this routine).
 */
extern void Matrix_vBackSubtitution_f32(matrix_f32_t *upper_tri_A, matrix_f32_t *matrix_B, matrix_f32_t *matrix_result);

extern void Matrix_vBackSubtitution_f64(matrix_f64_t *upper_tri_A, matrix_f64_t *matrix_B, matrix_f64_t *matrix_result);
/*Not yet tested, but should be working (?)*/

/* Melakukan operasi Forward-subtitution pada matrix triangular A & matrix kolom B.
 *                      Ax = B
 *
 *  Untuk menghemat komputansi, matrix A tidak dilakukan pengecekan triangular
 * (diasumsikan sudah lower-triangular).
 */
extern void
Matrix_vForwardSubtitution_f32(matrix_f32_t *lower_tri_A, matrix_f32_t *matrix_B, matrix_f32_t *matrix_result);

extern void
Matrix_vForwardSubtitution_f64(matrix_f64_t *lower_tri_A, matrix_f64_t *matrix_B, matrix_f64_t *matrix_result);

extern void Matrix_vRoundingadd_f32(matrix_f32_t *matrix_op, const float32_t _val);

extern void Matrix_vRoundingadd_f64(matrix_f64_t *matrix_op, const float64_t _val);

extern void Matrix_vRoundingsub_f32(matrix_f32_t *matrix_op, const float32_t _val, bool minuend__val);

extern void Matrix_vRoundingsub_f64(matrix_f64_t *matrix_op, const float64_t _val, bool minuend__val);

extern void Matrix_vscale_f32(matrix_f32_t *matrix_op, const float32_t _val);

extern void Matrix_vscale_f64(matrix_f64_t *matrix_op, const float64_t _val);

extern void Matrix_vCopy_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void Matrix_vCopy_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result);

extern void Matrix_vMove_f32(matrix_f32_t *matrix_op, matrix_f32_t *matrix_result);

extern void Matrix_vMove_f64(matrix_f64_t *matrix_op, matrix_f64_t *matrix_result);

extern void
Matrix_vassignment_f32(matrix_f32_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, const float32_t _val);

extern void
Matrix_vassignment_f64(matrix_f64_t *matrix_op, const uint16_t _i16row, const uint16_t _i16col, const float64_t _val);

extern void Matrix_print_f32(matrix_f32_t *matrix_op, PrintWay printway);

extern void Matrix_print_f64(matrix_f64_t *matrix_op, PrintWay printway);

extern void Matrix_vinit_f32(matrix_f32_t *matrix_op);

extern void Matrix_vinit_f64(matrix_f64_t *matrix_op);

/* self add arm matrix calculation for the future replacement*/
arm_status arm_mat_add_f64(
        const arm_matrix_instance_f64 *pSrcA,
        const arm_matrix_instance_f64 *pSrcB,
        arm_matrix_instance_f64 *pDst);

arm_status arm_mat_scale_f64(
        const arm_matrix_instance_f64 *pSrc,
        float64_t scale,
        arm_matrix_instance_f64 *pDst);
#endif //ROBOMASTERROBOTCODE_MATRIX_H
