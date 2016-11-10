#ifndef _MYMATH_H
#define _MYMATH_H

#include <math.h>
#include <stdlib.h>

#define my_math_eps  				(1.0f / 4294967296.0f)
#define my_math_pi  				(3.1415926535897932f)
#define my_math_2pi  				(2.0f*my_math_pi)
#define my_math_deg_to_rad  (my_math_pi/180.0f)
#define my_math_sqrt2  			(1.4142135623f)
#define my_math_sqrt2_reci  (1.0f/my_math_sqrt2)
#define my_math_sqrt3				(1.732050807568877f)
#define my_math_sqrt3_reci	(1.0f/my_math_sqrt3)


struct _s_polar
{
	float dis;
	float theta;
	float cos_theta;
	float sin_theta;
};

/*
 * matrix is represented as a one dimention C code array
 * matrix's "column" is array's "row"
 * all calculate is based on matrix's column
 */
struct _s_matrix
{
	unsigned int rowNum;
	unsigned int columnNum;
	float *data;
};

#define MATRIX_DEF(name,row,column) float name##_##data[row*column];\
																    struct _s_matrix name = {row,column,0}
#define STATIC_MATRIX_DEF(name,row,column) static float name##_##data[row*column];\
																    static struct _s_matrix name = {row,column,0}
#define MATRIX_INI(name) {name.data = name##_##data;}
#define EXTERN_MATRIX(name) extern struct _s_matrix name;\
														extern float name##_##data[]

float MinorArc(float a, float b);
float LimitAngleTo_npi_ppi(float a);
float LimitAngleTo_0_2pi(float a);
void RotationMatrix_2D(struct _s_matrix *R, float theta);
void Rotate_2D(float *x, float *y, float theta);

struct _s_matrix *MatrixIni(unsigned int rowNum, 
														unsigned int columnNum);
void VectSub(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB);
void VectAdd(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB);
void VectScaleAdd(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB,
		float scaleA, float scaleB);
void VectCopy(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA);
float VectLength(
		struct _s_matrix *matA, unsigned int columnIndex);
float VectDistance(
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB);
int VectNormalize(
		struct _s_matrix *matA, unsigned int columnIndex);
float VectDot(
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB);
void VectMult(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB);
void VectScale(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA,
		float scale);
void MatCopy(
		struct _s_matrix *matR,
		struct _s_matrix *matA);
void MatBlockCopy(
		struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
		struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
		unsigned int rowSizeBlock, unsigned int columnSizeBlock);
void MatScale(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		float scale);
void MatScaleAdd(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		float scale);
void MatSetValue(
		struct _s_matrix *matR, float value);
void MatBlockSetValue(
		struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
		int rowSizeBlock, int columnSizeBlock,
		float value);
void MatSetDiagonal(
		struct _s_matrix *matR,
		float diagValue,
		float otherValue,
		unsigned char isSetOther);
void MatAdd(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		struct _s_matrix *matB);
void MatBlockAdd(
		struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
		struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
		struct _s_matrix *matB, unsigned int rowPositionB, unsigned int columnPositionB,
		unsigned int rowSizeBlock, unsigned int columnSizeBlock);
void MatSub(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		struct _s_matrix *matB);
void MatBlockSub(
		struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
		struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
		struct _s_matrix *matB, unsigned int rowPositionB, unsigned int columnPositionB,
		unsigned int rowSizeBlock, unsigned int columnSizeBlock);
void MatISub(
		struct _s_matrix *matR,
		struct _s_matrix *matA);
void MatIAdd(
		struct _s_matrix *matR,
		struct _s_matrix *matA);
void MatMult(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		struct _s_matrix *matB);
void MatMult_SquareLowerTriangle(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		struct _s_matrix *matB);
void MatBlockMult(
		struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
		struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
		struct _s_matrix *matB, unsigned int rowPositionB, unsigned int columnPositionB,
		unsigned int rowSize, unsigned int innerSize, unsigned int columnSize);
void MatTranspose(
		struct _s_matrix *matR,
		struct _s_matrix *matA);
void MatBlockTranspose(
		struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
		struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
		unsigned int rowSizeBlock, unsigned int columnSizeBlock);
void QuaternionFromAxisAngle(
		struct _s_matrix *q,
		struct _s_matrix *axis,
		float angle);
void QuaternionMultiply(
		struct _s_matrix *qR,
		struct _s_matrix *qA,
		struct _s_matrix *qB);
void QuaternionMultiplyCore(
		struct _s_matrix *mR,
		struct _s_matrix *qA);
void QuaternionConjugate(
		struct _s_matrix *qR,
		struct _s_matrix *qA);
void QuaternionRotate(
		struct _s_matrix *vR,
		struct _s_matrix *q,
		struct _s_matrix *vA);
void QuaternionInvRotate(
		struct _s_matrix *vR,
		struct _s_matrix *q,
		struct _s_matrix *vA);
void QuaternionToRotateMatrix(
		struct _s_matrix *mR,
		struct _s_matrix *q);
void RotationMatrixToEuler(
		struct _s_matrix *m,
		float *pitch,
		float *roll,
		float *yaw);
void QuaternionDecomposition_h_v(
	struct _s_matrix *q,
	struct _s_matrix *qh,
	struct _s_matrix *qv);
void QuaternionDecomposition_v_h(
	struct _s_matrix *q,
	struct _s_matrix *qv,
	struct _s_matrix *qh);
void QuaternionSetIdentity(struct _s_matrix *q);
void QuaternionToEuler(
		struct _s_matrix *q,
		float *pitch,
		float *roll,
		float *yaw);
void EulerToQuaternion(
		struct _s_matrix *q,
		float pitch,
		float roll,
		float yaw);
float Vect3Dot_SP(
		struct _s_matrix *matA, 
		struct _s_matrix *matB);
void Vect3Cross_SP(
		struct _s_matrix *matR,
		struct _s_matrix *matA,
		struct _s_matrix *matB);
void Vect3Cross(
		struct _s_matrix *matR, unsigned int columnIndexR,
		struct _s_matrix *matA, unsigned int columnIndexA,
		struct _s_matrix *matB, unsigned int columnIndexB
		);
int MatGaussJordanInverse(
		struct _s_matrix *matR,
		struct _s_matrix *matA);

#endif

