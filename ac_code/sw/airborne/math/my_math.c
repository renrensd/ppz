#include "my_math.h"

float MinorArc(float a, float b)
{
	float ab = a - b;
	if( ab > my_math_pi )
	{
		ab -= 2.0f * my_math_pi;
	}
	else if( ab < -my_math_pi )
	{
		ab += 2.0f * my_math_pi;
	}

	return ab;
}

float LimitAngleTo_npi_ppi(float a)
{
	if( a > my_math_pi )
	{
		a -= 2.0f * my_math_pi;
	}
	else if( a < -my_math_pi )
	{
		a += 2.0f * my_math_pi;
	}

	return a;
}

float LimitAngleTo_0_2pi(float a)
{
	if( a < 0 )
	{
		a += my_math_2pi;
	}
	else if( a > my_math_2pi )
	{
		a -= my_math_2pi;
	}

	return a;
}

void RotationMatrix_2D(struct _s_matrix *R, float theta)
{
	float sin,cos;

	sin = sinf(theta);
	cos = cosf(theta);

	R->data[0] = (+cos);
	R->data[2] = (-sin);
	R->data[1] = (+sin);
	R->data[3] = (+cos);
}

void Rotate_2D(float *x, float *y, float theta)
{
	float temp_x,temp_y;
	float sin,cos;

	sin = sinf(theta);
	cos = cosf(theta);

	temp_x = *x * (+cos) + *y * (-sin);
	temp_y = *x * (+sin) + *y * (+cos);

	*x = temp_x;
	*y = temp_y;
}

struct _s_matrix *MatrixIni(unsigned int rowNum, unsigned int columnNum)
{
	struct _s_matrix *mat;
	float *data;

	// allocate memory for matrix struct
	mat = (struct _s_matrix *)malloc( sizeof(struct _s_matrix) );
	if( mat == 0 )
	{
		return 0;
	}
	// allocate memory for matrix elements data
	data = (float *)calloc( rowNum * columnNum, sizeof(float) );
	if( data == 0 )
	{
		return 0;
	}

	// ini
	mat->rowNum = rowNum;
	mat->columnNum = columnNum;
	mat->data = data;

	return mat;
}

void VectSub(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB)
{
	unsigned int rowNum;
	float *matR_data;
	float *matA_data;
	float *matB_data;

	rowNum = matR->rowNum;
	matR_data = matR->data + (columnIndexR * rowNum);
	matA_data = matA->data + (columnIndexA * rowNum);
	matB_data = matB->data + (columnIndexB * rowNum);

	while (rowNum--)
	{
		*(matR_data++) = *(matA_data++) - *(matB_data++);
	}
}
// r = scaleA*a + scaleB*b
void VectScaleAdd(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB,
	float scaleA, float scaleB)
{
	unsigned int rowNum;
	float *matR_data;
	float *matA_data;
	float *matB_data;

	rowNum = matR->rowNum;
	matR_data = matR->data + (columnIndexR * rowNum);
	matA_data = matA->data + (columnIndexA * rowNum);
	matB_data = matB->data + (columnIndexB * rowNum);

	while (rowNum--)
	{
		*(matR_data++) = (*(matA_data++) * scaleA) + (*(matB_data++) * scaleB);
	}
}
void VectAdd(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB)
{
	unsigned int rowNum;
	float *matR_data;
	float *matA_data;
	float *matB_data;

	rowNum = matR->rowNum;
	matR_data = matR->data + (columnIndexR * rowNum);
	matA_data = matA->data + (columnIndexA * rowNum);
	matB_data = matB->data + (columnIndexB * rowNum);

	while (rowNum--)
	{
		*(matR_data++) = *(matA_data++) + *(matB_data++);
	}
}

void VectCopy(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA)
{
	unsigned int rowNum;
	float *matR_data;
	float *matA_data;

	rowNum = matR->rowNum;
	matR_data = matR->data + (columnIndexR * rowNum);
	matA_data = matA->data + (columnIndexA * rowNum);

	while (rowNum--)
	{
		*(matR_data++) = *(matA_data++);
	}
}

float VectLength(
	struct _s_matrix *matA, unsigned int columnIndex)
{
	float sum = 0;
	unsigned int rowNum;
	float *matA_data;

	rowNum = matA->rowNum;
	matA_data = matA->data + (columnIndex * rowNum);

	while (rowNum--)
	{
		sum += (*matA_data) * (*matA_data);
		matA_data++;
	}
	sum = sqrtf(sum);

	return sum;
}

float VectDistance(
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB)
{
	float sum = 0;
	float temp = 0;
	unsigned int rowNum;
	float *matA_data;
	float *matB_data;

	rowNum = matA->rowNum;
	matA_data = matA->data + (columnIndexA * rowNum);
	matB_data = matB->data + (columnIndexB * rowNum);

	while (rowNum--)
	{
		temp = (*matA_data) - (*matB_data);
		sum += temp * temp;
		matA_data++;
		matB_data++;
	}
	sum = sqrtf(sum);
	return sum;
}

int VectNormalize(
	struct _s_matrix *matA, unsigned int columnIndex)
{
	float sum = 0;
	unsigned int rowNum;
	float *matA_data;

	rowNum = matA->rowNum;
	matA_data = matA->data + (columnIndex * rowNum);

	while (rowNum--)
	{
		sum += (*matA_data) * (*matA_data);
		matA_data++;
	}
	sum = sqrtf(sum);

	if (sum < my_math_eps)
	{
		return -1;
	}

	rowNum = matA->rowNum;
	matA_data -= rowNum;
	while (rowNum--)
	{
		(*matA_data) = (*matA_data) / sum;
		matA_data++;
	}
	return 0;
}

float VectDot(
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB)
{
	float result = 0;
	unsigned int rowNum;
	float *matA_data;
	float *matB_data;

	rowNum = matA->rowNum;
	matA_data = matA->data + (columnIndexA * rowNum);
	matB_data = matB->data + (columnIndexB * rowNum);

	while (rowNum--)
	{
		result += (*matA_data++) * (*matB_data++);

	}
	return result;
}

void VectMult(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB)
{
	unsigned int rowNum;
	float *matR_data;
	float *matA_data;
	float *matB_data;

	rowNum = matR->rowNum;
	matR_data = matR->data + (columnIndexR * rowNum);
	matA_data = matA->data + (columnIndexA * rowNum);
	matB_data = matB->data + (columnIndexB * rowNum);

	while (rowNum--)
	{
		(*matR_data++) = (*matA_data++) * (*matB_data++);
	}
}

void VectScale(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA,
	float scale)
{
	unsigned int rowNum;
	float *matR_data;
	float *matA_data;

	rowNum = matR->rowNum;
	matR_data = matR->data + (columnIndexR * rowNum);
	matA_data = matA->data + (columnIndexA * rowNum);

	while (rowNum--)
	{
		(*matR_data++) = (*matA_data++) * scale;
	}
}

void MatCopy(
	struct _s_matrix *matR,
	struct _s_matrix *matA)
{
	unsigned int cellNum;
	float *matR_data;
	float *matA_data;

	cellNum = matR->rowNum * matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;

	while (cellNum--)
	{
		(*matR_data++) = (*matA_data++);
	}
}

void MatBlockCopy(
	struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
	struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
	unsigned int rowSizeBlock, unsigned int columnSizeBlock)
{
	unsigned int rowSizeR;
	unsigned int rowSizeA;
	unsigned int vLoop;
	float *offsetR_data;
	float *offsetA_data;
	float *matR_data;
	float *matA_data;

	rowSizeR = matR->rowNum;
	rowSizeA = matA->rowNum;
	offsetR_data = matR->data + columnPositionR * rowSizeR + rowPositionR;
	offsetA_data = matA->data + columnPositionA * rowSizeA + rowPositionA;
	matR_data = offsetR_data;
	matA_data = offsetA_data;

	while (columnSizeBlock--)
	{
		vLoop = rowSizeBlock;
		while (vLoop--)
		{
			(*matR_data++) = (*matA_data++);
		}
		matR_data -= rowSizeBlock;
		matA_data -= rowSizeBlock;
		matR_data += rowSizeR;
		matA_data += rowSizeA;
	}
}

void MatScale(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	float scale)
{
	unsigned int cellNum;
	float *matR_data;
	float *matA_data;

	cellNum = matR->rowNum * matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;

	while (cellNum--)
	{
		(*matR_data++) = (*matA_data++) * scale;
	}
}

void MatSetValue(
	struct _s_matrix *matR, float value)
{
	unsigned int cellNum;
	float *matR_data;

	cellNum = matR->rowNum * matR->columnNum;
	matR_data = matR->data;

	while (cellNum--)
	{
		(*matR_data++) = value;
	}
}

void MatBlockSetValue(
	struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
	int rowSizeBlock, int columnSizeBlock,
	float value)
{
	unsigned int rowSizeR;
	unsigned int vLoop;
	float *matR_data;
	float *offsetR_data;

	rowSizeR = matR->rowNum;
	offsetR_data = matR->data + columnPositionR * rowSizeR + rowPositionR;
	matR_data = offsetR_data;

	while (columnSizeBlock--)
	{
		vLoop = rowSizeBlock;
		while (vLoop--)
		{
			(*matR_data++) = value;
		}
		matR_data -= rowSizeBlock;
		matR_data += rowSizeR;
	}
}

void MatSetDiagonal(
	struct _s_matrix *matR,
	float diagValue,
	float otherValue,
	unsigned char isSetOther)
{
	unsigned int rowSizeR;
	unsigned int columnSizeR;
	unsigned int hLoop;
	unsigned int vLoop;
	float *matR_data;

	rowSizeR = matR->rowNum;
	columnSizeR = matR->columnNum;
	matR_data = matR->data;

	hLoop = 0;
	while (hLoop < columnSizeR)
	{
		vLoop = 0;
		while (vLoop < rowSizeR)
		{
			if (hLoop == vLoop)
			{
				(*matR_data++) = diagValue;
			}
			else
			{
				if(isSetOther)
				{
					(*matR_data++) = otherValue;
				}
				else
				{
					matR_data++;
				}
			}
			vLoop++;
		}
		hLoop++;
	}
}

void MatAdd(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	struct _s_matrix *matB)
{
	unsigned int cellNum;
	float *matR_data;
	float *matA_data;
	float *matB_data;

	cellNum = matR->rowNum * matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;
	matB_data = matB->data;

	while (cellNum--)
	{
		(*matR_data++) = (*matA_data++) + (*matB_data++);
	}
}

void MatScaleAdd(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	float scale)
{
	unsigned int cellNum;
	float *matR_data;
	float *matA_data;

	cellNum = matR->rowNum * matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;

	while (cellNum--)
	{
		*matR_data = (*matR_data) + ((*matA_data++) * scale);
		matR_data++;
	}
}

void MatSub(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	struct _s_matrix *matB)
{
	unsigned int cellNum;
	float *matR_data;
	float *matA_data;
	float *matB_data;

	cellNum = matR->rowNum * matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;
	matB_data = matB->data;

	while (cellNum--)
	{
		(*matR_data++) = (*matA_data++) - (*matB_data++);
	}
}

void MatBlockAdd(
	struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
	struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
	struct _s_matrix *matB, unsigned int rowPositionB, unsigned int columnPositionB,
	unsigned int rowSizeBlock, unsigned int columnSizeBlock)
{
	unsigned int rowSizeR;
	unsigned int rowSizeA;
	unsigned int rowSizeB;
	unsigned int vLoop;
	float *matR_data;
	float *matA_data;
	float *matB_data;
	float *offsetR_data;
	float *offsetA_data;
	float *offsetB_data;

	rowSizeR = matR->rowNum;
	rowSizeA = matA->rowNum;
	rowSizeB = matB->rowNum;
	offsetR_data = matR->data + columnPositionR * rowSizeR + rowPositionR;
	offsetA_data = matA->data + columnPositionA * rowSizeA + rowPositionA;
	offsetB_data = matB->data + columnPositionB * rowSizeB + rowPositionB;
	matR_data = offsetR_data;
	matA_data = offsetA_data;
	matB_data = offsetB_data;

	while (columnSizeBlock--)
	{
		vLoop = rowSizeBlock;
		while (vLoop--)
		{
			(*matR_data++) = (*matA_data++) + (*matB_data++);
		}
		matR_data -= rowSizeBlock;
		matA_data -= rowSizeBlock;
		matB_data -= rowSizeBlock;
		matR_data += rowSizeR;
		matA_data += rowSizeA;
		matB_data += rowSizeB;
	}
}

void MatBlockSub(
	struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
	struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
	struct _s_matrix *matB, unsigned int rowPositionB, unsigned int columnPositionB,
	unsigned int rowSizeBlock, unsigned int columnSizeBlock)
{
	unsigned int rowSizeR;
	unsigned int rowSizeA;
	unsigned int rowSizeB;
	unsigned int vLoop;
	float *matR_data;
	float *matA_data;
	float *matB_data;
	float *offsetR_data;
	float *offsetA_data;
	float *offsetB_data;

	rowSizeR = matR->rowNum;
	rowSizeA = matA->rowNum;
	rowSizeB = matB->rowNum;
	offsetR_data = matR->data + columnPositionR * rowSizeR + rowPositionR;
	offsetA_data = matA->data + columnPositionA * rowSizeA + rowPositionA;
	offsetB_data = matB->data + columnPositionB * rowSizeB + rowPositionB;
	matR_data = offsetR_data;
	matA_data = offsetA_data;
	matB_data = offsetB_data;

	while (columnSizeBlock--)
	{
		vLoop = rowSizeBlock;
		while (vLoop--)
		{
			(*matR_data++) = (*matA_data++) - (*matB_data++);
		}
		matR_data -= rowSizeBlock;
		matA_data -= rowSizeBlock;
		matB_data -= rowSizeBlock;
		matR_data += rowSizeR;
		matA_data += rowSizeA;
		matB_data += rowSizeB;
	}
}

void MatISub(
	struct _s_matrix *matR,
	struct _s_matrix *matA)
{
	unsigned int rowSizeR;
	unsigned int columnSizeR;
	unsigned int hLoop;
	unsigned int vLoop;
	float *matR_data;
	float *matA_data;

	rowSizeR = matR->rowNum;
	columnSizeR = matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;

	hLoop = 0;
	while (hLoop < columnSizeR)
	{
		vLoop = 0;
		while (vLoop < rowSizeR)
		{
			if (hLoop == vLoop)
			{
				(*matR_data++) = 1.0f - (*matA_data++);
			}
			else
			{
				(*matR_data++) = - (*matA_data++);
			}
			vLoop++;
		}
		hLoop++;
	}
}

void MatIAdd(
	struct _s_matrix *matR,
	struct _s_matrix *matA)
{
	unsigned int rowSizeR;
	unsigned int columnSizeR;
	unsigned int hLoop;
	unsigned int vLoop;
	float *matR_data;
	float *matA_data;

	rowSizeR = matR->rowNum;
	columnSizeR = matR->columnNum;
	matR_data = matR->data;
	matA_data = matA->data;

	hLoop = 0;
	while (hLoop < columnSizeR)
	{
		vLoop = 0;
		while (vLoop < rowSizeR)
		{
			if (hLoop == vLoop)
			{
				(*matR_data++) = 1.0f + (*matA_data++);
			}
			else
			{
				(*matR_data++) = (*matA_data++);
			}
			vLoop++;
		}
		hLoop++;
	}
}

void MatMult(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	struct _s_matrix *matB)
{
	unsigned int rowSize;
	unsigned int columnSize;
	unsigned int innerSize;
	unsigned int vLoop;
	unsigned int innerLoop;
	float *matR_data;
	float *matA_data;
	float *matB_data;
	float sum;

	rowSize = matA->rowNum;
	columnSize = matB->columnNum;
	innerSize = matA->columnNum;

	matR_data = matR->data;
	matB_data = matB->data;

	while (columnSize--)
	{
		vLoop = 0;
		while (vLoop < rowSize)
		{
			matA_data = matA->data + vLoop;
			sum = 0;
			innerLoop = 0;
			while (innerLoop < innerSize)
			{
				sum += (*matA_data) * (*matB_data);
				matA_data += rowSize;
				matB_data++;
				innerLoop++;
			}
			(*matR_data++) = sum;
			matB_data -= innerSize;
			vLoop++;
		}
		matB_data += innerSize;
	}
}

void MatMult_SquareLowerTriangle(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	struct _s_matrix *matB)
{
	unsigned int rowSize;
	unsigned int columnSize;
	unsigned int innerSize;
	unsigned int currentColumn;
	unsigned int vLoop;
	unsigned int innerLoop;
	float *matR_data;
	float *matA_data;
	float *matB_data;
	float sum;

	rowSize = matA->rowNum;
	columnSize = matB->columnNum;
	innerSize = matA->columnNum;

	matR_data = matR->data;
	matB_data = matB->data;

	currentColumn = 0;
	while (columnSize--)
	{
		vLoop = currentColumn;
		++currentColumn;
		while (vLoop < rowSize)
		{
			matA_data = matA->data + vLoop;
			sum = 0;
			innerLoop = 0;
			while (innerLoop < innerSize)
			{
				sum += (*matA_data) * (*matB_data);
				matA_data += rowSize;
				matB_data++;
				innerLoop++;
			}
			(*matR_data++) = sum;
			matB_data -= innerSize;
			vLoop++;
		}
		matB_data += innerSize;
		matR_data += currentColumn;
	}
}

void MatBlockMult(
	struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
	struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
	struct _s_matrix *matB, unsigned int rowPositionB, unsigned int columnPositionB,
	unsigned int rowSize, unsigned int innerSize, unsigned int columnSize)
{
	unsigned int rowSizeR;
	unsigned int rowSizeA;
	unsigned int rowSizeB;
	unsigned int vLoop;
	unsigned int innerLoop;
	float *matR_data;
	float *matA_data;
	float *matB_data;
	float *offsetR_data;
	float *offsetA_data;
	float *offsetB_data;
	float sum;

	rowSizeR = matR->rowNum;
	rowSizeA = matA->rowNum;
	rowSizeB = matB->rowNum;
	offsetR_data = matR->data + columnPositionR * rowSizeR + rowPositionR;
	offsetA_data = matA->data + columnPositionA * rowSizeA + rowPositionA;
	offsetB_data = matB->data + columnPositionB * rowSizeB + rowPositionB;
	matR_data = offsetR_data;
	matB_data = offsetB_data;

	while (columnSize--)
	{
		vLoop = 0;
		while (vLoop < rowSize)
		{
			matA_data = offsetA_data + vLoop;
			sum = 0;
			innerLoop = 0;
			while (innerLoop < innerSize)
			{
				sum += (*matA_data) * (*matB_data);
				matA_data += rowSizeA;
				matB_data++;
				innerLoop++;
			}
			(*matR_data++) = sum;
			matB_data -= innerSize;
			vLoop++;
		}
		matR_data -= rowSize;
		matR_data += rowSizeR;
		matB_data += rowSizeB;
	}
}

void MatTranspose(
	struct _s_matrix *matR,
	struct _s_matrix *matA)
{
	unsigned int rowSizeR;
	unsigned int columnSizeR;
	unsigned int hLoop;
	unsigned int vLoop;
	float *matR_data;
	float *matA_data;

	rowSizeR = matR->rowNum;
	columnSizeR = matR->columnNum;
	matR_data = matR->data;

	hLoop = 0;
	while (hLoop < columnSizeR)
	{
		vLoop = 0;
		matA_data = matA->data + hLoop;
		while (vLoop < rowSizeR)
		{
			*matR_data = *matA_data;
			matR_data++;
			matA_data += columnSizeR;
			vLoop++;
		}
		hLoop++;
	}
}
//rowSizeBlock??columnSizeBlock?????????С
void MatBlockTranspose(
	struct _s_matrix *matR, unsigned int rowPositionR, unsigned int columnPositionR,
	struct _s_matrix *matA, unsigned int rowPositionA, unsigned int columnPositionA,
	unsigned int rowSizeBlock, unsigned int columnSizeBlock)
{
	unsigned int rowSizeR;
	unsigned int rowSizeA;
	unsigned int hLoop;
	unsigned int vLoop;
	float *matR_data;
	float *matA_data;
	float *offsetR_data;
	float *offsetA_data;

	rowSizeR = matR->rowNum;
	rowSizeA = matA->rowNum;
	offsetR_data = matR->data + columnPositionR * rowSizeR + rowPositionR;
	offsetA_data = matA->data + columnPositionA * rowSizeA + rowPositionA;
	matR_data = offsetR_data;

	hLoop = 0;
	while (hLoop < columnSizeBlock)
	{
		vLoop = 0;
		matA_data = offsetA_data + hLoop;
		while (vLoop < rowSizeBlock)
		{
			*matR_data = *matA_data;
			matR_data++;
			matA_data += rowSizeA;
			vLoop++;
		}
		matR_data -= rowSizeBlock;
		matR_data += rowSizeR;

		hLoop++;
	}
}

void QuaternionFromAxisAngle(
	struct _s_matrix *q,
	struct _s_matrix *axis,
	float angle)
{
	float theta;
	float sin;
	float cos;

	theta = angle * 0.5f;
	sin = sinf(theta);
	cos = cosf(theta);

	VectNormalize(axis, 0);

	q->data[0] = cos;
	q->data[1] = axis->data[0] * sin;
	q->data[2] = axis->data[1] * sin;
	q->data[3] = axis->data[2] * sin;
}

void QuaternionMultiply(
	struct _s_matrix *qR,
	struct _s_matrix *qA,
	struct _s_matrix *qB)
{
	float a0b0 = qA->data[0] * qB->data[0];
	float a1b1 = qA->data[1] * qB->data[1];
	float a2b2 = qA->data[2] * qB->data[2];
	float a3b3 = qA->data[3] * qB->data[3];
	float a0b1 = qA->data[0] * qB->data[1];
	float a0b2 = qA->data[0] * qB->data[2];
	float a0b3 = qA->data[0] * qB->data[3];
	float a1b0 = qA->data[1] * qB->data[0];
	float a1b2 = qA->data[1] * qB->data[2];
	float a1b3 = qA->data[1] * qB->data[3];
	float a2b0 = qA->data[2] * qB->data[0];
	float a2b1 = qA->data[2] * qB->data[1];
	float a2b3 = qA->data[2] * qB->data[3];
	float a3b0 = qA->data[3] * qB->data[0];
	float a3b1 = qA->data[3] * qB->data[1];
	float a3b2 = qA->data[3] * qB->data[2];

	qR->data[0] = a0b0 - a1b1 - a2b2 - a3b3;
	qR->data[1] = a1b0 + a0b1 + a2b3 - a3b2;
	qR->data[2] = a2b0 + a0b2 + a3b1 - a1b3;
	qR->data[3] = a3b0 + a0b3 + a1b2 - a2b1;
}

void QuaternionMultiplyCore(
	struct _s_matrix *mR,
	struct _s_matrix *qA)
{
	mR->data[0+0*4] = qA->data[0];
	mR->data[1+1*4] = qA->data[0];
	mR->data[2+2*4] = qA->data[0];
	mR->data[3+3*4] = qA->data[0];

	mR->data[1+0*4] = qA->data[1];
	mR->data[2+0*4] = qA->data[2];
	mR->data[3+0*4] = qA->data[3];

	mR->data[0+1*4] = -qA->data[1];
	mR->data[0+2*4] = -qA->data[2];
	mR->data[0+3*4] = -qA->data[3];

	mR->data[2+1*4] = qA->data[3];
	mR->data[3+1*4] = -qA->data[2];

	mR->data[1+2*4] = -qA->data[3];
	mR->data[1+3*4] = qA->data[2];

	mR->data[3+2*4] = qA->data[1];
	mR->data[2+3*4] = -qA->data[1];
}

void QuaternionConjugate(
	struct _s_matrix *qR,
	struct _s_matrix *qA)
{
	qR->data[0] = qA->data[0];
	qR->data[1] = -qA->data[1];
	qR->data[2] = -qA->data[2];
	qR->data[3] = -qA->data[3];
}

void QuaternionRotate(
	struct _s_matrix *vR,
	struct _s_matrix *q,
	struct _s_matrix *vA)
{
	float R00, R10, R20, R01, R11, R21, R02, R12, R22;

	float q0_q0 = q->data[0] * q->data[0];
	float q1_q1 = q->data[1] * q->data[1];
	float q2_q2 = q->data[2] * q->data[2];
	float q3_q3 = q->data[3] * q->data[3];
	float q0_q1 = q->data[0] * q->data[1];
	float q0_q2 = q->data[0] * q->data[2];
	float q0_q3 = q->data[0] * q->data[3];
	float q1_q2 = q->data[1] * q->data[2];
	float q1_q3 = q->data[1] * q->data[3];
	float q2_q3 = q->data[2] * q->data[3];
	float v1,v2,v3;

	R00 = q0_q0 + q1_q1 - q2_q2 - q3_q3;
	R10 = 2.0f * (q1_q2 + q0_q3);
	R20 = 2.0f * (q1_q3 - q0_q2);

	R01 = 2.0f * (q1_q2 - q0_q3);
	R11 = q0_q0 - q1_q1 + q2_q2 - q3_q3;
	R21 = 2.0f * (q2_q3 + q0_q1);

	R02 = 2.0f * (q1_q3 + q0_q2);
	R12 = 2.0f * (q2_q3 - q0_q1);
	R22 = q0_q0 - q1_q1 - q2_q2 + q3_q3;

	if( vR->rowNum == 4 )
	{
		v1 = vA->data[1];
		v2 = vA->data[2];
		v3 = vA->data[3];

		vR->data[0] = vA->data[0] * (q0_q0 + q1_q1 + q2_q2 + q3_q3);
		vR->data[1] = v1 * R00 + v2 * R01 + v3 * R02;
		vR->data[2] = v1 * R10 + v2 * R11 + v3 * R12;
		vR->data[3] = v1 * R20 + v2 * R21 + v3 * R22;
	}
	else
	{
		v1 = vA->data[0];
		v2 = vA->data[1];
		v3 = vA->data[2];

		vR->data[0] = v1 * R00 + v2 * R01 + v3 * R02;
		vR->data[1] = v1 * R10 + v2 * R11 + v3 * R12;
		vR->data[2] = v1 * R20 + v2 * R21 + v3 * R22;
	}
}

void QuaternionInvRotate(
	struct _s_matrix *vR,
	struct _s_matrix *q,
	struct _s_matrix *vA)
{
	float R00, R10, R20, R01, R11, R21, R02, R12, R22;

	float q0_q0 = q->data[0] * q->data[0];
	float q1_q1 = q->data[1] * q->data[1];
	float q2_q2 = q->data[2] * q->data[2];
	float q3_q3 = q->data[3] * q->data[3];
	float q0_q1 = q->data[0] * q->data[1];
	float q0_q2 = q->data[0] * q->data[2];
	float q0_q3 = q->data[0] * q->data[3];
	float q1_q2 = q->data[1] * q->data[2];
	float q1_q3 = q->data[1] * q->data[3];
	float q2_q3 = q->data[2] * q->data[3];
	float v1,v2,v3;

	R00 = q0_q0 + q1_q1 - q2_q2 - q3_q3;
	R10 = 2.0f * (q1_q2 - q0_q3);
	R20 = 2.0f * (q1_q3 + q0_q2);

	R01 = 2.0f * (q1_q2 + q0_q3);
	R11 = q0_q0 - q1_q1 + q2_q2 - q3_q3;
	R21 = 2.0f * (q2_q3 - q0_q1);

	R02 = 2.0f * (q1_q3 - q0_q2);
	R12 = 2.0f * (q2_q3 + q0_q1);
	R22 = q0_q0 - q1_q1 - q2_q2 + q3_q3;

	if( vR->rowNum == 4 )
	{
		v1 = vA->data[1];
		v2 = vA->data[2];
		v3 = vA->data[3];

		vR->data[0] = vA->data[0] * (q0_q0 + q1_q1 + q2_q2 + q3_q3);
		vR->data[1] = v1 * R00 + v2 * R01 + v3 * R02;
		vR->data[2] = v1 * R10 + v2 * R11 + v3 * R12;
		vR->data[3] = v1 * R20 + v2 * R21 + v3 * R22;
	}
	else
	{
		v1 = vA->data[0];
		v2 = vA->data[1];
		v3 = vA->data[2];

		vR->data[0] = v1 * R00 + v2 * R01 + v3 * R02;
		vR->data[1] = v1 * R10 + v2 * R11 + v3 * R12;
		vR->data[2] = v1 * R20 + v2 * R21 + v3 * R22;
	}
}

void QuaternionToRotateMatrix(
	struct _s_matrix *mR,
	struct _s_matrix *q)
{
	float q0_q0 = q->data[0] * q->data[0];
	float q1_q1 = q->data[1] * q->data[1];
	float q2_q2 = q->data[2] * q->data[2];
	float q3_q3 = q->data[3] * q->data[3];
	float q0_q1 = q->data[0] * q->data[1];
	float q0_q2 = q->data[0] * q->data[2];
	float q0_q3 = q->data[0] * q->data[3];
	float q1_q2 = q->data[1] * q->data[2];
	float q1_q3 = q->data[1] * q->data[3];
	float q2_q3 = q->data[2] * q->data[3];

	mR->data[0 + 0*3] = q0_q0 + q1_q1 - q2_q2 - q3_q3;
	mR->data[1 + 0*3] = 2.0f * (q1_q2 + q0_q3);
	mR->data[2 + 0*3] = 2.0f * (q1_q3 - q0_q2);

	mR->data[0 + 1*3] = 2.0f * (q1_q2 - q0_q3);
	mR->data[1 + 1*3] = q0_q0 - q1_q1 + q2_q2 - q3_q3;
	mR->data[2 + 1*3] = 2.0f * (q2_q3 + q0_q1);

	mR->data[0 + 2*3] = 2.0f * (q1_q3 + q0_q2);
	mR->data[1 + 2*3] = 2.0f * (q2_q3 - q0_q1);
	mR->data[2 + 2*3] = q0_q0 - q1_q1 - q2_q2 + q3_q3;
}

void RotationMatrixToEuler(
	struct _s_matrix *m,
	float *pitch,
	float *roll,
	float *yaw)
{
	if(pitch)
	{
		*pitch = asinf(- m->data[2 + 0*3]);
	}
	if(roll)
	{
		*roll = atan2f(m->data[2 + 1*3], m->data[2 + 2*3]);
	}
	if(yaw)
	{
		*yaw = atan2f(m->data[1 + 0*3], m->data[0 + 0*3]);
	}
}

void QuaternionDecomposition_h_v(
	struct _s_matrix *q,
	struct _s_matrix *qh,
	struct _s_matrix *qv)
{
	float theta_2;
	MATRIX_DEF(temp_qh, 4, 1);

	MATRIX_INI(temp_qh);

	theta_2 = atan2f(q->data[3], q->data[0]);
	temp_qh.data[0] = cosf(theta_2);
	temp_qh.data[1] = 0;
	temp_qh.data[2] = 0;
	temp_qh.data[3] = -sinf(theta_2);

	if(qh != 0)
	{
		qh->data[0] = temp_qh.data[0];
		qh->data[1] = 0;
		qh->data[2] = 0;
		qh->data[3] = -temp_qh.data[3];
	}
	if(qv != 0)
	{
		QuaternionMultiply(qv, &temp_qh, q);
	}
}

void QuaternionDecomposition_v_h(
	struct _s_matrix *q,
	struct _s_matrix *qv,
	struct _s_matrix *qh)
{
	float theta_2;
	MATRIX_DEF(temp_qh, 4, 1);

	MATRIX_INI(temp_qh);

	theta_2 = atan2f(q->data[3], q->data[0]);
	temp_qh.data[0] = cosf(theta_2);
	temp_qh.data[1] = 0;
	temp_qh.data[2] = 0;
	temp_qh.data[3] = -sinf(theta_2);

	if(qh != 0)
	{
		qh->data[0] = temp_qh.data[0];
		qh->data[1] = 0;
		qh->data[2] = 0;
		qh->data[3] = -temp_qh.data[3];
	}
	if(qv != 0)
	{
		QuaternionMultiply(qv, q, &temp_qh);
	}
}

void QuaternionSetIdentity(struct _s_matrix *q)
{
	q->data[0] = 1;
	q->data[1] = 0;
	q->data[2] = 0;
	q->data[3] = 0;
}

void QuaternionToEuler(
	struct _s_matrix *q,
	float *pitch,
	float *roll,
	float *yaw)
{
	float R21, R22, R20, R10, R00;

	float q1_q1 = q->data[1] * q->data[1];
	float q2_q2 = q->data[2] * q->data[2];
	float q3_q3 = q->data[3] * q->data[3];
	float q0_q1 = q->data[0] * q->data[1];
	float q0_q2 = q->data[0] * q->data[2];
	float q0_q3 = q->data[0] * q->data[3];
	float q1_q2 = q->data[1] * q->data[2];
	float q1_q3 = q->data[1] * q->data[3];
	float q2_q3 = q->data[2] * q->data[3];

	R21 = 2.0f * (q0_q1 + q2_q3);
	R22 = 1.0f - 2.0f * (q1_q1 + q2_q2);
	R20 = 2.0f * (q1_q3 - q0_q2);
	R10 = 2.0f * (q0_q3 + q1_q2);
	R00 = 1.0f - 2.0f * (q2_q2 + q3_q3);

	if(pitch)
	{
		*pitch = asinf(-R20);
	}
	if(roll)
	{
		*roll = atan2f(R21, R22);
	}
	if(yaw)
	{
		*yaw = atan2f(R10, R00);
	}
}

void EulerToQuaternion(
	struct _s_matrix *q,
	float pitch,
	float roll,
	float yaw)
{
	float sin_roll;
	float cos_roll;
	float sin_pitch;
	float cos_pitch;
	float sin_yaw;
	float cos_yaw;

	pitch /= 2.0f;
	roll /= 2.0f;
	yaw /= 2.0f;

	sin_roll = sinf(roll);
	cos_roll = cosf(roll);
	sin_pitch = sinf(pitch);
	cos_pitch = cosf(pitch);
	sin_yaw = sinf(yaw);
	cos_yaw = cosf(yaw);

	q->data[0] = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
	q->data[1] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
	q->data[2] = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
	q->data[3] = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;

	VectNormalize(q, 0);
}

float Vect3Dot_SP(
	struct _s_matrix *matA,
	struct _s_matrix *matB)
{
	return matA->data[0]*matB->data[0] + matA->data[1]*matB->data[1] + matA->data[2]*matB->data[2];
}

void Vect3Cross_SP(
	struct _s_matrix *matR,
	struct _s_matrix *matA,
	struct _s_matrix *matB)
{
	matR->data[0] = matA->data[1] * matB->data[2] - matB->data[1] * matA->data[2];
	matR->data[1] = matB->data[0] * matA->data[2] - matA->data[0] * matB->data[2];
	matR->data[2] = matA->data[0] * matB->data[1] - matB->data[0] * matA->data[1];
}

/*
 * | i  j  k  |
 * | x1 y1 z1 |
 * | x2 y2 z2 |
 */
void Vect3Cross(
	struct _s_matrix *matR, unsigned int columnIndexR,
	struct _s_matrix *matA, unsigned int columnIndexA,
	struct _s_matrix *matB, unsigned int columnIndexB
)
{
	unsigned int rowNum;
	float *vR_data;
	float *v1_data;
	float *v2_data;

	float y1z2;
	float y2z1;
	float x2z1;
	float x1z2;
	float x1y2;
	float x2y1;

	rowNum = matR->rowNum;
	vR_data = matR->data + (columnIndexR * rowNum);
	v1_data = matA->data + (columnIndexA * rowNum);
	v2_data = matB->data + (columnIndexB * rowNum);

	y1z2 = v1_data[1] * v2_data[2];
	y2z1 = v2_data[1] * v1_data[2];
	x2z1 = v2_data[0] * v1_data[2];
	x1z2 = v1_data[0] * v2_data[2];
	x1y2 = v1_data[0] * v2_data[1];
	x2y1 = v2_data[0] * v1_data[1];

	vR_data[0] = y1z2 - y2z1;
	vR_data[1] = x2z1 - x1z2;
	vR_data[2] = x1y2 - x2y1;
}

int MatGaussJordanInverse(
	struct _s_matrix *matR,
	struct _s_matrix *matA)
{
	unsigned int dimNum,iPivot,iMx,iRow,iColumn;
	float det,pivot,factor,temp;
	struct _s_matrix A;
	float *A_data;
	float *R_data;
	float *AP_data;
	float *RP_data;

	det = 1;
	dimNum = matA->rowNum;

	A.rowNum = dimNum;
	A.columnNum = dimNum;
	A.data = (float*)calloc(dimNum*dimNum, sizeof(float));

	MatSetDiagonal(matR, 1, 0, 1);
	MatCopy(&A, matA);

	for(iPivot = 0; iPivot < dimNum; ++iPivot)
	{
		// find maximum element in the pivot column
		A_data = A.data + dimNum * iPivot + iPivot;
		pivot = 0;
		iMx = iPivot;
		for(iRow = iPivot; iRow < dimNum; ++iRow)
		{
			if( fabsf(*A_data) > fabsf(pivot) )
			{
				pivot = *A_data;
				iMx = iRow;
			}
			A_data++;
		}

		// interchange row
		if (iMx != iPivot)
		{
			R_data = matR->data + iPivot;
			A_data = matR->data + iMx;
			for(iColumn = 0; iColumn < dimNum; ++iColumn)
			{
				temp = *A_data;
				*A_data = *R_data;
				*R_data = temp;

				R_data += dimNum;
				A_data += dimNum;
			}
			R_data = A.data + iPivot + iPivot * dimNum;
			A_data = A.data + iMx + iPivot * dimNum;
			for(iColumn = iPivot; iColumn < dimNum; ++iColumn)
			{
				temp = *A_data;
				*A_data = *R_data;
				*R_data = temp;

				R_data += dimNum;
				A_data += dimNum;
			}
		}

		// check determinant
		if(pivot < my_math_eps)
		{
			free(A.data);
			return -1;
		}
		det = det * pivot;

		// Normalize the pivot row
		R_data = matR->data + iPivot;
		for(iColumn = 0; iColumn < dimNum; ++iColumn)
		{
			*R_data = *R_data / pivot;
			R_data += dimNum;
		}
		A_data = A.data + iPivot + iPivot * dimNum;
		for(iColumn = iPivot; iColumn < dimNum; ++iColumn)
		{
			*A_data = *A_data / pivot;
			A_data += dimNum;
		}

		// Add a multiple of the pivot row to each row
		for (iRow = 0; iRow < dimNum; ++iRow)
		{
			if(iRow != iPivot)
			{
				RP_data = matR->data + iPivot;
				AP_data = A.data + iPivot;
				R_data = matR->data + iRow;
				A_data = A.data + iRow;
				factor = *(A_data + iPivot * dimNum);
				for(iColumn = 0; iColumn < dimNum; ++iColumn)
				{
					*R_data -= factor * (*RP_data);
					*A_data -= factor * (*AP_data);
					R_data += dimNum;
					A_data += dimNum;
					RP_data += dimNum;
					AP_data += dimNum;
				}
			}
		}
	}

	free(A.data);
	return 0;
}
