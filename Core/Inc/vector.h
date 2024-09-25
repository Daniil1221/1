
#ifndef INC_VECTOR_
#define INC_VECTOR_




#include "stdlib.h"
#include "math.h"
//#include "stm32f3xx_hal.h"
typedef float Vector3f[3];
typedef float Matrix3f[3][3];
#define Pi 3.14159265358979323846f;
#define PiOver2 1.570796326794897f;
void M3fSetRow(Matrix3f M1, Vector3f V1, uint8_t Row) // заполнение строки в матрице
{
	M1[Row][0] = V1[0];
	M1[Row][1] = V1[1];
	M1[Row][2] = V1[2];
}

void M3fMultiply(Matrix3f M1,Matrix3f M2,Matrix3f MRes)
{
MRes[0][0] = M1[0][0]*M2[0][0] + M1[0][1]*M2[1][0] + M1[0][2]*M2[2][0];
MRes[0][1] = M1[0][0]*M2[0][1] + M1[0][1]*M2[1][1] + M1[0][2]*M2[2][1];
MRes[0][2] = M1[0][0]*M2[0][2] + M1[0][1]*M2[1][2] + M1[0][2]*M2[2][2];

MRes[1][0] = M1[1][0]*M2[0][0] + M1[1][1]*M2[1][0] + M1[1][2]*M2[2][0];
MRes[1][1] = M1[1][0]*M2[0][1] + M1[1][1]*M2[1][1] + M1[1][2]*M2[2][1];
MRes[1][2] = M1[1][0]*M2[0][2] + M1[1][1]*M2[1][2] + M1[1][2]*M2[2][2];

MRes[2][0] = M1[2][0]*M2[0][0] + M1[2][1]*M2[1][0] + M1[2][2]*M2[2][0];
MRes[2][1] = M1[2][0]*M2[0][1] + M1[2][1]*M2[1][1] + M1[2][2]*M2[2][1];
MRes[2][2] = M1[2][0]*M2[0][2] + M1[2][1]*M2[1][2] + M1[2][2]*M2[2][2];
}
void V3fTransform(Vector3f V, Matrix3f M, Vector3f VRes) // умножение вектора на матрицу
{
	VRes[0] = V[0]*M[0][0] + V[1]*M[1][0] + V[2]*M[2][0];
	VRes[1] = V[0]*M[0][1] + V[1]*M[1][1] + V[2]*M[2][1];
	VRes[2] = V[0]*M[0][2] + V[1]*M[1][2] + V[2]*M[2][2];
}
#endif /* INC_VECTOR_ */
