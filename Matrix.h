#ifndef MATRIX_H
#define MATRIX_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

void matrix_multiply(float* A, float* B, float* C, int rA, int cA, int cB);
void matrix_add(float* A, float* B, float* C, int r, int c);
void matrix_subtract(float* A, float* B, float* C, int r, int c);
void matrix_transpose(float* A, float* C, int r, int c);
int matrix_invert(float* A, float* inv);

#endif
