#include "Matrix.h"

void matrix_multiply(float* A, float* B, float* C, int rA, int cA, int cB) {
    for (int i = 0; i < rA; i++) {
        for (int j = 0; j < cB; j++) {
            C[i * cB + j] = 0;
            for (int k = 0; k < cA; k++) {
                C[i * cB + j] += A[i * cA + k] * B[k * cB + j];
            }
        }
    }
}

void matrix_add(float* A, float* B, float* C, int r, int c) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            C[i * c + j] = A[i * c + j] + B[i * c + j];
        }
    }
}

void matrix_subtract(float* A, float* B, float* C, int r, int c) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            C[i * c + j] = A[i * c + j] - B[i * c + j];
        }
    }
}

void matrix_transpose(float* A, float* C, int r, int c) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            C[j * r + i] = A[i * c + j];
        }
    }
}

int matrix_invert(float* A, float* inv) {
    // 3x3 matrix inversion
    float det = A[0] * (A[4] * A[8] - A[5] * A[7]) -
                A[1] * (A[3] * A[8] - A[5] * A[6]) +
                A[2] * (A[3] * A[7] - A[4] * A[6]);

    if (det == 0) {
        return 0; // singular matrix
    }

    float invDet = 1.0 / det;

    inv[0] = (A[4] * A[8] - A[5] * A[7]) * invDet;
    inv[1] = (A[2] * A[7] - A[1] * A[8]) * invDet;
    inv[2] = (A[1] * A[5] - A[2] * A[4]) * invDet;
    inv[3] = (A[5] * A[6] - A[3] * A[8]) * invDet;
    inv[4] = (A[0] * A[8] - A[2] * A[6]) * invDet;
    inv[5] = (A[2] * A[3] - A[0] * A[5]) * invDet;
    inv[6] = (A[3] * A[7] - A[4] * A[6]) * invDet;
    inv[7] = (A[1] * A[6] - A[0] * A[7]) * invDet;
    inv[8] = (A[0] * A[4] - A[1] * A[3]) * invDet;

    return 1;
}
