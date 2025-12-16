/**
 * @file matrix.h
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief this file contains some matrix operations for the calculating the matrix transformation, transpose and generating matrix state representation
 * @version 0.1
 * @date 2024-02-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "math.h"   
#include <stdio.h>
#define PI 3.14159265359
#define DEG_TO_RAD  degree * (pi / 180)

float deg_to_rad(float degree)
{
    float pi = 3.14159265359;
    return (degree * (pi / 180));
}

// Function to swap two rows of a 4x4 matrix
void swapRows(float matrix[4][4], int row1, int row2) {
    for (int i = 0; i < 4; i++) {
        float temp = matrix[row1][i];
        matrix[row1][i] = matrix[row2][i];
        matrix[row2][i] = temp;
    }
}

// Function to perform row operations to obtain the identity matrix
void rowOperations(float matrix[4][4], float result[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == j) {
                result[i][j] = 1.0;
            } else {
                result[i][j] = 0.0;
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        float pivot = matrix[i][i];
        if (pivot == 0) {
            // If the pivot is zero, swap rows to make it non-zero
            for (int j = i + 1; j < 4; j++) {
                if (matrix[j][i] != 0) {
                    swapRows(matrix, i, j);
                    swapRows(result, i, j);
                    break;
                }
            }
            pivot = matrix[i][i];
        }

        for (int j = 0; j < 4; j++) {
            matrix[i][j] /= pivot;
            result[i][j] /= pivot;
        }

        for (int k = 0; k < 4; k++) {
            if (k != i) {
                float factor = matrix[k][i];
                for (int j = 0; j < 4; j++) {
                    matrix[k][j] -= factor * matrix[i][j];
                    result[k][j] -= factor * result[i][j];
                }
            }
        }
    }
}


int gluInvertMatrix(const float m[4][4], float invOut[4][4])
{
    float inv[16], det;
    int i, j;

    inv[0] = m[1][1]  * m[2][2] * m[3][3] - 
             m[1][1]  * m[2][3] * m[3][2] - 
             m[2][1]  * m[1][2]  * m[3][3] + 
             m[2][1]  * m[1][3]  * m[3][2] +
             m[3][1] * m[1][2]  * m[2][3] - 
             m[3][1] * m[1][3]  * m[2][2];

    inv[4] = -m[1][0]  * m[2][2] * m[3][3] + 
              m[1][0]  * m[2][3] * m[3][2] + 
              m[2][0]  * m[1][2]  * m[3][3] - 
              m[2][0]  * m[1][3]  * m[3][2] - 
              m[3][0] * m[1][2]  * m[2][3] + 
              m[3][0] * m[1][3]  * m[2][2];

    inv[8] = m[1][0]  * m[2][1] * m[3][3] - 
             m[1][0]  * m[2][3] * m[3][1] - 
             m[2][0]  * m[1][1] * m[3][3] + 
             m[2][0]  * m[1][3] * m[3][1] + 
             m[3][0] * m[1][1] * m[2][3] - 
             m[3][0] * m[1][3] * m[2][1];

    inv[12] = -m[1][0]  * m[2][1] * m[3][2] + 
               m[1][0]  * m[2][2] * m[3][1] +
               m[2][0]  * m[1][1] * m[3][2] - 
               m[2][0]  * m[1][2] * m[3][1] - 
               m[3][0] * m[1][1] * m[2][2] + 
               m[3][0] * m[1][2] * m[2][1];

    inv[1] = -m[0][1]  * m[2][2] * m[3][3] + 
              m[0][1]  * m[2][3] * m[3][2] + 
              m[2][1]  * m[0][2] * m[3][3] - 
              m[2][1]  * m[0][3] * m[3][2] - 
              m[3][1] * m[0][2] * m[2][3] + 
              m[3][1] * m[0][3] * m[2][2];

    inv[5] = m[0][0]  * m[2][2] * m[3][3] - 
             m[0][0]  * m[2][3] * m[3][2] - 
             m[2][0]  * m[0][2] * m[3][3] + 
             m[2][0]  * m[0][3] * m[3][2] + 
             m[3][0] * m[0][2] * m[2][3] - 
             m[3][0] * m[0][3] * m[2][2];

    inv[9] = -m[0][0]  * m[2][1] * m[3][3] + 
              m[0][0]  * m[2][3] * m[3][1] + 
              m[2][0]  * m[0][1] * m[3][3] - 
              m[2][0]  * m[0][3] * m[3][1] - 
              m[3][0] * m[0][1] * m[2][3] + 
              m[3][0] * m[0][3] * m[2][1];

    inv[13] = m[0][0]  * m[2][1] * m[3][2] - 
              m[0][0]  * m[2][2] * m[3][1] - 
              m[2][0]  * m[0][1] * m[3][2] + 
              m[2][0]  * m[0][2] * m[3][1] + 
              m[3][0] * m[0][1] * m[2][2] - 
              m[3][0] * m[0][2] * m[2][1];

    inv[2] = m[0][1]  * m[1][2] * m[3][3] - 
             m[0][1]  * m[1][3] * m[3][2] - 
             m[1][1]  * m[0][2] * m[3][3] + 
             m[1][1]  * m[0][3] * m[3][2] + 
             m[3][1] * m[0][2] * m[1][3] - 
             m[3][1] * m[0][3] * m[1][2];

    inv[6] = -m[0][0]  * m[1][2] * m[3][3] + 
              m[0][0]  * m[1][3] * m[3][2] + 
              m[1][0]  * m[0][2] * m[3][3] - 
              m[1][0]  * m[0][3] * m[3][2] - 
              m[3][0] * m[0][2] * m[1][3] + 
              m[3][0] * m[0][3] * m[1][2];

    inv[10] = m[0][0]  * m[1][1] * m[3][3] - 
              m[0][0]  * m[1][3] * m[3][1] - 
              m[1][0]  * m[0][1] * m[3][3] + 
              m[1][0]  * m[0][3] * m[3][1] + 
              m[3][0] * m[0][1] * m[1][3] - 
              m[3][0] * m[0][3] * m[1][1];

    inv[14] = -m[0][0]  * m[1][1] * m[3][2] + 
               m[0][0]  * m[1][2] * m[3][1] + 
               m[1][0]  * m[0][1] * m[3][2] - 
               m[1][0]  * m[0][2] * m[3][1] - 
               m[3][0] * m[0][1] * m[1][2] + 
               m[3][0] * m[0][2] * m[1][1];

    inv[3] = -m[0][1] * m[1][2] * m[2][3] + 
              m[0][1] * m[1][3] * m[2][2] + 
              m[1][1] * m[0][2] * m[2][3] - 
              m[1][1] * m[0][3] * m[2][2] - 
              m[2][1] * m[0][2] * m[1][3] + 
              m[2][1] * m[0][3] * m[1][2];

    inv[7] = m[0][0] * m[1][2] * m[2][3] - 
             m[0][0] * m[1][3] * m[2][2] - 
             m[1][0] * m[0][2] * m[2][3] + 
             m[1][0] * m[0][3] * m[2][2] + 
             m[2][0] * m[0][2] * m[1][3] - 
             m[2][0] * m[0][3] * m[1][2];

    inv[11] = -m[0][0] * m[1][1] * m[2][3] + 
               m[0][0] * m[1][3] * m[2][1] + 
               m[1][0] * m[0][1] * m[2][3] - 
               m[1][0] * m[0][3] * m[2][1] - 
               m[2][0] * m[0][1] * m[1][3] + 
               m[2][0] * m[0][3] * m[1][1];

    inv[15] = m[0][0] * m[1][1] * m[2][2] - 
              m[0][0] * m[1][2] * m[2][1] - 
              m[1][0] * m[0][1] * m[2][2] + 
              m[1][0] * m[0][2] * m[2][1] + 
              m[2][0] * m[0][1] * m[1][2] - 
              m[2][0] * m[0][2] * m[1][1];

    det = m[0][0] * inv[0] + m[0][1] * inv[4] + m[0][2] * inv[8] + m[0][3] * inv[12];

    if (det == 0)
        return 0;

    det = 1.0 / det;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
        invOut[i][j] = inv[i] * det;
        }
    }

    return 1;
}



void multiplication(float A[4][4], float B[4][4], float result[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void state_matrix(float x, float y, float heading, float C[4][4]) {
    C[0][0] = cos(deg_to_rad(heading));
    C[0][1] = -sin(deg_to_rad(heading));
    C[0][2] = 0.0;
    C[0][3] = x;

    C[1][0] = sin(deg_to_rad(heading));
    C[1][1] = cos(deg_to_rad(heading));
    C[1][2] = 0.0;
    C[1][3] = y;

    C[2][0] = 0.0;
    C[2][1] = 0.0;
    C[2][2] = 1;
    C[2][3] = 0;

    C[3][0] = 0.0;
    C[3][1] = 0.0;
    C[3][2] = 0.0;
    C[3][3] = 1;
}

void printMatrix(float matrix[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f\t", matrix[i][j]);
        }
        printf("\n");
    }
}