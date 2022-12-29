#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void ballbot_dynamics_flow_map_forward_zero(double const *const * in,
                                            double*const * out,
                                            struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[28];

   v[0] = sin(x[3]);
   v[1] = 0.5 * v[0];
   v[2] = 3.75 * v[1];
   v[3] = cos(x[3]);
   v[4] = -0.5 * v[3];
   v[5] = 0 - 1.72532384100864 * v[4];
   v[6] = 0.4375 + v[2] * v[1] - v[5] * v[4];
   v[7] = x[4] + x[5];
   v[8] = v[7] + x[6];
   v[9] = cos(x[2]);
   v[10] = 0.5 * v[9];
   v[11] = v[10] * x[4];
   v[12] = 0.5 * sin(x[2]);
   v[13] = v[12] * x[4];
   v[14] = v[0] * v[11] + v[3] * v[13] + v[1] * v[7];
   v[15] = v[8] * v[14];
   v[16] = 1.12500004470348 * v[15];
   v[17] = x[9] - v[16];
   v[18] = 0.5 * v[3];
   v[14] = (0 - v[14]) * x[6];
   v[15] = 1.72532384100864 * v[14] + -3.75 * (0 - v[15]) + (1.12500004470348 * v[17]) / 0.625100016593933;
   v[19] = 0 - v[0];
   v[20] = v[3] * v[11] + v[19] * v[13] + v[18] * v[7];
   v[20] = 3.75 * (0 - v[8] * v[20]) + 3.75 * v[20] * x[6] - 1.12500004470348 * v[8] * v[8];
   v[8] = v[7] * v[13];
   v[16] = (0.625100016593933 * v[17]) / 0.625100016593933 + v[16] + v[18] * v[15] + v[1] * v[20] + 0.75 * v[8];
   v[21] = x[8] - v[16];
   v[22] = -2.02467615899136 * v[3];
   v[23] = v[22] * v[3];
   v[4] = -1.12500004470348 + 3.75 * v[4];
   v[5] = v[5] + v[4];
   v[1] = 0 - 3.75 * v[1];
   v[2] = v[2] + v[1];
   v[24] = 0 - v[0];
   v[4] = 0 - v[4];
   v[25] = 0.75 + v[5] * v[3] + v[2] * v[0] + v[24] * v[1] + v[3] * v[4];
   v[26] = v[25] / v[6];
   v[27] = 6.75 + v[23] - v[25] * v[26];
   v[13] = (0 - v[13]) * x[5];
   v[4] = v[5] * v[24] + v[2] * v[3] - v[3] * v[1] - v[0] * v[4];
   v[26] = v[22] * v[24] - v[4] * v[26];
   v[24] = v[11] * x[5];
   v[23] = 4.72532384100864 - v[23] - v[4] * v[4] / v[6];
   v[9] = -0.5 * v[9];
   y[3] = (x[7] - (v[6] * v[21]) / v[6] - v[16] - v[10] * (v[3] * v[15] + v[0] * v[20] + -3. * (0 - v[8]) + v[27] * v[13] + v[26] * v[24] + (v[25] * v[21]) / v[6]) - v[12] * (3. * (0 - v[7] * v[11]) + v[19] * v[15] + v[3] * v[20] + v[26] * v[13] + v[23] * v[24] + (v[4] * v[21]) / v[6] - 0.75 * v[7] * v[7])) / (0.4375 + (v[12] * v[23] - v[9] * v[26]) * v[12] - (v[12] * v[26] - v[9] * v[27]) * v[9]);
   v[13] = v[10] * y[3] + v[13];
   v[24] = v[12] * y[3] + v[24];
   y[4] = (v[21] - v[25] * v[13] - v[4] * v[24] - v[6] * y[3]) / v[6];
   v[4] = y[3] + y[4];
   y[5] = (v[17] - 1.12500004470348 * (v[3] * v[13] + v[19] * v[24] + v[18] * v[4] + v[14]) - 0.625100016593933 * v[4]) / 0.625100016593933;
   // dependent variables without operations
   y[0] = x[4];
   y[1] = x[5];
   y[2] = x[6];
}

