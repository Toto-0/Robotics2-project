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

void ballbot_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                               double*const * out,
                                               struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[85];

   v[0] = 0.5 * sin(x[2]);
   v[1] = cos(x[3]);
   v[2] = -2.02467615899136 * v[1];
   v[3] = v[2] * v[1];
   v[4] = -0.5 * v[1];
   v[5] = 0 - 1.72532384100864 * v[4];
   v[6] = -1.12500004470348 + 3.75 * v[4];
   v[7] = v[5] + v[6];
   v[8] = sin(x[3]);
   v[9] = 0 - v[8];
   v[10] = 0.5 * v[8];
   v[11] = 3.75 * v[10];
   v[12] = 0 - 3.75 * v[10];
   v[13] = v[11] + v[12];
   v[6] = 0 - v[6];
   v[14] = v[7] * v[9] + v[13] * v[1] - v[1] * v[12] - v[8] * v[6];
   v[15] = 0.4375 + v[11] * v[10] - v[5] * v[4];
   v[16] = v[14] / v[15];
   v[17] = 4.72532384100864 - v[3] - v[14] * v[16];
   v[18] = cos(x[2]);
   v[19] = -0.5 * v[18];
   v[20] = 0.75 + v[7] * v[1] + v[13] * v[8] + v[9] * v[12] + v[1] * v[6];
   v[21] = v[20] / v[15];
   v[22] = v[2] * v[9] - v[14] * v[21];
   v[23] = v[0] * v[17] - v[19] * v[22];
   v[3] = 6.75 + v[3] - v[20] * v[21];
   v[24] = v[0] * v[22] - v[19] * v[3];
   v[25] = 0.4375 + v[23] * v[0] - v[24] * v[19];
   jac[8] = 1 / v[25];
   v[26] = x[4] + x[5];
   v[27] = v[26] + x[6];
   v[18] = 0.5 * v[18];
   v[28] = v[18] * x[4];
   v[29] = v[0] * x[4];
   v[30] = v[8] * v[28] + v[1] * v[29] + v[10] * v[26];
   v[31] = v[27] * v[30];
   v[32] = 1.12500004470348 * v[31];
   v[33] = x[9] - v[32];
   v[34] = 0.5 * v[1];
   v[35] = 0 - v[30];
   v[31] = 1.72532384100864 * v[35] * x[6] + -3.75 * (0 - v[31]) + (1.12500004470348 * v[33]) / 0.625100016593933;
   v[36] = 0 - v[8];
   v[37] = v[1] * v[28] + v[36] * v[29] + v[34] * v[26];
   v[38] = 3.75 * (0 - v[27] * v[37]) + 3.75 * v[37] * x[6] - 1.12500004470348 * v[27] * v[27];
   v[39] = v[26] * v[29];
   v[33] = (0.625100016593933 * v[33]) / 0.625100016593933 + v[32] + v[34] * v[31] + v[10] * v[38] + 0.75 * v[39];
   v[32] = x[8] - v[33];
   v[40] = (v[15] * v[32]) / v[15];
   v[41] = 0 - v[29];
   v[42] = v[41] * x[5];
   v[43] = v[28] * x[5];
   v[44] = (v[20] * v[32]) / v[15];
   v[39] = v[1] * v[31] + v[8] * v[38] + -3. * (0 - v[39]) + v[3] * v[42] + v[22] * v[43] + v[44];
   v[45] = (v[14] * v[32]) / v[15];
   v[46] = 3. * (0 - v[26] * v[28]) + v[36] * v[31] + v[1] * v[38] + v[22] * v[42] + v[17] * v[43] + v[45] - 0.75 * v[26] * v[26];
   v[33] = (x[7] - v[40] - v[33] - v[18] * v[39] - v[0] * v[46]) / v[25];
   v[47] = 0 - jac[8] * v[33];
   v[48] = 0 - v[47];
   v[49] = v[48] * v[19];
   v[50] = 0 - v[49];
   v[51] = v[47] * v[0];
   v[52] = 0 - v[51];
   v[53] = 0 - jac[8];
   v[54] = v[53] * v[18];
   v[55] = 0 - jac[8];
   v[56] = v[55] * v[0];
   v[57] = v[54] * 1 / v[15];
   v[58] = v[56] * 1 / v[15];
   v[59] = (0 - jac[8]) * 1 / v[15];
   jac[9] = v[57] * v[20] + v[58] * v[14] + v[59] * v[15];
   v[60] = 0 - jac[8] - jac[9];
   v[61] = v[54] * v[8] + v[56] * v[1] + v[60] * v[10];
   v[62] = 0 - v[61] * 3.75;
   v[63] = v[61] * 3.75;
   v[64] = v[62] * v[27] + v[63] * x[6];
   v[65] = v[54] * v[1] + v[56] * v[36] + v[60] * v[34];
   jac[10] = v[65] * 1.59974399848657 * 1.12500004470348 + v[60] * 1.59974399848657 * 0.625100016593933;
   v[66] = 0 - v[65] * -3.75 + (v[60] - jac[10]) * 1.12500004470348;
   v[65] = v[65] * 1.72532384100864;
   v[67] = v[66] * v[27] - v[65] * x[6];
   v[68] = 0 - v[56] * 3.;
   v[69] = v[54] * v[22] + v[56] * v[17];
   v[70] = v[64] * v[1] + v[67] * v[8] + v[68] * v[26] + v[69] * x[5];
   v[71] = sin(x[2]);
   v[72] = 0 - v[54] * -3. + v[60] * 0.75;
   v[73] = v[54] * v[3] + v[56] * v[22];
   v[74] = v[64] * v[36] + v[67] * v[1] + v[72] * v[26] - v[73] * x[5];
   v[75] = cos(x[2]);
   jac[3] = 0 - ((v[48] * v[24] + v[50] * v[3] + v[52] * v[22]) * -0.5 + (v[53] * v[39] + v[70] * x[4]) * 0.5) * v[71] + (v[55] * v[46] + v[47] * v[23] + v[49] * v[22] + v[51] * v[17] + v[74] * x[4]) * 0.5 * v[75];
   v[50] = v[50] * v[19] + v[54] * v[42];
   v[51] = v[51] * v[0] + v[56] * v[43];
   v[55] = v[50] - v[51];
   v[52] = v[49] * v[0] + v[52] * v[19] + v[54] * v[43] + v[56] * v[42];
   v[50] = 0 - v[50];
   v[49] = 0 - v[52];
   v[53] = (v[50] * v[20] + v[49] * v[14]) * 1 / v[15];
   v[50] = v[57] * v[32] + v[50] * v[21] + v[53];
   v[51] = 0 - v[51];
   v[48] = v[51] * v[14] * 1 / v[15];
   v[51] = v[58] * v[32] + v[49] * v[21] + v[51] * v[16] + v[48];
   v[49] = 0 - v[51];
   v[48] = 0 - v[57] * v[44] - v[58] * v[45] - v[59] * v[40] + v[59] * v[32] - v[53] * v[21] - v[48] * v[16];
   v[53] = 0 - v[48];
   v[59] = v[50] * v[1] + v[51] * v[9];
   v[58] = 0 - v[51];
   v[57] = sin(x[3]);
   v[47] = v[50] * v[8] + v[51] * v[1];
   v[76] = cos(x[3]);
   jac[4] = 0 - (v[54] * v[31] + v[56] * v[38] + v[55] * v[2] + (v[55] * v[1] + v[52] * v[9]) * -2.02467615899136 + v[50] * v[7] + v[51] * v[13] + v[49] * v[12] + v[50] * v[6] + (v[53] * v[5] + (0 - (v[59] + v[53] * v[4])) * 1.72532384100864 + (0 - (v[58] * v[8] + v[50] * v[1]) + v[59]) * 3.75) * -0.5 + v[64] * v[28] + v[67] * v[29] + (v[60] * v[31] + v[64] * v[26]) * 0.5) * v[57] + (v[54] * v[38] + v[50] * v[13] + v[58] * v[6] + v[67] * v[28] - (v[56] * v[31] + v[64] * v[29]) + (v[60] * v[38] + v[48] * v[11] + (v[47] + v[48] * v[10]) * 3.75 + (0 - (v[49] * v[1] + v[50] * v[9] + v[47])) * 3.75 + v[67] * v[26]) * 0.5 - (v[52] * v[2] + v[51] * v[7] + v[50] * v[12])) * v[76];
   v[61] = (0 - v[61]) * 1.12500004470348;
   v[61] = v[62] * v[37] + v[66] * v[30] + v[61] * v[27] + v[61] * v[27];
   v[56] = (0 - v[56]) * 0.75;
   v[56] = v[61] + v[64] * v[34] + v[67] * v[10] + v[68] * v[28] + v[72] * v[29] + v[56] * v[26] + v[56] * v[26];
   jac[5] = v[56] + v[70] * v[18] + v[74] * v[0];
   jac[6] = v[73] * v[41] + v[69] * v[28] + v[56];
   jac[7] = v[65] * v[35] + v[63] * v[37] + v[61];
   v[61] = 1 / v[15];
   v[65] = 0 - v[61];
   v[63] = 0 - v[61];
   v[56] = v[63] * v[20];
   v[73] = 0 - v[61];
   v[69] = v[73] * v[14];
   jac[16] = (v[65] * v[15] + v[56] * v[18] + v[69] * v[0]) * 1 / v[25];
   v[74] = 0 - jac[16] * v[33];
   v[70] = 0 - v[74];
   v[72] = v[70] * v[19];
   v[68] = 0 - v[72];
   v[67] = v[74] * v[0];
   v[64] = 0 - v[67];
   v[66] = 0 - jac[16];
   v[62] = v[66] * v[18];
   v[47] = 0 - jac[16];
   v[58] = v[47] * v[0];
   v[59] = v[62] * 1 / v[15];
   v[53] = v[58] * 1 / v[15];
   v[48] = (0 - jac[16]) * 1 / v[15];
   jac[17] = v[61] + v[59] * v[20] + v[53] * v[14] + v[48] * v[15];
   v[49] = 0 - jac[16] - jac[17];
   v[51] = v[62] * v[8] + v[58] * v[1] + v[49] * v[10];
   v[50] = 0 - v[51] * 3.75;
   v[52] = v[51] * 3.75;
   v[55] = v[50] * v[27] + v[52] * x[6];
   v[60] = v[62] * v[1] + v[58] * v[36] + v[49] * v[34];
   jac[18] = v[60] * 1.59974399848657 * 1.12500004470348 + v[49] * 1.59974399848657 * 0.625100016593933;
   v[54] = 0 - v[60] * -3.75 + (v[49] - jac[18]) * 1.12500004470348;
   v[60] = v[60] * 1.72532384100864;
   v[77] = v[54] * v[27] - v[60] * x[6];
   v[78] = 0 - v[58] * 3.;
   v[79] = v[69] + v[62] * v[22] + v[58] * v[17];
   v[80] = v[55] * v[1] + v[77] * v[8] + v[78] * v[26] + v[79] * x[5];
   v[81] = 0 - v[62] * -3. + v[49] * 0.75;
   v[82] = v[56] + v[62] * v[3] + v[58] * v[22];
   v[83] = v[55] * v[36] + v[77] * v[1] + v[81] * v[26] - v[82] * x[5];
   jac[11] = 0 - ((v[70] * v[24] + v[68] * v[3] + v[64] * v[22]) * -0.5 + (v[56] * v[33] + v[66] * v[39] + v[80] * x[4]) * 0.5) * v[71] + (v[69] * v[33] + v[47] * v[46] + v[74] * v[23] + v[72] * v[22] + v[67] * v[17] + v[83] * x[4]) * 0.5 * v[75];
   v[68] = v[68] * v[19] + v[62] * v[42];
   v[67] = v[67] * v[0] + v[58] * v[43];
   v[47] = v[68] - v[67];
   v[64] = v[72] * v[0] + v[64] * v[19] + v[62] * v[43] + v[58] * v[42];
   v[72] = v[18] * v[33] + v[42];
   v[68] = 0 - v[68];
   v[66] = 0 - v[64];
   v[70] = (v[68] * v[20] + v[66] * v[14]) * 1 / v[15];
   v[68] = v[63] * v[72] + v[59] * v[32] + v[68] * v[21] + v[70];
   v[63] = v[0] * v[33] + v[43];
   v[67] = 0 - v[67];
   v[74] = v[67] * v[14] * 1 / v[15];
   v[67] = v[73] * v[63] + v[53] * v[32] + v[66] * v[21] + v[67] * v[16] + v[74];
   v[66] = 0 - v[67];
   v[73] = (v[32] - v[20] * v[72] - v[14] * v[63] - v[15] * v[33]) / v[15];
   v[74] = 0 - v[61] * v[73] + v[65] * v[33] - v[59] * v[44] - v[53] * v[45] - v[48] * v[40] + v[48] * v[32] - v[70] * v[21] - v[74] * v[16];
   v[70] = 0 - v[74];
   v[48] = v[68] * v[1] + v[67] * v[9];
   v[53] = 0 - v[67];
   v[59] = v[68] * v[8] + v[67] * v[1];
   jac[12] = 0 - (v[62] * v[31] + v[58] * v[38] + v[47] * v[2] + (v[47] * v[1] + v[64] * v[9]) * -2.02467615899136 + v[68] * v[7] + v[67] * v[13] + v[66] * v[12] + v[68] * v[6] + (v[70] * v[5] + (0 - (v[48] + v[70] * v[4])) * 1.72532384100864 + (0 - (v[53] * v[8] + v[68] * v[1]) + v[48]) * 3.75) * -0.5 + v[55] * v[28] + v[77] * v[29] + (v[49] * v[31] + v[55] * v[26]) * 0.5) * v[57] + (v[62] * v[38] + v[68] * v[13] + v[53] * v[6] + v[77] * v[28] - (v[58] * v[31] + v[55] * v[29]) + (v[49] * v[38] + v[74] * v[11] + (v[59] + v[74] * v[10]) * 3.75 + (0 - (v[66] * v[1] + v[68] * v[9] + v[59])) * 3.75 + v[77] * v[26]) * 0.5 - (v[64] * v[2] + v[67] * v[7] + v[68] * v[12])) * v[76];
   v[51] = (0 - v[51]) * 1.12500004470348;
   v[51] = v[50] * v[37] + v[54] * v[30] + v[51] * v[27] + v[51] * v[27];
   v[58] = (0 - v[58]) * 0.75;
   v[58] = v[51] + v[55] * v[34] + v[77] * v[10] + v[78] * v[28] + v[81] * v[29] + v[58] * v[26] + v[58] * v[26];
   jac[13] = v[58] + v[80] * v[18] + v[83] * v[0];
   jac[14] = v[82] * v[41] + v[79] * v[28] + v[58];
   jac[15] = v[60] * v[35] + v[52] * v[37] + v[51];
   v[51] = -1 + -1.79971206981152 * v[34];
   v[60] = v[51] * 1 / v[15];
   v[52] = 0 - v[60];
   v[58] = 0 - v[60];
   v[82] = -1.79971206981152 * v[1] + v[58] * v[20];
   v[79] = 0 - v[60];
   v[83] = -1.79971206981152 * v[36] + v[79] * v[14];
   jac[24] = (v[51] + v[52] * v[15] + v[82] * v[18] + v[83] * v[0]) * 1 / v[25];
   v[51] = 0 - jac[24] * v[33];
   v[25] = 0 - v[51];
   v[80] = v[25] * v[19];
   v[81] = 0 - v[80];
   v[78] = v[51] * v[0];
   v[77] = 0 - v[78];
   v[55] = 0 - jac[24];
   v[54] = v[55] * v[18];
   v[50] = 0 - jac[24];
   v[59] = v[50] * v[0];
   v[53] = v[54] * 1 / v[15];
   v[48] = v[59] * 1 / v[15];
   v[70] = (0 - jac[24]) * 1 / v[15];
   jac[25] = v[60] + v[53] * v[20] + v[48] * v[14] + v[70] * v[15];
   v[74] = 0 - jac[24] - jac[25];
   v[66] = v[54] * v[8] + v[59] * v[1] + v[74] * v[10];
   v[67] = 0 - v[66] * 3.75;
   v[68] = v[66] * 3.75;
   v[64] = v[67] * v[27] + v[68] * x[6];
   v[47] = v[54] * v[1] + v[59] * v[36] + v[74] * v[34];
   jac[26] = 1.59974399848657 + v[47] * 1.59974399848657 * 1.12500004470348 + v[74] * 1.59974399848657 * 0.625100016593933;
   v[49] = 0 - v[47] * -3.75 + (v[74] - jac[26]) * 1.12500004470348;
   v[47] = -1.79971206981152 + v[47] * 1.72532384100864;
   v[62] = v[49] * v[27] - v[47] * x[6];
   v[65] = 0 - v[59] * 3.;
   v[61] = v[83] + v[54] * v[22] + v[59] * v[17];
   v[69] = v[64] * v[1] + v[62] * v[8] + v[65] * v[26] + v[61] * x[5];
   v[56] = 0 - v[54] * -3. + v[74] * 0.75;
   v[84] = v[82] + v[54] * v[3] + v[59] * v[22];
   v[36] = v[64] * v[36] + v[62] * v[1] + v[56] * v[26] - v[84] * x[5];
   jac[19] = 0 - ((v[25] * v[24] + v[81] * v[3] + v[77] * v[22]) * -0.5 + (v[82] * v[33] + v[55] * v[39] + v[69] * x[4]) * 0.5) * v[71] + (v[83] * v[33] + v[50] * v[46] + v[51] * v[23] + v[80] * v[22] + v[78] * v[17] + v[36] * x[4]) * 0.5 * v[75];
   v[81] = v[81] * v[19] + v[54] * v[42];
   v[78] = v[78] * v[0] + v[59] * v[43];
   v[50] = v[81] - v[78];
   v[77] = v[80] * v[0] + v[77] * v[19] + v[54] * v[43] + v[59] * v[42];
   v[81] = 0 - v[81];
   v[80] = 0 - v[77];
   v[20] = (v[81] * v[20] + v[80] * v[14]) * 1 / v[15];
   v[81] = v[58] * v[72] + v[53] * v[32] + v[81] * v[21] + v[20];
   v[78] = 0 - v[78];
   v[15] = v[78] * v[14] * 1 / v[15];
   v[78] = v[79] * v[63] + v[48] * v[32] + v[80] * v[21] + v[78] * v[16] + v[15];
   v[80] = 0 - v[78];
   v[15] = 0 - v[60] * v[73] + v[52] * v[33] - v[53] * v[44] - v[48] * v[45] - v[70] * v[40] + v[70] * v[32] - v[20] * v[21] - v[15] * v[16];
   v[20] = 0 - v[15];
   v[70] = v[81] * v[1] + v[78] * v[9];
   v[48] = 0 - v[78];
   v[53] = v[81] * v[8] + v[78] * v[1];
   jac[20] = 0 - (-1.79971206981152 * v[72] + v[54] * v[31] + v[59] * v[38] + v[50] * v[2] + (v[50] * v[1] + v[77] * v[9]) * -2.02467615899136 + v[81] * v[7] + v[78] * v[13] + v[80] * v[12] + v[81] * v[6] + (v[20] * v[5] + (0 - (v[70] + v[20] * v[4])) * 1.72532384100864 + (0 - (v[48] * v[8] + v[81] * v[1]) + v[70]) * 3.75) * -0.5 + v[64] * v[28] + v[62] * v[29] + (-1.79971206981152 * (v[33] + v[73]) + v[74] * v[31] + v[64] * v[26]) * 0.5) * v[57] + (v[54] * v[38] + v[81] * v[13] + v[48] * v[6] + v[62] * v[28] - (-1.79971206981152 * v[63] + v[59] * v[31] + v[64] * v[29]) + (v[74] * v[38] + v[15] * v[11] + (v[53] + v[15] * v[10]) * 3.75 + (0 - (v[80] * v[1] + v[81] * v[9] + v[53])) * 3.75 + v[62] * v[26]) * 0.5 - (v[77] * v[2] + v[78] * v[7] + v[81] * v[12])) * v[76];
   v[66] = (0 - v[66]) * 1.12500004470348;
   v[66] = v[67] * v[37] + v[49] * v[30] + v[66] * v[27] + v[66] * v[27];
   v[59] = (0 - v[59]) * 0.75;
   v[59] = v[66] + v[64] * v[34] + v[62] * v[10] + v[65] * v[28] + v[56] * v[29] + v[59] * v[26] + v[59] * v[26];
   jac[21] = v[59] + v[69] * v[18] + v[36] * v[0];
   jac[22] = v[84] * v[41] + v[61] * v[28] + v[59];
   jac[23] = v[47] * v[35] + v[68] * v[37] + v[66];
   // dependent variables without operations
   jac[0] = 1;
   jac[1] = 1;
   jac[2] = 1;
}
