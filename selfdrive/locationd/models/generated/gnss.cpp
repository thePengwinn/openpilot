#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5871428431702092966) {
   out_5871428431702092966[0] = delta_x[0] + nom_x[0];
   out_5871428431702092966[1] = delta_x[1] + nom_x[1];
   out_5871428431702092966[2] = delta_x[2] + nom_x[2];
   out_5871428431702092966[3] = delta_x[3] + nom_x[3];
   out_5871428431702092966[4] = delta_x[4] + nom_x[4];
   out_5871428431702092966[5] = delta_x[5] + nom_x[5];
   out_5871428431702092966[6] = delta_x[6] + nom_x[6];
   out_5871428431702092966[7] = delta_x[7] + nom_x[7];
   out_5871428431702092966[8] = delta_x[8] + nom_x[8];
   out_5871428431702092966[9] = delta_x[9] + nom_x[9];
   out_5871428431702092966[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7767822878109567636) {
   out_7767822878109567636[0] = -nom_x[0] + true_x[0];
   out_7767822878109567636[1] = -nom_x[1] + true_x[1];
   out_7767822878109567636[2] = -nom_x[2] + true_x[2];
   out_7767822878109567636[3] = -nom_x[3] + true_x[3];
   out_7767822878109567636[4] = -nom_x[4] + true_x[4];
   out_7767822878109567636[5] = -nom_x[5] + true_x[5];
   out_7767822878109567636[6] = -nom_x[6] + true_x[6];
   out_7767822878109567636[7] = -nom_x[7] + true_x[7];
   out_7767822878109567636[8] = -nom_x[8] + true_x[8];
   out_7767822878109567636[9] = -nom_x[9] + true_x[9];
   out_7767822878109567636[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2554109935190345649) {
   out_2554109935190345649[0] = 1.0;
   out_2554109935190345649[1] = 0;
   out_2554109935190345649[2] = 0;
   out_2554109935190345649[3] = 0;
   out_2554109935190345649[4] = 0;
   out_2554109935190345649[5] = 0;
   out_2554109935190345649[6] = 0;
   out_2554109935190345649[7] = 0;
   out_2554109935190345649[8] = 0;
   out_2554109935190345649[9] = 0;
   out_2554109935190345649[10] = 0;
   out_2554109935190345649[11] = 0;
   out_2554109935190345649[12] = 1.0;
   out_2554109935190345649[13] = 0;
   out_2554109935190345649[14] = 0;
   out_2554109935190345649[15] = 0;
   out_2554109935190345649[16] = 0;
   out_2554109935190345649[17] = 0;
   out_2554109935190345649[18] = 0;
   out_2554109935190345649[19] = 0;
   out_2554109935190345649[20] = 0;
   out_2554109935190345649[21] = 0;
   out_2554109935190345649[22] = 0;
   out_2554109935190345649[23] = 0;
   out_2554109935190345649[24] = 1.0;
   out_2554109935190345649[25] = 0;
   out_2554109935190345649[26] = 0;
   out_2554109935190345649[27] = 0;
   out_2554109935190345649[28] = 0;
   out_2554109935190345649[29] = 0;
   out_2554109935190345649[30] = 0;
   out_2554109935190345649[31] = 0;
   out_2554109935190345649[32] = 0;
   out_2554109935190345649[33] = 0;
   out_2554109935190345649[34] = 0;
   out_2554109935190345649[35] = 0;
   out_2554109935190345649[36] = 1.0;
   out_2554109935190345649[37] = 0;
   out_2554109935190345649[38] = 0;
   out_2554109935190345649[39] = 0;
   out_2554109935190345649[40] = 0;
   out_2554109935190345649[41] = 0;
   out_2554109935190345649[42] = 0;
   out_2554109935190345649[43] = 0;
   out_2554109935190345649[44] = 0;
   out_2554109935190345649[45] = 0;
   out_2554109935190345649[46] = 0;
   out_2554109935190345649[47] = 0;
   out_2554109935190345649[48] = 1.0;
   out_2554109935190345649[49] = 0;
   out_2554109935190345649[50] = 0;
   out_2554109935190345649[51] = 0;
   out_2554109935190345649[52] = 0;
   out_2554109935190345649[53] = 0;
   out_2554109935190345649[54] = 0;
   out_2554109935190345649[55] = 0;
   out_2554109935190345649[56] = 0;
   out_2554109935190345649[57] = 0;
   out_2554109935190345649[58] = 0;
   out_2554109935190345649[59] = 0;
   out_2554109935190345649[60] = 1.0;
   out_2554109935190345649[61] = 0;
   out_2554109935190345649[62] = 0;
   out_2554109935190345649[63] = 0;
   out_2554109935190345649[64] = 0;
   out_2554109935190345649[65] = 0;
   out_2554109935190345649[66] = 0;
   out_2554109935190345649[67] = 0;
   out_2554109935190345649[68] = 0;
   out_2554109935190345649[69] = 0;
   out_2554109935190345649[70] = 0;
   out_2554109935190345649[71] = 0;
   out_2554109935190345649[72] = 1.0;
   out_2554109935190345649[73] = 0;
   out_2554109935190345649[74] = 0;
   out_2554109935190345649[75] = 0;
   out_2554109935190345649[76] = 0;
   out_2554109935190345649[77] = 0;
   out_2554109935190345649[78] = 0;
   out_2554109935190345649[79] = 0;
   out_2554109935190345649[80] = 0;
   out_2554109935190345649[81] = 0;
   out_2554109935190345649[82] = 0;
   out_2554109935190345649[83] = 0;
   out_2554109935190345649[84] = 1.0;
   out_2554109935190345649[85] = 0;
   out_2554109935190345649[86] = 0;
   out_2554109935190345649[87] = 0;
   out_2554109935190345649[88] = 0;
   out_2554109935190345649[89] = 0;
   out_2554109935190345649[90] = 0;
   out_2554109935190345649[91] = 0;
   out_2554109935190345649[92] = 0;
   out_2554109935190345649[93] = 0;
   out_2554109935190345649[94] = 0;
   out_2554109935190345649[95] = 0;
   out_2554109935190345649[96] = 1.0;
   out_2554109935190345649[97] = 0;
   out_2554109935190345649[98] = 0;
   out_2554109935190345649[99] = 0;
   out_2554109935190345649[100] = 0;
   out_2554109935190345649[101] = 0;
   out_2554109935190345649[102] = 0;
   out_2554109935190345649[103] = 0;
   out_2554109935190345649[104] = 0;
   out_2554109935190345649[105] = 0;
   out_2554109935190345649[106] = 0;
   out_2554109935190345649[107] = 0;
   out_2554109935190345649[108] = 1.0;
   out_2554109935190345649[109] = 0;
   out_2554109935190345649[110] = 0;
   out_2554109935190345649[111] = 0;
   out_2554109935190345649[112] = 0;
   out_2554109935190345649[113] = 0;
   out_2554109935190345649[114] = 0;
   out_2554109935190345649[115] = 0;
   out_2554109935190345649[116] = 0;
   out_2554109935190345649[117] = 0;
   out_2554109935190345649[118] = 0;
   out_2554109935190345649[119] = 0;
   out_2554109935190345649[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2003281458956840132) {
   out_2003281458956840132[0] = dt*state[3] + state[0];
   out_2003281458956840132[1] = dt*state[4] + state[1];
   out_2003281458956840132[2] = dt*state[5] + state[2];
   out_2003281458956840132[3] = state[3];
   out_2003281458956840132[4] = state[4];
   out_2003281458956840132[5] = state[5];
   out_2003281458956840132[6] = dt*state[7] + state[6];
   out_2003281458956840132[7] = dt*state[8] + state[7];
   out_2003281458956840132[8] = state[8];
   out_2003281458956840132[9] = state[9];
   out_2003281458956840132[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6875832990013989469) {
   out_6875832990013989469[0] = 1;
   out_6875832990013989469[1] = 0;
   out_6875832990013989469[2] = 0;
   out_6875832990013989469[3] = dt;
   out_6875832990013989469[4] = 0;
   out_6875832990013989469[5] = 0;
   out_6875832990013989469[6] = 0;
   out_6875832990013989469[7] = 0;
   out_6875832990013989469[8] = 0;
   out_6875832990013989469[9] = 0;
   out_6875832990013989469[10] = 0;
   out_6875832990013989469[11] = 0;
   out_6875832990013989469[12] = 1;
   out_6875832990013989469[13] = 0;
   out_6875832990013989469[14] = 0;
   out_6875832990013989469[15] = dt;
   out_6875832990013989469[16] = 0;
   out_6875832990013989469[17] = 0;
   out_6875832990013989469[18] = 0;
   out_6875832990013989469[19] = 0;
   out_6875832990013989469[20] = 0;
   out_6875832990013989469[21] = 0;
   out_6875832990013989469[22] = 0;
   out_6875832990013989469[23] = 0;
   out_6875832990013989469[24] = 1;
   out_6875832990013989469[25] = 0;
   out_6875832990013989469[26] = 0;
   out_6875832990013989469[27] = dt;
   out_6875832990013989469[28] = 0;
   out_6875832990013989469[29] = 0;
   out_6875832990013989469[30] = 0;
   out_6875832990013989469[31] = 0;
   out_6875832990013989469[32] = 0;
   out_6875832990013989469[33] = 0;
   out_6875832990013989469[34] = 0;
   out_6875832990013989469[35] = 0;
   out_6875832990013989469[36] = 1;
   out_6875832990013989469[37] = 0;
   out_6875832990013989469[38] = 0;
   out_6875832990013989469[39] = 0;
   out_6875832990013989469[40] = 0;
   out_6875832990013989469[41] = 0;
   out_6875832990013989469[42] = 0;
   out_6875832990013989469[43] = 0;
   out_6875832990013989469[44] = 0;
   out_6875832990013989469[45] = 0;
   out_6875832990013989469[46] = 0;
   out_6875832990013989469[47] = 0;
   out_6875832990013989469[48] = 1;
   out_6875832990013989469[49] = 0;
   out_6875832990013989469[50] = 0;
   out_6875832990013989469[51] = 0;
   out_6875832990013989469[52] = 0;
   out_6875832990013989469[53] = 0;
   out_6875832990013989469[54] = 0;
   out_6875832990013989469[55] = 0;
   out_6875832990013989469[56] = 0;
   out_6875832990013989469[57] = 0;
   out_6875832990013989469[58] = 0;
   out_6875832990013989469[59] = 0;
   out_6875832990013989469[60] = 1;
   out_6875832990013989469[61] = 0;
   out_6875832990013989469[62] = 0;
   out_6875832990013989469[63] = 0;
   out_6875832990013989469[64] = 0;
   out_6875832990013989469[65] = 0;
   out_6875832990013989469[66] = 0;
   out_6875832990013989469[67] = 0;
   out_6875832990013989469[68] = 0;
   out_6875832990013989469[69] = 0;
   out_6875832990013989469[70] = 0;
   out_6875832990013989469[71] = 0;
   out_6875832990013989469[72] = 1;
   out_6875832990013989469[73] = dt;
   out_6875832990013989469[74] = 0;
   out_6875832990013989469[75] = 0;
   out_6875832990013989469[76] = 0;
   out_6875832990013989469[77] = 0;
   out_6875832990013989469[78] = 0;
   out_6875832990013989469[79] = 0;
   out_6875832990013989469[80] = 0;
   out_6875832990013989469[81] = 0;
   out_6875832990013989469[82] = 0;
   out_6875832990013989469[83] = 0;
   out_6875832990013989469[84] = 1;
   out_6875832990013989469[85] = dt;
   out_6875832990013989469[86] = 0;
   out_6875832990013989469[87] = 0;
   out_6875832990013989469[88] = 0;
   out_6875832990013989469[89] = 0;
   out_6875832990013989469[90] = 0;
   out_6875832990013989469[91] = 0;
   out_6875832990013989469[92] = 0;
   out_6875832990013989469[93] = 0;
   out_6875832990013989469[94] = 0;
   out_6875832990013989469[95] = 0;
   out_6875832990013989469[96] = 1;
   out_6875832990013989469[97] = 0;
   out_6875832990013989469[98] = 0;
   out_6875832990013989469[99] = 0;
   out_6875832990013989469[100] = 0;
   out_6875832990013989469[101] = 0;
   out_6875832990013989469[102] = 0;
   out_6875832990013989469[103] = 0;
   out_6875832990013989469[104] = 0;
   out_6875832990013989469[105] = 0;
   out_6875832990013989469[106] = 0;
   out_6875832990013989469[107] = 0;
   out_6875832990013989469[108] = 1;
   out_6875832990013989469[109] = 0;
   out_6875832990013989469[110] = 0;
   out_6875832990013989469[111] = 0;
   out_6875832990013989469[112] = 0;
   out_6875832990013989469[113] = 0;
   out_6875832990013989469[114] = 0;
   out_6875832990013989469[115] = 0;
   out_6875832990013989469[116] = 0;
   out_6875832990013989469[117] = 0;
   out_6875832990013989469[118] = 0;
   out_6875832990013989469[119] = 0;
   out_6875832990013989469[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6660006016391780320) {
   out_6660006016391780320[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_651387725702215192) {
   out_651387725702215192[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_651387725702215192[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_651387725702215192[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_651387725702215192[3] = 0;
   out_651387725702215192[4] = 0;
   out_651387725702215192[5] = 0;
   out_651387725702215192[6] = 1;
   out_651387725702215192[7] = 0;
   out_651387725702215192[8] = 0;
   out_651387725702215192[9] = 0;
   out_651387725702215192[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4892345393658224162) {
   out_4892345393658224162[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8576617868203100303) {
   out_8576617868203100303[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8576617868203100303[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8576617868203100303[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8576617868203100303[3] = 0;
   out_8576617868203100303[4] = 0;
   out_8576617868203100303[5] = 0;
   out_8576617868203100303[6] = 1;
   out_8576617868203100303[7] = 0;
   out_8576617868203100303[8] = 0;
   out_8576617868203100303[9] = 1;
   out_8576617868203100303[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4352390767440585904) {
   out_4352390767440585904[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_5527866029776887273) {
   out_5527866029776887273[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[6] = 0;
   out_5527866029776887273[7] = 1;
   out_5527866029776887273[8] = 0;
   out_5527866029776887273[9] = 0;
   out_5527866029776887273[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4352390767440585904) {
   out_4352390767440585904[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_5527866029776887273) {
   out_5527866029776887273[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5527866029776887273[6] = 0;
   out_5527866029776887273[7] = 1;
   out_5527866029776887273[8] = 0;
   out_5527866029776887273[9] = 0;
   out_5527866029776887273[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5871428431702092966) {
  err_fun(nom_x, delta_x, out_5871428431702092966);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7767822878109567636) {
  inv_err_fun(nom_x, true_x, out_7767822878109567636);
}
void gnss_H_mod_fun(double *state, double *out_2554109935190345649) {
  H_mod_fun(state, out_2554109935190345649);
}
void gnss_f_fun(double *state, double dt, double *out_2003281458956840132) {
  f_fun(state,  dt, out_2003281458956840132);
}
void gnss_F_fun(double *state, double dt, double *out_6875832990013989469) {
  F_fun(state,  dt, out_6875832990013989469);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6660006016391780320) {
  h_6(state, sat_pos, out_6660006016391780320);
}
void gnss_H_6(double *state, double *sat_pos, double *out_651387725702215192) {
  H_6(state, sat_pos, out_651387725702215192);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4892345393658224162) {
  h_20(state, sat_pos, out_4892345393658224162);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8576617868203100303) {
  H_20(state, sat_pos, out_8576617868203100303);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4352390767440585904) {
  h_7(state, sat_pos_vel, out_4352390767440585904);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5527866029776887273) {
  H_7(state, sat_pos_vel, out_5527866029776887273);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4352390767440585904) {
  h_21(state, sat_pos_vel, out_4352390767440585904);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5527866029776887273) {
  H_21(state, sat_pos_vel, out_5527866029776887273);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
