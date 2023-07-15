#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7197372880823570987) {
   out_7197372880823570987[0] = delta_x[0] + nom_x[0];
   out_7197372880823570987[1] = delta_x[1] + nom_x[1];
   out_7197372880823570987[2] = delta_x[2] + nom_x[2];
   out_7197372880823570987[3] = delta_x[3] + nom_x[3];
   out_7197372880823570987[4] = delta_x[4] + nom_x[4];
   out_7197372880823570987[5] = delta_x[5] + nom_x[5];
   out_7197372880823570987[6] = delta_x[6] + nom_x[6];
   out_7197372880823570987[7] = delta_x[7] + nom_x[7];
   out_7197372880823570987[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8538761970312805771) {
   out_8538761970312805771[0] = -nom_x[0] + true_x[0];
   out_8538761970312805771[1] = -nom_x[1] + true_x[1];
   out_8538761970312805771[2] = -nom_x[2] + true_x[2];
   out_8538761970312805771[3] = -nom_x[3] + true_x[3];
   out_8538761970312805771[4] = -nom_x[4] + true_x[4];
   out_8538761970312805771[5] = -nom_x[5] + true_x[5];
   out_8538761970312805771[6] = -nom_x[6] + true_x[6];
   out_8538761970312805771[7] = -nom_x[7] + true_x[7];
   out_8538761970312805771[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5606927004741127484) {
   out_5606927004741127484[0] = 1.0;
   out_5606927004741127484[1] = 0;
   out_5606927004741127484[2] = 0;
   out_5606927004741127484[3] = 0;
   out_5606927004741127484[4] = 0;
   out_5606927004741127484[5] = 0;
   out_5606927004741127484[6] = 0;
   out_5606927004741127484[7] = 0;
   out_5606927004741127484[8] = 0;
   out_5606927004741127484[9] = 0;
   out_5606927004741127484[10] = 1.0;
   out_5606927004741127484[11] = 0;
   out_5606927004741127484[12] = 0;
   out_5606927004741127484[13] = 0;
   out_5606927004741127484[14] = 0;
   out_5606927004741127484[15] = 0;
   out_5606927004741127484[16] = 0;
   out_5606927004741127484[17] = 0;
   out_5606927004741127484[18] = 0;
   out_5606927004741127484[19] = 0;
   out_5606927004741127484[20] = 1.0;
   out_5606927004741127484[21] = 0;
   out_5606927004741127484[22] = 0;
   out_5606927004741127484[23] = 0;
   out_5606927004741127484[24] = 0;
   out_5606927004741127484[25] = 0;
   out_5606927004741127484[26] = 0;
   out_5606927004741127484[27] = 0;
   out_5606927004741127484[28] = 0;
   out_5606927004741127484[29] = 0;
   out_5606927004741127484[30] = 1.0;
   out_5606927004741127484[31] = 0;
   out_5606927004741127484[32] = 0;
   out_5606927004741127484[33] = 0;
   out_5606927004741127484[34] = 0;
   out_5606927004741127484[35] = 0;
   out_5606927004741127484[36] = 0;
   out_5606927004741127484[37] = 0;
   out_5606927004741127484[38] = 0;
   out_5606927004741127484[39] = 0;
   out_5606927004741127484[40] = 1.0;
   out_5606927004741127484[41] = 0;
   out_5606927004741127484[42] = 0;
   out_5606927004741127484[43] = 0;
   out_5606927004741127484[44] = 0;
   out_5606927004741127484[45] = 0;
   out_5606927004741127484[46] = 0;
   out_5606927004741127484[47] = 0;
   out_5606927004741127484[48] = 0;
   out_5606927004741127484[49] = 0;
   out_5606927004741127484[50] = 1.0;
   out_5606927004741127484[51] = 0;
   out_5606927004741127484[52] = 0;
   out_5606927004741127484[53] = 0;
   out_5606927004741127484[54] = 0;
   out_5606927004741127484[55] = 0;
   out_5606927004741127484[56] = 0;
   out_5606927004741127484[57] = 0;
   out_5606927004741127484[58] = 0;
   out_5606927004741127484[59] = 0;
   out_5606927004741127484[60] = 1.0;
   out_5606927004741127484[61] = 0;
   out_5606927004741127484[62] = 0;
   out_5606927004741127484[63] = 0;
   out_5606927004741127484[64] = 0;
   out_5606927004741127484[65] = 0;
   out_5606927004741127484[66] = 0;
   out_5606927004741127484[67] = 0;
   out_5606927004741127484[68] = 0;
   out_5606927004741127484[69] = 0;
   out_5606927004741127484[70] = 1.0;
   out_5606927004741127484[71] = 0;
   out_5606927004741127484[72] = 0;
   out_5606927004741127484[73] = 0;
   out_5606927004741127484[74] = 0;
   out_5606927004741127484[75] = 0;
   out_5606927004741127484[76] = 0;
   out_5606927004741127484[77] = 0;
   out_5606927004741127484[78] = 0;
   out_5606927004741127484[79] = 0;
   out_5606927004741127484[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_799691487437747399) {
   out_799691487437747399[0] = state[0];
   out_799691487437747399[1] = state[1];
   out_799691487437747399[2] = state[2];
   out_799691487437747399[3] = state[3];
   out_799691487437747399[4] = state[4];
   out_799691487437747399[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_799691487437747399[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_799691487437747399[7] = state[7];
   out_799691487437747399[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1338573725919830316) {
   out_1338573725919830316[0] = 1;
   out_1338573725919830316[1] = 0;
   out_1338573725919830316[2] = 0;
   out_1338573725919830316[3] = 0;
   out_1338573725919830316[4] = 0;
   out_1338573725919830316[5] = 0;
   out_1338573725919830316[6] = 0;
   out_1338573725919830316[7] = 0;
   out_1338573725919830316[8] = 0;
   out_1338573725919830316[9] = 0;
   out_1338573725919830316[10] = 1;
   out_1338573725919830316[11] = 0;
   out_1338573725919830316[12] = 0;
   out_1338573725919830316[13] = 0;
   out_1338573725919830316[14] = 0;
   out_1338573725919830316[15] = 0;
   out_1338573725919830316[16] = 0;
   out_1338573725919830316[17] = 0;
   out_1338573725919830316[18] = 0;
   out_1338573725919830316[19] = 0;
   out_1338573725919830316[20] = 1;
   out_1338573725919830316[21] = 0;
   out_1338573725919830316[22] = 0;
   out_1338573725919830316[23] = 0;
   out_1338573725919830316[24] = 0;
   out_1338573725919830316[25] = 0;
   out_1338573725919830316[26] = 0;
   out_1338573725919830316[27] = 0;
   out_1338573725919830316[28] = 0;
   out_1338573725919830316[29] = 0;
   out_1338573725919830316[30] = 1;
   out_1338573725919830316[31] = 0;
   out_1338573725919830316[32] = 0;
   out_1338573725919830316[33] = 0;
   out_1338573725919830316[34] = 0;
   out_1338573725919830316[35] = 0;
   out_1338573725919830316[36] = 0;
   out_1338573725919830316[37] = 0;
   out_1338573725919830316[38] = 0;
   out_1338573725919830316[39] = 0;
   out_1338573725919830316[40] = 1;
   out_1338573725919830316[41] = 0;
   out_1338573725919830316[42] = 0;
   out_1338573725919830316[43] = 0;
   out_1338573725919830316[44] = 0;
   out_1338573725919830316[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1338573725919830316[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1338573725919830316[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1338573725919830316[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1338573725919830316[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1338573725919830316[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1338573725919830316[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1338573725919830316[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1338573725919830316[53] = -9.8000000000000007*dt;
   out_1338573725919830316[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1338573725919830316[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1338573725919830316[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1338573725919830316[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1338573725919830316[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1338573725919830316[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1338573725919830316[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1338573725919830316[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1338573725919830316[62] = 0;
   out_1338573725919830316[63] = 0;
   out_1338573725919830316[64] = 0;
   out_1338573725919830316[65] = 0;
   out_1338573725919830316[66] = 0;
   out_1338573725919830316[67] = 0;
   out_1338573725919830316[68] = 0;
   out_1338573725919830316[69] = 0;
   out_1338573725919830316[70] = 1;
   out_1338573725919830316[71] = 0;
   out_1338573725919830316[72] = 0;
   out_1338573725919830316[73] = 0;
   out_1338573725919830316[74] = 0;
   out_1338573725919830316[75] = 0;
   out_1338573725919830316[76] = 0;
   out_1338573725919830316[77] = 0;
   out_1338573725919830316[78] = 0;
   out_1338573725919830316[79] = 0;
   out_1338573725919830316[80] = 1;
}
void h_25(double *state, double *unused, double *out_870220601115161893) {
   out_870220601115161893[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2427880263093653096) {
   out_2427880263093653096[0] = 0;
   out_2427880263093653096[1] = 0;
   out_2427880263093653096[2] = 0;
   out_2427880263093653096[3] = 0;
   out_2427880263093653096[4] = 0;
   out_2427880263093653096[5] = 0;
   out_2427880263093653096[6] = 1;
   out_2427880263093653096[7] = 0;
   out_2427880263093653096[8] = 0;
}
void h_24(double *state, double *unused, double *out_717591017730205382) {
   out_717591017730205382[0] = state[4];
   out_717591017730205382[1] = state[5];
}
void H_24(double *state, double *unused, double *out_202172479114784534) {
   out_202172479114784534[0] = 0;
   out_202172479114784534[1] = 0;
   out_202172479114784534[2] = 0;
   out_202172479114784534[3] = 0;
   out_202172479114784534[4] = 1;
   out_202172479114784534[5] = 0;
   out_202172479114784534[6] = 0;
   out_202172479114784534[7] = 0;
   out_202172479114784534[8] = 0;
   out_202172479114784534[9] = 0;
   out_202172479114784534[10] = 0;
   out_202172479114784534[11] = 0;
   out_202172479114784534[12] = 0;
   out_202172479114784534[13] = 0;
   out_202172479114784534[14] = 1;
   out_202172479114784534[15] = 0;
   out_202172479114784534[16] = 0;
   out_202172479114784534[17] = 0;
}
void h_30(double *state, double *unused, double *out_2272185059265618861) {
   out_2272185059265618861[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4488810078397963659) {
   out_4488810078397963659[0] = 0;
   out_4488810078397963659[1] = 0;
   out_4488810078397963659[2] = 0;
   out_4488810078397963659[3] = 0;
   out_4488810078397963659[4] = 1;
   out_4488810078397963659[5] = 0;
   out_4488810078397963659[6] = 0;
   out_4488810078397963659[7] = 0;
   out_4488810078397963659[8] = 0;
}
void h_26(double *state, double *unused, double *out_2875500943942096378) {
   out_2875500943942096378[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6169383581967709320) {
   out_6169383581967709320[0] = 0;
   out_6169383581967709320[1] = 0;
   out_6169383581967709320[2] = 0;
   out_6169383581967709320[3] = 0;
   out_6169383581967709320[4] = 0;
   out_6169383581967709320[5] = 0;
   out_6169383581967709320[6] = 0;
   out_6169383581967709320[7] = 1;
   out_6169383581967709320[8] = 0;
}
void h_27(double *state, double *unused, double *out_3555140314183069640) {
   out_3555140314183069640[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4731982522037318077) {
   out_4731982522037318077[0] = 0;
   out_4731982522037318077[1] = 0;
   out_4731982522037318077[2] = 0;
   out_4731982522037318077[3] = 1;
   out_4731982522037318077[4] = 0;
   out_4731982522037318077[5] = 0;
   out_4731982522037318077[6] = 0;
   out_4731982522037318077[7] = 0;
   out_4731982522037318077[8] = 0;
}
void h_29(double *state, double *unused, double *out_3631812590274988689) {
   out_3631812590274988689[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2046987865922500982) {
   out_2046987865922500982[0] = 0;
   out_2046987865922500982[1] = 1;
   out_2046987865922500982[2] = 0;
   out_2046987865922500982[3] = 0;
   out_2046987865922500982[4] = 0;
   out_2046987865922500982[5] = 0;
   out_2046987865922500982[6] = 0;
   out_2046987865922500982[7] = 0;
   out_2046987865922500982[8] = 0;
}
void h_28(double *state, double *unused, double *out_6095294295260484836) {
   out_6095294295260484836[0] = state[0];
}
void H_28(double *state, double *unused, double *out_83357594357174731) {
   out_83357594357174731[0] = 1;
   out_83357594357174731[1] = 0;
   out_83357594357174731[2] = 0;
   out_83357594357174731[3] = 0;
   out_83357594357174731[4] = 0;
   out_83357594357174731[5] = 0;
   out_83357594357174731[6] = 0;
   out_83357594357174731[7] = 0;
   out_83357594357174731[8] = 0;
}
void h_31(double *state, double *unused, double *out_4655836835435861220) {
   out_4655836835435861220[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2397234301216692668) {
   out_2397234301216692668[0] = 0;
   out_2397234301216692668[1] = 0;
   out_2397234301216692668[2] = 0;
   out_2397234301216692668[3] = 0;
   out_2397234301216692668[4] = 0;
   out_2397234301216692668[5] = 0;
   out_2397234301216692668[6] = 0;
   out_2397234301216692668[7] = 0;
   out_2397234301216692668[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7197372880823570987) {
  err_fun(nom_x, delta_x, out_7197372880823570987);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8538761970312805771) {
  inv_err_fun(nom_x, true_x, out_8538761970312805771);
}
void car_H_mod_fun(double *state, double *out_5606927004741127484) {
  H_mod_fun(state, out_5606927004741127484);
}
void car_f_fun(double *state, double dt, double *out_799691487437747399) {
  f_fun(state,  dt, out_799691487437747399);
}
void car_F_fun(double *state, double dt, double *out_1338573725919830316) {
  F_fun(state,  dt, out_1338573725919830316);
}
void car_h_25(double *state, double *unused, double *out_870220601115161893) {
  h_25(state, unused, out_870220601115161893);
}
void car_H_25(double *state, double *unused, double *out_2427880263093653096) {
  H_25(state, unused, out_2427880263093653096);
}
void car_h_24(double *state, double *unused, double *out_717591017730205382) {
  h_24(state, unused, out_717591017730205382);
}
void car_H_24(double *state, double *unused, double *out_202172479114784534) {
  H_24(state, unused, out_202172479114784534);
}
void car_h_30(double *state, double *unused, double *out_2272185059265618861) {
  h_30(state, unused, out_2272185059265618861);
}
void car_H_30(double *state, double *unused, double *out_4488810078397963659) {
  H_30(state, unused, out_4488810078397963659);
}
void car_h_26(double *state, double *unused, double *out_2875500943942096378) {
  h_26(state, unused, out_2875500943942096378);
}
void car_H_26(double *state, double *unused, double *out_6169383581967709320) {
  H_26(state, unused, out_6169383581967709320);
}
void car_h_27(double *state, double *unused, double *out_3555140314183069640) {
  h_27(state, unused, out_3555140314183069640);
}
void car_H_27(double *state, double *unused, double *out_4731982522037318077) {
  H_27(state, unused, out_4731982522037318077);
}
void car_h_29(double *state, double *unused, double *out_3631812590274988689) {
  h_29(state, unused, out_3631812590274988689);
}
void car_H_29(double *state, double *unused, double *out_2046987865922500982) {
  H_29(state, unused, out_2046987865922500982);
}
void car_h_28(double *state, double *unused, double *out_6095294295260484836) {
  h_28(state, unused, out_6095294295260484836);
}
void car_H_28(double *state, double *unused, double *out_83357594357174731) {
  H_28(state, unused, out_83357594357174731);
}
void car_h_31(double *state, double *unused, double *out_4655836835435861220) {
  h_31(state, unused, out_4655836835435861220);
}
void car_H_31(double *state, double *unused, double *out_2397234301216692668) {
  H_31(state, unused, out_2397234301216692668);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
