#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5871428431702092966);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7767822878109567636);
void gnss_H_mod_fun(double *state, double *out_2554109935190345649);
void gnss_f_fun(double *state, double dt, double *out_2003281458956840132);
void gnss_F_fun(double *state, double dt, double *out_6875832990013989469);
void gnss_h_6(double *state, double *sat_pos, double *out_6660006016391780320);
void gnss_H_6(double *state, double *sat_pos, double *out_651387725702215192);
void gnss_h_20(double *state, double *sat_pos, double *out_4892345393658224162);
void gnss_H_20(double *state, double *sat_pos, double *out_8576617868203100303);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4352390767440585904);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5527866029776887273);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4352390767440585904);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5527866029776887273);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}