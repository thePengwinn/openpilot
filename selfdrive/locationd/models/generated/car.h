#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7197372880823570987);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8538761970312805771);
void car_H_mod_fun(double *state, double *out_5606927004741127484);
void car_f_fun(double *state, double dt, double *out_799691487437747399);
void car_F_fun(double *state, double dt, double *out_1338573725919830316);
void car_h_25(double *state, double *unused, double *out_870220601115161893);
void car_H_25(double *state, double *unused, double *out_2427880263093653096);
void car_h_24(double *state, double *unused, double *out_717591017730205382);
void car_H_24(double *state, double *unused, double *out_202172479114784534);
void car_h_30(double *state, double *unused, double *out_2272185059265618861);
void car_H_30(double *state, double *unused, double *out_4488810078397963659);
void car_h_26(double *state, double *unused, double *out_2875500943942096378);
void car_H_26(double *state, double *unused, double *out_6169383581967709320);
void car_h_27(double *state, double *unused, double *out_3555140314183069640);
void car_H_27(double *state, double *unused, double *out_4731982522037318077);
void car_h_29(double *state, double *unused, double *out_3631812590274988689);
void car_H_29(double *state, double *unused, double *out_2046987865922500982);
void car_h_28(double *state, double *unused, double *out_6095294295260484836);
void car_H_28(double *state, double *unused, double *out_83357594357174731);
void car_h_31(double *state, double *unused, double *out_4655836835435861220);
void car_H_31(double *state, double *unused, double *out_2397234301216692668);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}