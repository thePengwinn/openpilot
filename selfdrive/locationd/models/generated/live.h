#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1714139590100607649);
void live_err_fun(double *nom_x, double *delta_x, double *out_4734381944002936068);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6641555120034503616);
void live_H_mod_fun(double *state, double *out_3452772395566410425);
void live_f_fun(double *state, double dt, double *out_8111635540139481671);
void live_F_fun(double *state, double dt, double *out_5082976101736264509);
void live_h_4(double *state, double *unused, double *out_2067637480446307409);
void live_H_4(double *state, double *unused, double *out_7659764409753360472);
void live_h_9(double *state, double *unused, double *out_946917515667527202);
void live_H_9(double *state, double *unused, double *out_3499760728691743674);
void live_h_10(double *state, double *unused, double *out_8504343323004889212);
void live_H_10(double *state, double *unused, double *out_6754682501565135545);
void live_h_12(double *state, double *unused, double *out_9215057582622425738);
void live_H_12(double *state, double *unused, double *out_5767523255924229349);
void live_h_35(double *state, double *unused, double *out_9051161341501652915);
void live_H_35(double *state, double *unused, double *out_7420317606583583768);
void live_h_32(double *state, double *unused, double *out_4834041447458540479);
void live_H_32(double *state, double *unused, double *out_6807727229454058778);
void live_h_13(double *state, double *unused, double *out_2731948318297328391);
void live_H_13(double *state, double *unused, double *out_2599809282740318436);
void live_h_14(double *state, double *unused, double *out_946917515667527202);
void live_H_14(double *state, double *unused, double *out_3499760728691743674);
void live_h_33(double *state, double *unused, double *out_6594617187238831656);
void live_H_33(double *state, double *unused, double *out_4269760601944726164);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}