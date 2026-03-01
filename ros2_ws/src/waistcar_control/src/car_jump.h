#pragma once
#include <iostream>
#include <cmath>
#include <algorithm>
#include <unistd.h>
#include <sys/stat.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <chrono>
#include "lqr_inst.h"
#include "control_utils.h"


#include "waistcar_msgs/msg/motor_feedback_msg.hpp"
#include "waistcar_msgs/msg/imu_msg.hpp"
#include "waistcar_msgs/msg/remote_msg.hpp"
#include "waistcar_msgs/msg/mode_command_msg.hpp"
#include "waistcar_msgs/msg/motor_command_msg.hpp"
#include "lowpassfilter.h"
#include "data_logger.h"
using namespace Eigen;

double sgn(const double& in);

class car_jump{
public:

    // Parameters / configuration
    std::chrono::steady_clock::time_point start_time;

    my_lqr::LQR53 lqrsolver;
    
    const double m1 = 1.70208;
    const double m2 = 6.27033;
    const double mr = 0.41106;
    double M = m1+m2+mr;

    const double l1 = 0.3;
    const double l2 = 0.3;
    const double r = 0.09;

    double d1 = 0.14332;
    double d2 = 0.16954;
    double h1 = 0.016;
    double h2 = 0.015;
    double I1 = 0.02664;
    double I2 = 0.05828;
    // double Ir = 1.0/2*mr*r*r;
    double Ir = 0.00139;  
    const double g = 9.8;
    const double mu = 0.8;

    // Gear reduction ratios
    double belt_ratio = 0.6111;   //44/72 = 0.6111
    double motor_gear_ratio = 9.1;

    // State variables (inputs/feedback/commands)

    // const double *orientation = nullptr;
    // const double *position = nullptr;
    waistcar_msgs::msg::MotorFeedbackMsg motor_msg_array;
    waistcar_msgs::msg::ImuMsg imu_msg;
    waistcar_msgs::msg::RemoteMsg remote_msg;
    waistcar_msgs::msg::RemoteMsg last_remote_msg;
    waistcar_msgs::msg::ModeCommandMsg mode_msg;
    waistcar_msgs::msg::ModeCommandMsg last_mode_msg;
    waistcar_msgs::msg::MotorCommandMsg motor_cmd_array;
    double wheel_pos_offset = 0;

    double pitch_pos = 0;
    double last_pitch_pos = 0;
    double pitch_vel = 0;
    double pitch_int = 0;
    double roll_pos = 0;

    double waist_pos = -2.35;
    double waist_pos_degree = 0;
    double last_waist_pos = 0;
    double waist_vel = 0;
    double waist_torque = 0;
    double waist_int = 0;

    double wheel_pos = 0;
    double last_wheel_pos = 0;
    double wheel_vel = 0;
    double wheel_torque = 0;
    double wheel_int = 0;

    bool filter_open = false;
    lowpassfilter pitch_pos_filter;
    lowpassfilter pitch_vel_filter;
    lowpassfilter wheel_pos_filter;
    lowpassfilter wheel_vel_filter;
    lowpassfilter waist_pos_filter;
    lowpassfilter pitch_dist_filter;
    lowpassfilter whtor_dist_filter;
    lowpassfilter waist_pos_cmd_filter;
    double est_pitch_dist_filt;
    double est_whtor_dist_filt;
    double last_est_whtor_dist;

    Matrix<double,5,1> est_state;
    double est_pitch_dist;
    double est_whtor_dist;
    double control_start_time;
    double ecbc_start_time;
    double control_time = 0.0;
    double ecbc_time = 0.0;
    double last_output = 0.0;
    int ct_cnt = 0;
    Matrix<double,3,5> L;
    Matrix<double,5,5> oQ;
    Matrix<double,3,3> oR;
    Matrix<double,5,5> oA = Matrix<double,5,5>::Zero();
    Matrix<double,5,1> oB = Matrix<double,5,1>::Zero();
    Matrix<double,5,3> oC = Matrix<double,5,3>::Zero();

    // Control outputs (torque/velocity commands)
    double output = 0;
    double fr = 0;
    double fr_add = 0;
    double fb = 0;
    double waist_vel_cmd = 0;

    //double c_pitch_pos_ref = 0.03;
    //double c_pitch_pos_ref = -0.042;
    //double c_pitch_pos_ref = 0.45;
    //double c_pitch_pos_ref = -0.48;
    //double c_pitch_pos_ref = 0.035;  
    double c_pitch_pos_ref = 0;
    double c_pitch_vel_ref = 0;
    double wheel_pos_ref = 0;
    double wheel_vel_ref = 0;
    double waist_pos_cmd = 0.0;
    double waist_tor_cmd = 0.0;
    int waist_mode_cmd = 0;
    int ramp_state = 0;
    double ramp_state2_time = 0.0;

    bool integral_enabled = false;
    double error = 0.0;

    int error_sum_value = 0;
    int fr_error_sum = 0;
    double K_error_sum = 0.1;

    double jump_start_time = 0.0;
    int jump_phase = 0;
    
    //Matrix<double,1,4> K;
    Matrix<double,1,3> K;
    double K2;
    double K4;
    
    // Mode transition flags
    int stand_flag = 0;
    int standup_t = 0;
    bool sum_flag = false;
    // Jump flags
    int jump_long_flag = 0;

    // Perception: obstacle descriptors
    double obstacle_type = 0;
    double obstacle_distance = 0;
    double obstacle_zmin = 0;
    double obstacle_zmax = 0;

    // Two-link equivalent model parameters

    double mc;
    double Ic;
    double lc;
    double c_pitch_pos;
    double c_pitch_vel;
    void modelcom();
    void modelcom_ref();
    bool readParamsFromFile(const std::string& filename);

    // Simulation / logging
    double imu_update_time = 0;
    double t = 0;
    double dt = 0.001;
    DataLogger logger;
    void initial(double dt);
    void calc_observer_gain(const Matrix<double,3,3>& A, const Matrix<double,3,1>& B);
    void balance_control();
    void balance_control_state4();
    void waist_control();
    void run();
    void process_sensordata();
    void error_sum(bool sum_flag_func);
    void sum_flag_change(double wheel_vel_ref_func, double wheel_vel_func);
    void motor_limit();
    void log();
    void clean();
private:




};