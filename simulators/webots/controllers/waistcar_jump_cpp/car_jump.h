#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "ct_lqr.h"
using namespace Eigen;


class car_jump{
public:

    // parameters

    const double m1 = 1.5;
    const double m2 = 2;
    const double mr = 0.5;
    double M = m1+m2+mr;

    const double l1 = 0.2;
    const double l2 = 0.2;
    const double r = 0.05;

    double d1 = 0.5*l1;
    double d2 = 0.5*l2;
    double I1 = 1.0/12*m1*l1*l1;
    double I2 = 1.0/12*m2*l2*l2;
    double Ir = 1.0/2*mr*r*r;

    const double g = 9.8;
    const double mu = 0.8;

    // state

    const double *orientation = nullptr;
    const double *position = nullptr;
    double pitch_pos = 0;
    double last_pitch_pos = 0;
    double pitch_vel = 0;
    double pitch_int = 0;
    double roll_pos = 0;

    double waist_pos = 0;
    double last_waist_pos = 0;
    double waist_vel = 0;
    double waist_int = 0;

    double wheel_pos = 0;
    double last_wheel_pos = 0;
    double wheel_vel = 0;

    // inputtorque
    double fr = 0;
    double fb = 0;

    double c_pitch_pos_ref = 0;
    double c_pitch_vel_ref = 0;
    double wheel_pos_ref = 0;
    double wheel_vel_ref = 0;
    double waist_pos_ref = 2.2;

    // parameters compute

    double mc;
    double Ic;
    double lc;
    double c_pitch_pos;
    double c_pitch_vel;
    void modelcom();

    // simulation
    double t = 0;
    double dt = 0.001;
    void initial(int timestep);
    void balance_control();
    void run();
    void motor_limit();
private:




};