#include "car_jump.h"
#include "control_utils.h"
extern std::vector<double> poles_real;
extern std::vector<double> poles_imag;
extern std::vector<double> obsv_Q;
extern std::vector<double> obsv_R;
extern bool enable_three_state;
extern bool info_debug;
extern double waist_pos_param;
extern int cal_obsv_gain_steps;
extern bool enable_fric_comp;
extern int ramp_mode;
extern bool enable_stand_mode;
extern bool enable_waist_control;
extern double pitch_dist_lpf_freq;
extern double whtor_dist_lpf_freq;
extern double ext_wheel_vel_ref;
extern double launch_wheel_vel_ref;
extern double max_pitch_dist;
extern double max_wheel_tor;
extern double ramp_detect_tor;
extern double wheel_vel_lpf_freq;
extern double ramp_state2_wait_time;
extern double ramp_prepare_dist;
extern double ramp_raise_dist;
extern double ramp_zero_speed_dist;
extern double ramp_top_dist;
extern double stand_back_time;
extern double standup_fr;
extern double stand_switch_angle;
extern double remote_max_waist_angle;
extern double waist_pos_cmd_lpf_freq;
extern double jump_waist_tor;
extern double jump_wheel_tor;
extern bool jump_back_mode;
extern int jump_mode;
extern double jump_long_waist_pos2;

void car_jump::initial(double _dt)
{
    dt = _dt;
    process_sensordata();
    
    start_time = std::chrono::steady_clock::now();

    filter_open = true;
    pitch_pos_filter.setparams(1.0 / _dt, 50, pitch_pos);
    pitch_vel_filter.setparams(1.0 / _dt, 50, pitch_vel);
    wheel_pos_filter.setparams(1.0 / _dt, 50, wheel_pos);
    wheel_vel_filter.setparams(1.0 / _dt, wheel_vel_lpf_freq, wheel_vel);
    waist_pos_filter.setparams(1.0 / _dt, 50, waist_pos);
    pitch_dist_filter.setparams(1.0 / _dt, pitch_dist_lpf_freq, 0.0);
    whtor_dist_filter.setparams(1.0 / _dt, whtor_dist_lpf_freq, 0.0);
    waist_pos_cmd_filter.setparams(1.0 / _dt, waist_pos_cmd_lpf_freq, waist_pos);

    oQ = Matrix<double,5,5>::Zero();
    oR = Matrix<double,3,3>::Zero();
    for(int i = 0; i < 5; i++) {
        oQ(i,i) = obsv_Q[i];
    }
    for(int i = 0; i < 3; i++) {
        oR(i,i) = obsv_R[i];
    }
    L = Matrix<double,3,5>::Zero();

    // Optional: load controller tuning parameters from a file (one-time at startup).
    // std::string param_file = "/home/ubuntu/waistcar_ws/src/waistcar_control/src/params.txt";
    // if (readParamsFromFile(param_file)) {
    //     std::cout << "Parameters loaded: reference_pos = " << c_pitch_pos_ref 
    //     <<", wheel_vel_ref = " << wheel_vel_ref << ", force = " << fr_add << std::endl ;
    // } else {
    //     std::cout << "Failed to load parameters from file." << std::endl;
    // }

    // if(access("/home/ubuntu/waistcar_ws/data",0)==-1){
    // // #ifdef WIN32
    // //     mkdir("../data");
    // // #endif
    // #ifdef linux
    //     mkdir("/home/ubuntu/waistcar_ws/data",0777);
    // #endif
    // }

    std::string header_str[] = {"T","pitch_pos","pitch_vel","wheel_pos","wheel_vel"
                                ,"waist_pos", "waist_pos_cmd", "waist_vel" , "c_pitch_pos", "c_pitch_vel", "Fr","waist_tor_cmd","wheel_torque","waist_torque","error_sum_value","sum_flag", 
                                "imu_update_time", "est_pitch_dist", "est_whtor_dist",
                                "est_pitch_dist_filt", "est_whtor_dist_filt","ramp_mode", "ramp_state","ext_wheel_vel_ref"};

    int header_length = sizeof(header_str)/sizeof(header_str[0]);

    logger.init("/home/ubuntu/waistcar_ws/data/waistcar_control",true,header_str,header_length);
}

void car_jump::log()
{
    logger << t, pitch_pos, pitch_vel, wheel_pos, wheel_vel, 
              waist_pos, waist_pos_cmd, waist_vel, c_pitch_pos, c_pitch_vel, fr, waist_tor_cmd, wheel_torque, waist_torque, error_sum_value, sum_flag, 
              imu_update_time, est_pitch_dist, est_whtor_dist, est_pitch_dist_filt, est_whtor_dist_filt, ramp_mode, ramp_state, ext_wheel_vel_ref;
    logger << std::endl;
}
bool car_jump::readParamsFromFile(const std::string& filename)
{
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cerr << "Failed to open params file: " << filename << std::endl;
        return false;
    }

    std::string key;
    double value;

    while (fin >> key >> value) {
        if (key == "c_pitch_pos_ref") {
           // c_pitch_pos_ref = value;
        } else if (key == "fr_add") {
           // fr_add = value;
        }        
        else if (key == "wheel_vel_ref") {
           // wheel_vel_ref = value;
        };
    }
    fin.close();
    return true;
}

void car_jump::modelcom()
// double mc;
// double Ic;
// double lc;
// double c_pitch_pos;
// double c_pitch_vel;
{    
    // Optional: manual waist position override for bench testing (disabled).
    //waist_pos = -2.35;
    // waist_pos = -1.57;
    if (enable_waist_control) {
        waist_pos = motor_msg_array.waist_pos;
        waist_vel = motor_msg_array.waist_vel;
    } else {
        waist_pos = waist_pos_param;
        waist_vel = 0.0;
    } 

    // pitch_pos = 1;
    // pitch_vel = 1;
    // waist_vel = 2;
    
    
    mc = m1 + m2;
    // double sigma1 = m2*(d2*cos(waist_pos + pitch_pos) + l1*cos(pitch_pos)) + d1*m1*cos(pitch_pos);
    // double sigma2 = d1*m1*sin(pitch_pos) + l1*m2*sin(pitch_pos) + d2*m2*sin(waist_pos + pitch_pos);

    // Ic = I1+I2+m1*(pow(d1*sin(pitch_pos)-sigma2/mc,2)+pow(d1*cos(pitch_pos)-sigma1/mc,2))
    //         + m2*(pow(l1*sin(pitch_pos)+d2*sin(pitch_pos+waist_pos)-sigma2/mc,2)+pow(l1*cos(pitch_pos)+d2*cos(pitch_pos+waist_pos)-sigma1/mc,2));

    // lc = sqrt(pow(sigma1/mc,2)+pow(sigma2/mc,2));        

    // c_pitch_pos = atan(sigma2/sigma1);

    // c_pitch_vel = (pow(d2,2)*waist_vel*pow(m2,2) + pow(d1,2)*pitch_vel*pow(m1,2) + pow(d2,2)*pitch_vel*pow(m2,2) + pitch_vel*pow(l1,2)*pow(m2,2) + d2*waist_vel*l1*pow(m2,2)*cos(waist_pos) + 2*d2*pitch_vel*l1*pow(m2,2)*cos(waist_pos) + 2*d1*pitch_vel*l1*m1*m2 + d1*d2*waist_vel*m1*m2*cos(waist_pos) + 2*d1*d2*pitch_vel*m1*m2*cos(waist_pos))/(pow(d1,2)*pow(m1,2) + 2*cos(waist_pos)*d1*d2*m1*m2 + 2*d1*l1*m1*m2 + pow(d2,2)*pow(m2,2) + 2*cos(waist_pos)*d2*l1*pow(m2,2) + pow(l1,2)*pow(m2,2));
    
    double sinp = sin(pitch_pos);
    double cosp = cos(pitch_pos);
    double sinpw = sin(pitch_pos + waist_pos);
    double cospw = cos(pitch_pos + waist_pos);
    double sinw = sin(waist_pos);
    double cosw = cos(waist_pos);


    double sigma1 = m1*(h1*cosp + d1*sinp) + m2*(h2*cospw + d2*sinpw + l1*sinp);
    double sigma2 = m1*(d1*cosp - h1*sinp) + m2*(d2*cospw - h2*sinpw + l1*cosp);
    
    Ic = I1 + I2 + m1*(pow(d1*sinp+h1*cosp-sigma1/mc,2) + pow(d1*cosp-h1*sinp-sigma2/mc,2))
             + m2*(pow(l1*sinp+d2*sinpw+h2*cospw-sigma1/mc,2) + pow(l1*cosp+d2*cospw-h2*sinpw-sigma2/mc,2));

    lc = sqrt(pow(sigma1/mc,2) + pow(sigma2/mc,2));

    c_pitch_pos = atan(sigma1/sigma2);

    c_pitch_vel = (d2*d2*waist_vel*m2*m2 + d1*d1*pitch_vel*m1*m1 + d2*d2*pitch_vel*m2*m2 + waist_vel*h2*h2*m2*m2 + pitch_vel*h1*h1*m1*m1 + pitch_vel*h2*h2*m2*m2 + pitch_vel*l1*l1*m2*m2 + d2*waist_vel*l1*m2*m2*cosw + 2*d2*pitch_vel*l1*m2*m2*cosw - waist_vel*h2*l1*m2*m2*sinw - 2*pitch_vel*h2*l1*m2*m2*sinw + 2*d1*pitch_vel*l1*m1*m2 + d1*d2*waist_vel*m1*m2*cosw + 2*d1*d2*pitch_vel*m1*m2*cosw + waist_vel*h1*h2*m1*m2*cosw + 2*pitch_vel*h1*h2*m1*m2*cosw - d1*waist_vel*h2*m1*m2*sinw + d2*waist_vel*h1*m1*m2*sinw - 2*d1*pitch_vel*h2*m1*m2*sinw + 2*d2*pitch_vel*h1*m1*m2*sinw)/(d1*d1*m1*m1 + 2*cosw*d1*d2*m1*m2 - 2*sinw*d1*h2*m1*m2 + 2*d1*l1*m1*m2 + d2*d2*m2*m2 + 2*sinw*d2*h1*m1*m2 + 2*cosw*d2*l1*m2*m2 + h1*h1*m1*m1 + 2*cosw*h1*h2*m1*m2 + h2*h2*m2*m2 - 2*sinw*h2*l1*m2*m2 + l1*l1*m2*m2);


    // std::cout << "model com: c_pitch_pos" << c_pitch_pos << std::endl;
}

void car_jump::modelcom_ref()

{    

    // double c_pitch_pos_ref = 0;
    // double c_pitch_vel_ref = 0;
    // double wheel_pos_ref = 0;
    // double wheel_vel_ref = 0;
    // double waist_pos_cmd = 0;


    mc = m1 + m2;
 
    double sinp = sin(pitch_pos);
    double cosp = cos(pitch_pos);
    double sinpw = sin(pitch_pos + waist_pos);
    double cospw = cos(pitch_pos + waist_pos);
    double sinw = sin(waist_pos);
    double cosw = cos(waist_pos);


    double sigma1 = m1*(h1*cosp + d1*sinp) + m2*(h2*cospw + d2*sinpw + l1*sinp);
    double sigma2 = m1*(d1*cosp - h1*sinp) + m2*(d2*cospw - h2*sinpw + l1*cosp);
    
    Ic = I1 + I2 + m1*(pow(d1*sinp+h1*cosp-sigma1/mc,2) + pow(d1*cosp-h1*sinp-sigma2/mc,2))
             + m2*(pow(l1*sinp+d2*sinpw+h2*cospw-sigma1/mc,2) + pow(l1*cosp+d2*cospw-h2*sinpw-sigma2/mc,2));

    lc = sqrt(pow(sigma1/mc,2) + pow(sigma2/mc,2));

    c_pitch_pos = atan(sigma1/sigma2);

    c_pitch_vel = (d2*d2*waist_vel*m2*m2 + d1*d1*pitch_vel*m1*m1 + d2*d2*pitch_vel*m2*m2 + waist_vel*h2*h2*m2*m2 + pitch_vel*h1*h1*m1*m1 + pitch_vel*h2*h2*m2*m2 + pitch_vel*l1*l1*m2*m2 + d2*waist_vel*l1*m2*m2*cosw + 2*d2*pitch_vel*l1*m2*m2*cosw - waist_vel*h2*l1*m2*m2*sinw - 2*pitch_vel*h2*l1*m2*m2*sinw + 2*d1*pitch_vel*l1*m1*m2 + d1*d2*waist_vel*m1*m2*cosw + 2*d1*d2*pitch_vel*m1*m2*cosw + waist_vel*h1*h2*m1*m2*cosw + 2*pitch_vel*h1*h2*m1*m2*cosw - d1*waist_vel*h2*m1*m2*sinw + d2*waist_vel*h1*m1*m2*sinw - 2*d1*pitch_vel*h2*m1*m2*sinw + 2*d2*pitch_vel*h1*m1*m2*sinw)/(d1*d1*m1*m1 + 2*cosw*d1*d2*m1*m2 - 2*sinw*d1*h2*m1*m2 + 2*d1*l1*m1*m2 + d2*d2*m2*m2 + 2*sinw*d2*h1*m1*m2 + 2*cosw*d2*l1*m2*m2 + h1*h1*m1*m1 + 2*cosw*h1*h2*m1*m2 + h2*h2*m2*m2 - 2*sinw*h2*l1*m2*m2 + l1*l1*m2*m2);

}


void car_jump::run()
{    

    process_sensordata();

    sum_flag_change(wheel_vel_ref, wheel_vel);
    error_sum(sum_flag);
    
    if (enable_three_state) {
        balance_control();
    } else {
        balance_control_state4();
    }
    
    last_pitch_pos = pitch_pos;
    last_waist_pos = waist_pos;
    last_wheel_pos = wheel_pos;
    waist_int = waist_int + (waist_pos - waist_pos_cmd)*dt;

    log();
}

void car_jump::process_sensordata()
{
    pitch_pos = imu_msg.roll - 1.570796 + 0;
    pitch_vel = imu_msg.droll;
    // Waist angle derived from the waist motor encoder.
    waist_pos_degree = - motor_msg_array.waist_pos / 2194.5185;   //encoder data -> degree
    waist_vel = motor_msg_array.waist_vel;
    waist_torque = motor_msg_array.waist_tff;

    wheel_pos = motor_msg_array.wheel_pos - wheel_pos_offset;
    wheel_vel = motor_msg_array.wheel_vel;
    wheel_torque = motor_msg_array.wheel_tff;

    if (filter_open)
    {
        // pitch_pos = pitch_pos_filter.update(pitch_pos);
        // pitch_vel = pitch_vel_filter.update(pitch_vel);
        // wheel_pos = wheel_pos_filter.update(wheel_pos);
        wheel_vel = wheel_vel_filter.update(wheel_vel);
        // waist_pos = waist_pos_filter.update(waist_pos);
    }

}



void car_jump::balance_control_state4()
{
    modelcom();
    // std::cout<< t <<"  "<< mc <<"  "<< Ic <<"  "<< lc <<"  "<< c_pitch_pos <<"  "<< c_pitch_vel << std::endl;
    Matrix<double,4,1> rx;
    double wheel_pos_ref = 0.0;
    rx << c_pitch_pos_ref, c_pitch_vel_ref, wheel_vel_ref, wheel_pos_ref;
    Matrix<double,4,1> x;
    
    double sat_wheel_vel = std::min(std::max(wheel_vel, -3.0), 3.0);
    x << c_pitch_pos, c_pitch_vel, sat_wheel_vel, wheel_pos;
    
    Matrix<double,2,2> Mass;
    Mass(0,0) = Ic + Ir + pow(lc,2)*mc + mc*pow(r,2) + mr*pow(r,2) + 2*lc*mc*r*cos(c_pitch_pos_ref);
    Mass(0,1) = Ir + mr*pow(r,2) + mc*r*(r + lc*cos(c_pitch_pos_ref));
    Mass(1,0) = Mass(0,1);
    Mass(1,1) = Ir + mc*pow(r,2) + mr*pow(r,2);

    Matrix<double,2,1> pbpq;
    pbpq(0,0) = -lc*mc*cos(c_pitch_pos_ref)*(r*pow(c_pitch_vel_ref,2) + g);
    pbpq(1,0) = -pow(c_pitch_vel_ref,2)*lc*mc*r*cos(c_pitch_pos_ref);

    Matrix<double,2,2> pbpdq;
    pbpdq(0,0) = -lc*mc*sin(c_pitch_pos_ref)*r*c_pitch_vel_ref*2;
    pbpdq(1,0) = -c_pitch_vel_ref*2*lc*mc*r*sin(c_pitch_pos_ref);
    pbpdq(0,1) = 0;
    pbpdq(1,1) = 0;
    
    Matrix<double,4,4> A;
    A(0,0) = 0;
    A(0,1) = 1;
    A(0,2) = 0;
    A.block(1,0,2,1) = -Mass.inverse()*pbpq;
    A.block(1,1,2,2) = -Mass.inverse()*pbpdq;
    A(3,2) = 1;

    Matrix<double,4,1> B;
    B(0,0) = 0;
    Matrix<double,2,1> temp;
    temp << 0,
            1;
    // B.block(1,0,2,1) = Mass.inverse().block(0,1,2,1);
    B.block(1,0,2,1) = Mass.inverse()*temp;
    
    Matrix<double,1,4> K;
    K << 0, 0, 0, 0;
    // std::cout << A << std::endl;
    // std::cout << B << std::endl;
    std::vector<std::complex<double>> desired_poles = {{-3.5, 0}, {-3.0, 0}, {-1.1, 0.0}, {-1.0, 0.0}};
    for(int i = 0; i < 4; i++) {
        desired_poles[i] = std::complex<double>(poles_real[i], poles_imag[i]);
        if (info_debug) {
            std::cout << "real:" << poles_real[i] << ", imag:" << poles_imag[i] << std::endl;
        }
    }
    K = place_poles(A, B, desired_poles);
    output = K*(rx - x);
    output = std::max(-max_wheel_tor,std::min((double)output,max_wheel_tor));

    // --- Experimental stand-up routine (disabled) ---------------------------------
    //     if(stand_flag == 0 && mode_msg.car_mode == 4)
    // {
    //     if(standup_t < 300)
    //     {
    //         fr = 2;
    //         standup_t = standup_t + 1;
    //         // if(standup_t > 1005)
    //         // {standup_t = 0;}
    //     }
    //     else{
    //         fr = -7;
    //         if ((c_pitch_pos-c_pitch_pos_ref) > -0.4 && (c_pitch_pos-c_pitch_pos_ref) < 0.4)
    //         {
    //             stand_flag = 1;
    //         }
    //     }
        
    // }
    // else{
    //         // Empirical friction/velocity compensation (tuned experimentally).
    // }
    // --- End experimental stand-up routine ----------------------------------------
    fr = output;
    double dead_vel = 0.1;
    double cmp_tor = 0.4;
    if (wheel_vel < -dead_vel) {
        fr += -cmp_tor;
    } else if (wheel_vel > dead_vel) {
        fr += cmp_tor;
    } else {
        fr += wheel_vel/dead_vel*cmp_tor;
    }

    fb = 0;
}

void car_jump::calc_observer_gain(const Matrix<double,3,3>& A, const Matrix<double,3,1>& B) {
    oA.block(0,0,3,3) = A.block(0,0,3,3);
    oA.block(0,3,3,1) = A.block(0,0,3,1); // roll disturbance term
    oA.block(0,4,3,1) = B;               // torque disturbance term
    oB.block(0,0,3,1) = B;
    oC(0,0) = 1;
    oC(1,1) = 1;
    oC(2,2) = 1;
    // std::vector<std::complex<double>> desired_poles_obsv = {{-20.0, 0}, {-21.0, 0}, {-22.0, 0.0}, {-23.0, 0.0}};
    // L = place_poles(oAT, oC, desired_poles_obsv);

    lqrsolver.compute(oQ, oR, oA.transpose(), oC, L, true, true);
    if (info_debug) {
        std::cout << "oQ:" << oQ << std::endl;
        std::cout << "oR:" << oR << std::endl;
        std::cout << "L:" << L << std::endl;
        auto Lpoles = (oA.transpose() - oC*L).eigenvalues();
        std::cout << "Lpoles:" << Lpoles << std::endl;
    }
}

void car_jump::balance_control()
{
    if (mode_msg.car_mode == 4) {
        control_time = t - control_start_time;
        ecbc_time = t - ecbc_start_time;
    } else {
        control_time = 0.0;
        ecbc_time = 0.0;
    }
    modelcom();
    // std::cout<< t <<"  "<< mc <<"  "<< Ic <<"  "<< lc <<"  "<< c_pitch_pos <<"  "<< c_pitch_vel << std::endl;
    Matrix<double,3,1> rx;
    wheel_vel_ref = ext_wheel_vel_ref;
    rx << c_pitch_pos_ref, c_pitch_vel_ref, wheel_vel_ref;
    Matrix<double,3,1> x;
    x << c_pitch_pos, c_pitch_vel, wheel_vel;
    
    Matrix<double,2,2> Mass;
    Mass(0,0) = Ic + Ir + pow(lc,2)*mc + mc*pow(r,2) + mr*pow(r,2) + 2*lc*mc*r*cos(c_pitch_pos_ref);
    Mass(0,1) = Ir + mr*pow(r,2) + mc*r*(r + lc*cos(c_pitch_pos_ref));
    Mass(1,0) = Mass(0,1);
    Mass(1,1) = Ir + mc*pow(r,2) + mr*pow(r,2);

    Matrix<double,2,1> pbpq;
    pbpq(0,0) = -lc*mc*cos(c_pitch_pos_ref)*(r*pow(c_pitch_vel_ref,2) + g);
    pbpq(1,0) = -pow(c_pitch_vel_ref,2)*lc*mc*r*cos(c_pitch_pos_ref);

    Matrix<double,2,2> pbpdq;
    pbpdq(0,0) = -lc*mc*sin(c_pitch_pos_ref)*r*c_pitch_vel_ref*2;
    pbpdq(1,0) = -c_pitch_vel_ref*2*lc*mc*r*sin(c_pitch_pos_ref);
    pbpdq(0,1) = 0;
    pbpdq(1,1) = 0;
    

    Matrix<double,3,3> A;
    A(0,0) = 0;
    A(0,1) = 1;
    A(0,2) = 0;
    A.block(1,0,2,1) = -Mass.inverse()*pbpq;
    A.block(1,1,2,2) = -Mass.inverse()*pbpdq;

    Matrix<double,3,1> B;
    B(0,0) = 0;
    Matrix<double,2,1> temp;
    temp << 0,
            1;
    // B.block(1,0,2,1) = Mass.inverse().block(0,1,2,1);
    B.block(1,0,2,1) = Mass.inverse()*temp;
    
    Matrix<double,1,3> K;
    K << 0, 0, 0;
    // std::cout << A << std::endl;
    // std::cout << B << std::endl;
    // std::vector<std::complex<double>> desired_poles = {{-6.5, 0}, {-6.0, 0}, {-3.5, 0.0}}; // legacy poles
    std::vector<std::complex<double>> desired_poles = {{-6.5, 0}, {-6.0, 0}, {-3.5, 0.0}};
    
    for(int i = 0; i < 3; i++) {
        desired_poles[i] = std::complex<double>(poles_real[i], poles_imag[i]);
        // printf("real:%.2f, imag:%.2f", poles_real[i], poles_imag[i]);
    }

    //read param poles value from txt
   //std::vector<std::complex<double>> desired_poles = desired_poles_param;
    K = place_poles(A, B, desired_poles);
    //K = place_poles(A, B, desired_poles_param);
    //std::cout << "K: " << K << std::endl;

    // Example gain set from previous tuning (kept for reference): K << ...

    if (ct_cnt%cal_obsv_gain_steps == 0) {
        if (info_debug) {
            std::cout << "cal_obsv_gain_steps: " << cal_obsv_gain_steps << std::endl;
        }
        calc_observer_gain(A, B);
    }

    Matrix<double,5,1> d_est_state;
    Matrix<double,3,1> est_y = est_state.block(0,0,3,1); 
    Matrix<double,3,1> msr_y = x.block(0,0,3,1); 
    d_est_state = oA*est_state + oB*last_output - L.transpose()*(est_y-msr_y);
    est_state += d_est_state*dt;
    est_pitch_dist = std::min(max_pitch_dist, std::max(est_state(3), -max_pitch_dist));
    est_whtor_dist = est_state(4);

    double ramp_ang = 0.0*M_PI/180.0;
    double ramp_whtor = 9.4*9.81*sin(ramp_ang)*0.09;
    double ramp_pitch = ramp_whtor/(mc*9.81*lc);
 
    if (ramp_mode == 1) {
        if (ramp_state == 0 && t > control_start_time + 1.5 && wheel_pos*0.09 < -ramp_prepare_dist &&
            est_whtor_dist > ramp_detect_tor && last_est_whtor_dist < ramp_detect_tor) {
            ramp_ang = 20.0*M_PI/180.0;
            ramp_whtor = 9.4*9.81*sin(ramp_ang)*0.09;
            ramp_pitch = ramp_whtor/(mc*9.81*lc);
            est_pitch_dist = ramp_pitch;
            est_whtor_dist = ramp_whtor;
            est_state(3) = ramp_pitch;
            est_state(4) = ramp_whtor;
            pitch_dist_filter.state = ramp_pitch;
            whtor_dist_filter.state = ramp_whtor;
            ramp_state = 1;
        }
        if (ramp_state == 1 && t > control_start_time + 1.5 && wheel_pos*0.09 < -(ramp_prepare_dist+ramp_raise_dist) &&
            est_whtor_dist < ramp_detect_tor) {
            ramp_ang = 0.0*M_PI/180.0;
            ramp_whtor = 9.4*9.81*sin(ramp_ang)*0.09;
            ramp_pitch = ramp_whtor/(mc*9.81*lc);
            est_pitch_dist = ramp_pitch;
            est_whtor_dist = ramp_whtor;
            est_state(3) = ramp_pitch;
            est_state(4) = ramp_whtor;
            pitch_dist_filter.state = ramp_pitch;
            whtor_dist_filter.state = ramp_whtor;
            ramp_state = 2;
            ramp_state2_time = t;
        }
        if (ramp_state == 2 && wheel_pos*0.09 < -(ramp_prepare_dist+ramp_raise_dist+ramp_zero_speed_dist) ) {
            ext_wheel_vel_ref = 0.0;
        }
    } else if (ramp_mode == 2) {
        // if (ramp_state == 0 && control_time > 3) {
        //     ext_wheel_vel_ref = 8.0;
        // }
        if (ramp_state == 0 && control_time > 1.5 && wheel_pos*0.09 > ramp_prepare_dist &&
            est_whtor_dist > ramp_detect_tor && last_est_whtor_dist < ramp_detect_tor) {
            ramp_ang = 20.0*M_PI/180.0;
            ramp_whtor = 9.4*9.81*sin(ramp_ang)*0.09;
            ramp_pitch = ramp_whtor/(mc*9.81*lc);
            est_pitch_dist = ramp_pitch;
            est_whtor_dist = ramp_whtor;
            est_state(3) = ramp_pitch;
            est_state(4) = ramp_whtor;
            pitch_dist_filter.state = ramp_pitch;
            whtor_dist_filter.state = ramp_whtor;
            ramp_state = 1;
        }
        if (ramp_state == 1 && control_time > 1.5 && wheel_pos*0.09 > (ramp_prepare_dist+ramp_raise_dist) &&
            est_whtor_dist < ramp_detect_tor) {
            ramp_ang = 0.0*M_PI/180.0;
            ramp_whtor = 9.4*9.81*sin(ramp_ang)*0.09;
            ramp_pitch = ramp_whtor/(mc*9.81*lc);
            est_pitch_dist = ramp_pitch;
            est_whtor_dist = ramp_whtor;
            est_state(3) = ramp_pitch;
            est_state(4) = ramp_whtor;
            pitch_dist_filter.state = ramp_pitch;
            whtor_dist_filter.state = ramp_whtor;
            ramp_state = 2;
            ramp_state2_time = t;
        }
    }
    last_est_whtor_dist = est_whtor_dist;

    est_pitch_dist_filt = pitch_dist_filter.update(est_pitch_dist);
    est_whtor_dist_filt = whtor_dist_filter.update(est_whtor_dist);
    if (info_debug) {
        std::cout << "t:" << t << ", control_start_time:" << control_start_time << std::endl;
        std::cout << "est_pitch_dist_filt:" << est_pitch_dist_filt << std::endl;
        std::cout << "est_whtor_dist_filt:" << est_whtor_dist_filt << std::endl;
    }
    if (ecbc_time > 1.5) {
        x(0,0) = x(0,0) + est_pitch_dist_filt;
    }
    output = K*(rx - x);
    if (ecbc_time > 1.5) {
        output = output - est_whtor_dist_filt;
    }
    // Note: output saturation was previously fixed at ±2.2; now uses max_wheel_tor.
    //output = std::max(-2.22,std::min((double)output,2.22));
    output = std::max(-max_wheel_tor,std::min((double)output,max_wheel_tor));
    last_output = output;
    //fr = output - 0.06 * wheel_vel +0.4 * (wheel_vel/(abs(wheel_vel)+0.1));

    if (enable_stand_mode && mode_msg.car_mode == 4) {
        if (control_time < stand_back_time) {
            fr = 2.0;
        } else if (fabs(c_pitch_pos) > stand_switch_angle){
            // fr = sgn(c_pitch_pos) * 7.0;   // legacy value
            fr = sgn(c_pitch_pos) * standup_fr; // configurable via ROS parameter
            est_pitch_dist = 0.0;
            est_whtor_dist = 0.0;
            last_est_whtor_dist = 0.0;
            pitch_dist_filter.reset();
            whtor_dist_filter.reset();
            est_state(0) = c_pitch_pos;
            est_state(1) = c_pitch_vel;
            est_state(2) = wheel_vel;
            est_state(3) = 0.0;
            est_state(4) = 0.0;
            ecbc_start_time = t;
            ecbc_time = 0.0;
            last_output = fr;
        } else {
            fr = output;
        }
    } else {
        fr = output;
    }
    if (jump_mode == 0) {
            waist_mode_cmd = 7;
            waist_tor_cmd = 0.0;
            waist_pos_cmd = remote_max_waist_angle * remote_msg.right_updown;
    }
    if(jump_mode == 1){
        // Long jump
        // if (jump_phase == 0 && remote_msg.right_side > 0.8 && last_remote_msg.right_side <= 0.8) {
        //if (jump_phase == 0 && remote_msg.right_side > 0.8 && remote_msg.buttons[3] == 3)
        //if (jump_phase == 0 && jump_long_flag == 1 && remote_msg.buttons[3] == 3)
        if (jump_phase == 0 && remote_msg.right_side > 0.8 && last_remote_msg.right_side <= 0.8)
        {
            jump_start_time = control_time;
            jump_phase = 1;
        } 
        // (Legacy branch notes removed; jump phases handled below.)
        else if (jump_phase == 1) {
            waist_mode_cmd = 4;
            waist_tor_cmd = jump_waist_tor;
            fr = jump_wheel_tor;
            if (waist_pos > 0.0) {
                jump_phase = 2;
            }
        } else if (jump_phase == 2) {
            waist_mode_cmd = 4;
            waist_tor_cmd = -jump_waist_tor;
            // waist_pos_cmd = jump_long_waist_pos2;
            // waist_pos_cmd_filter.state = waist_pos_cmd;
            fr = 1 * (ext_wheel_vel_ref - wheel_vel);
            if (abs(waist_pos - jump_long_waist_pos2) < 0.2 || abs(waist_pos-remote_max_waist_angle) < 0.2) {
                jump_phase = 3;
            }
        } else if (jump_phase == 3) {
            waist_mode_cmd = 7;
            waist_tor_cmd = 0.0;
            waist_pos_cmd = jump_long_waist_pos2;
            waist_pos_cmd_filter.state = waist_pos_cmd;
            if (abs(c_pitch_pos) > 0.4) {
                fr = 1 * (ext_wheel_vel_ref*0.5 - wheel_vel);
            }
        } 
        // (End of jump phase handling.)
        else {
            waist_mode_cmd = 7;
            waist_tor_cmd = 0.0;
            waist_pos_cmd = remote_max_waist_angle * remote_msg.right_updown;
        }
    }

    // High jump
    if(jump_mode == 2){
    ////// if (jump_phase == 0 && remote_msg.right_side > 0.8 && last_remote_msg.right_side <= 0.8) 
        //if (jump_phase == 0 && remote_msg.right_side > 0.8 && remote_msg.buttons[3] == 3) 
        if (jump_phase == 0 && remote_msg.right_side > 0.8 && last_remote_msg.right_side <= 0.8)
        {
            jump_start_time = control_time;
            jump_phase = 1;
        } else if (jump_phase == 1) {
            waist_mode_cmd = 4;
            waist_tor_cmd = jump_waist_tor;
            fr = jump_wheel_tor;
            if (waist_pos > 0.1) {
                jump_phase = 2;
            }
        } else if (jump_phase == 2) {
            waist_mode_cmd = 4;
            waist_tor_cmd = jump_waist_tor;
            fr = 1 * (0 - wheel_vel);
            if (waist_pos > 1.5) {
                jump_phase = 3;
            }
        } else if (jump_phase == 3) {
            waist_mode_cmd = 7;
            waist_tor_cmd = 0.0;
            waist_pos_cmd = -remote_max_waist_angle * remote_msg.right_updown;
            waist_pos_cmd_filter.state = waist_pos_cmd;
            if (abs(c_pitch_pos) > 0.4) {
                fr = 1 * (ext_wheel_vel_ref - wheel_vel);
            }
        } else {
            waist_mode_cmd = 7;
            waist_tor_cmd = 0.0;
            waist_pos_cmd = remote_max_waist_angle * remote_msg.right_updown;
        }
    }
        // v = 2.5 * 6 = 15 rad/s; 2.3 rad / 15 = 0.15;
        // else if (jump_phase == 1) {
        //     waist_pos_cmd = 2.3;
        //     if (waist_pos > 0.1) {
        //         jump_phase = 2;
        //     }
        // } else if (jump_phase == 2) {
        //     waist_pos_cmd = -2.3;
        //     if (abs(waist_pos-waist_pos_cmd)<0.01) {
        //         jump_phase = 3;
        //     }
        // } else if (jump_phase == 3) {
        //     waist_pos_cmd = -2.3;
        //     if (abs(c_pitch_pos) > 0.4) {
        //         fr = 1 * (ext_wheel_vel_ref - wheel_vel);
        //     }
        // }
    // --- Experimental stand-up routine (disabled) ---------------------------------
    //     if(stand_flag == 0 && mode_msg.car_mode == 4)
    // {
    //     if(standup_t < 300)
    //     {
    //         fr = 2;
    //         standup_t = standup_t + 1;
    //         // if(standup_t > 1005)
    //         // {standup_t = 0;}
    //     }
    //     else{
    //         fr = -7;
    //         if ((c_pitch_pos-c_pitch_pos_ref) > -0.4 && (c_pitch_pos-c_pitch_pos_ref) < 0.4)
    //         {
    //             stand_flag = 1;
    //         }
    //     }
        
    // }
    // else{
    //         // Empirical friction/velocity compensation (tuned experimentally).
    // }
    // --- End experimental stand-up routine ----------------------------------------
// Original control output 'fr' (kept for reference) - START
    // if (std::fabs(est_pitch_dist) < 0.1) {
    if (enable_fric_comp) {
        if (info_debug) {
            std::cout << "enable_fric_comp:" << enable_fric_comp << std::endl;
        }
        // if (wheel_vel < -0.1) { 
        //     fr += -0.4;
        // } else if (wheel_vel > 0.1) {
        //     fr += 0.4;
        // }
        fr += - 0 * wheel_vel + 0.4 * (wheel_vel/(abs(wheel_vel)+0.1)); // empirical friction compensation
    }
    // Original control output 'fr' (kept for reference) - END
    //fr = output - 0 * wheel_vel + 0 * (wheel_vel/(abs(wheel_vel)+0.1)) - 0.1 * wheel_int;
    //fr = output - 0 * wheel_vel + 0 * (wheel_vel/(abs(wheel_vel)+0.1)) + fr_error_sum;
    // waist_vel_cmd= 0;
    fb = 0;
    //std::cout << "fr: "  << fr << " c_pitch_pos: "  << c_pitch_pos << std::endl;
    ct_cnt++;

   
    waist_pos_cmd = waist_pos_cmd_filter.update(waist_pos_cmd); 
    last_remote_msg = remote_msg;
}

void car_jump::sum_flag_change(double wheel_vel_ref_func, double wheel_vel_func)
{
    if(abs(wheel_vel_ref_func - wheel_vel_func) < 0.1)
    {
        sum_flag = true;
    }
}
void car_jump::error_sum(bool sum_flag_func)
{
    if(sum_flag_func)
    {
        error_sum_value = error_sum_value + (wheel_vel - wheel_vel_ref) * dt;
        fr_error_sum = K_error_sum * error_sum_value;
    }
    //std::cout << "wheel_vel: " << wheel_vel << ", wheel_vel_ref: " << wheel_vel_ref <<", sum_flag: " << sum_flag << ", error_sum_value: " << error_sum_value <<  std::endl;
}
void car_jump::motor_limit()
{
    double maxfb = 0;
    if (abs(waist_vel) > 21)
    {
        maxfb = 0;
    }
    else
    {
        maxfb = abs(79.64 - 3.9541*abs(waist_vel));
        maxfb = std::min(maxfb,33.0);        
    }
    if (waist_vel > 0)
    {
        fb = std::max(-33.0,std::min(fb,maxfb));       
    }
    else
    {
        fb = std::max(-maxfb,std::min(fb,33.0));
    }


    double maxfr = 0;
    if (abs(wheel_vel) > 35)
    {
        maxfr = 0;
    }
    else
    {
        maxfr = abs(47.78 - 1.42*abs(wheel_vel));
        maxfr = std::min(maxfr,19.8);
    }

    if (wheel_vel > 0)
    {
        fr = std::max(-19.8,std::min(fr,maxfr));
    }
    else
    {
        fr = std::max(-maxfr,std::min(fr,19.8));
    }
}

void car_jump::clean()
{
    logger.close();
}

void car_jump::waist_control()
{   
    waist_vel_cmd= 0;
    fb = 0;
}

double sgn(const double& in) {
    return (in > 0.0 ? 1.0 : (in < 0.0 ? -1.0 : 0.0));
}