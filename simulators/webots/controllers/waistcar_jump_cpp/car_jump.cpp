#include "car_jump.h"

void car_jump::initial(int timestep)
{
    dt = timestep*0.001;
    pitch_pos = *orientation;
    // roll_pos = *orientation;
    // if (roll_pos > 0)
    // {
    //     pitch_pos = 1.57 - *(orientation+1);
    // }
    // else
    // {
    //     pitch_pos = *(orientation+1) - 1.57;
    // }
    
    last_pitch_pos = pitch_pos;
    pitch_vel = 0;
    

    last_waist_pos = waist_pos;
    waist_vel = 0;
    waist_int = 0;

    last_wheel_pos = wheel_pos;
    wheel_vel = 0;

}

void car_jump::modelcom()
// double mc;
// double Ic;
// double lc;
// double c_pitch_pos;
// double c_pitch_vel;
{    
    // waist_pos = 2.2;
    // pitch_pos = 1;
    // pitch_vel = 1;
    // waist_vel = 2;
    
    
    mc = m1 + m2;
    double sigma1 = m2*(d2*cos(waist_pos + pitch_pos) + l1*cos(pitch_pos)) + d1*m1*cos(pitch_pos);
    double sigma2 = d1*m1*sin(pitch_pos) + l1*m2*sin(pitch_pos) + d2*m2*sin(waist_pos + pitch_pos);

    Ic = I1+I2+m1*(pow(d1*sin(pitch_pos)-sigma2/mc,2)+pow(d1*cos(pitch_pos)-sigma1/mc,2))
            + m2*(pow(l1*sin(pitch_pos)+d2*sin(pitch_pos+waist_pos)-sigma2/mc,2)+pow(l1*cos(pitch_pos)+d2*cos(pitch_pos+waist_pos)-sigma1/mc,2));

    lc = sqrt(pow(sigma1/mc,2)+pow(sigma2/mc,2));        

    c_pitch_pos = atan(sigma2/sigma1);

    c_pitch_vel = (pow(d2,2)*waist_vel*pow(m2,2) + pow(d1,2)*pitch_vel*pow(m1,2) + pow(d2,2)*pitch_vel*pow(m2,2) + pitch_vel*pow(l1,2)*pow(m2,2) + d2*waist_vel*l1*pow(m2,2)*cos(waist_pos) + 2*d2*pitch_vel*l1*pow(m2,2)*cos(waist_pos) + 2*d1*pitch_vel*l1*m1*m2 + d1*d2*waist_vel*m1*m2*cos(waist_pos) + 2*d1*d2*pitch_vel*m1*m2*cos(waist_pos))/(pow(d1,2)*pow(m1,2) + 2*cos(waist_pos)*d1*d2*m1*m2 + 2*d1*l1*m1*m2 + pow(d2,2)*pow(m2,2) + 2*cos(waist_pos)*d2*l1*pow(m2,2) + pow(l1,2)*pow(m2,2));
    
    //(d2^2*waist_vel*m2^2 + d1^2*pitch_vel*m1^2 + d2^2*pitch_vel*m2^2 + pitch_vel*l1^2*m2^2 + d2*waist_vel*l1*m2^2*cos(waist_pos) + 2*d2*pitch_vel*l1*m2^2*cos(waist_pos) + 2*d1*pitch_vel*l1*m1*m2 + d1*d2*waist_vel*m1*m2*cos(waist_pos) + 2*d1*d2*pitch_vel*m1*m2*cos(waist_pos))/(d1^2*m1^2 + 2*cos(waist_pos)*d1*d2*m1*m2 + 2*d1*l1*m1*m2 + d2^2*m2^2 + 2*cos(waist_pos)*d2*l1*m2^2 + l1^2*m2^2)
}


void car_jump::run()
{
    if (orientation != nullptr)
    {
        pitch_pos = *orientation;
        // roll_pos = *orientation;
        // if (roll_pos > 0)
        // {
        //     pitch_pos = 1.57 - *(orientation+1);
        // }
        // else
        // {
        //     pitch_pos = *(orientation+1) - 1.57;
        // }
    }
    pitch_vel = (pitch_pos - last_pitch_pos)/dt;
    waist_vel = (waist_pos - last_waist_pos)/dt;
    wheel_vel = (wheel_pos - last_wheel_pos)/dt;

    if (t > 3)
    {
        waist_pos_ref = 1.5;
    }

    balance_control();
    fb = -10*(waist_pos-waist_pos_ref) - 1*waist_vel - 25*waist_int;
    // std::cout <<"t:  "<< t << "  waist_pos:  "<< waist_pos << "  fb:  " << fb<< std::endl;

    motor_limit();



    last_pitch_pos = pitch_pos;
    last_waist_pos = waist_pos;
    last_wheel_pos = wheel_pos;
    waist_int = waist_int + (waist_pos - waist_pos_ref)*dt;

}

void car_jump::balance_control()
{
    modelcom();
    // std::cout<< t <<"  "<< mc <<"  "<< Ic <<"  "<< lc <<"  "<< c_pitch_pos <<"  "<< c_pitch_vel << std::endl;
    Matrix<double,4,1> rx;
    rx << c_pitch_pos_ref, wheel_pos_ref, c_pitch_vel_ref, wheel_vel_ref;
    Matrix<double,4,1> x;
    x << c_pitch_pos, wheel_pos, c_pitch_vel, wheel_vel;
    
    Matrix<double,2,2> Mass;
    Mass(0,0) = Ic + Ir + pow(lc,2)*mc + mc*pow(r,2) + mr*pow(r,2) + 2*lc*mc*r*cos(c_pitch_pos_ref);
    Mass(0,1) = Ir + mr*pow(r,2) + mc*r*(r + lc*cos(c_pitch_pos_ref));
    Mass(1,0) = Mass(0,1);
    Mass(1,1) = Ir + mc*pow(r,2) + mr*pow(r,2);

    Matrix<double,2,2> pbpq;
    pbpq(0,0) = -lc*mc*cos(c_pitch_pos_ref)*(r*pow(c_pitch_vel_ref,2) + g);
    pbpq(1,0) = -pow(c_pitch_vel_ref,2)*lc*mc*r*cos(c_pitch_pos_ref);
    pbpq(0,1) = 0;
    pbpq(1,1) = 0;

    Matrix<double,2,2> pbpdq;
    pbpdq(0,0) = -lc*mc*sin(c_pitch_pos_ref)*r*c_pitch_vel_ref*2;
    pbpdq(1,0) = -c_pitch_vel_ref*2*lc*mc*r*sin(c_pitch_pos_ref);
    pbpdq(0,1) = 0;
    pbpdq(1,1) = 0;
    

    Matrix<double,4,4> A;   
    A(0,0) = 0;
    A(0,1) = 0;
    A(0,2) = 1;
    A(0,3) = 0;

    A(1,0) = 0;
    A(1,1) = 0;
    A(1,2) = 0;
    A(1,3) = 1;    
    A.block(2,0,2,2) = -Mass.inverse()*pbpq;
    A.block(2,2,2,2) = -Mass.inverse()*pbpdq;

    Matrix<double,4,1> B;
    B(0,0) = 0;
    B(1,0) = 0;
    Matrix<double,2,1> temp;
    temp << 0,
            1;
    // B.block(1,0,2,1) = Mass.inverse().block(0,1,2,1);
    B.block(2,0,2,1) = Mass.inverse()*temp;
    
    Matrix<double,1,4> K;
    Matrix<double,4,4> Q;
    Matrix<double,1,1> R;

    K.setZero();
    Q << 100, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    R << 1;

    // std::cout << A << std::endl;
    // std::cout << B << std::endl;
    ct::optcon::LQR<4,1> lqrsolver;
    lqrsolver.compute(Q,R,A,B,K,true,true);
    // std::cout << K << std::endl;
    // K << -3.474, -0.4655, -0.0538;
    if (t > 1)
    {
        fr = K*(rx - x);
    }
    else
    {
        fr = 0;
    }
 

}

// void car_jump::balance_control()
// {
//     modelcom();
//     // std::cout<< t <<"  "<< mc <<"  "<< Ic <<"  "<< lc <<"  "<< c_pitch_pos <<"  "<< c_pitch_vel << std::endl;
//     Matrix<double,3,1> rx;
//     rx << c_pitch_pos_ref, c_pitch_vel_ref, wheel_vel_ref;
//     Matrix<double,3,1> x;
//     x << c_pitch_pos, c_pitch_vel, wheel_vel;
    
//     Matrix<double,2,2> Mass;
//     Mass(0,0) = Ic + Ir + pow(lc,2)*mc + mc*pow(r,2) + mr*pow(r,2) + 2*lc*mc*r*cos(c_pitch_pos_ref);
//     Mass(0,1) = Ir + mr*pow(r,2) + mc*r*(r + lc*cos(c_pitch_pos_ref));
//     Mass(1,0) = Mass(0,1);
//     Mass(1,1) = Ir + mc*pow(r,2) + mr*pow(r,2);

//     Matrix<double,2,1> pbpq;
//     pbpq(0,0) = -lc*mc*cos(c_pitch_pos_ref)*(r*pow(c_pitch_vel_ref,2) + g);
//     pbpq(1,0) = -pow(c_pitch_vel_ref,2)*lc*mc*r*cos(c_pitch_pos_ref);

//     Matrix<double,2,2> pbpdq;
//     pbpdq(0,0) = -lc*mc*sin(c_pitch_pos_ref)*r*c_pitch_vel_ref*2;
//     pbpdq(1,0) = -c_pitch_vel_ref*2*lc*mc*r*sin(c_pitch_pos_ref);
//     pbpdq(0,1) = 0;
//     pbpdq(1,1) = 0;
    

//     Matrix<double,3,3> A;
//     A(0,0) = 0;
//     A(0,1) = 1;
//     A(0,0) = 0;
//     A.block(1,0,2,1) = -Mass.inverse()*pbpq;
//     A.block(1,1,2,2) = -Mass.inverse()*pbpdq;

//     Matrix<double,3,1> B;
//     B(0,0) = 0;
//     Matrix<double,2,1> temp;
//     temp << 0,
//             1;
//     // B.block(1,0,2,1) = Mass.inverse().block(0,1,2,1);
//     B.block(1,0,2,1) = Mass.inverse()*temp;
    
//     Matrix<double,1,3> K;
//     Matrix<double,3,3> Q;
//     Matrix<double,1,1> R;

//     K.setZero();
//     Q << 1, 0, 0,
//            0, 1, 0,
//            0, 0, 1;
//     R << 1;

//     // std::cout << A << std::endl;
//     // std::cout << B << std::endl;
//     ct::optcon::LQR<3,1> lqrsolver;
//     lqrsolver.compute(Q,R,A,B,K,true,true);
//     // std::cout << K << std::endl;
//     // K << -3.474, -0.4655, -0.0538;
//     if (t > 1)
//     {
//         fr = K*(rx - x);
//     }
//     else
//     {
//         fr = 0;
//     }
 

// }

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