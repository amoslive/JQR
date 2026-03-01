#include <iostream>
#include <unistd.h>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "car_jump.h"
#include "utils.h"
#include "std_msgs/msg/float32_multi_array.hpp"   // obstacle_info
#include "waistcar_msgs/msg/imu_msg.hpp"
#include "waistcar_msgs/msg/remote_msg.hpp"
#include "waistcar_msgs/msg/mode_command_msg.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::vector<double> poles_real;
std::vector<double> poles_imag;
std::vector<double> obsv_Q;
std::vector<double> obsv_R;
bool enable_three_state = true;
bool info_debug = false;
double waist_pos_param = -0.01;
int cal_obsv_gain_steps = 5;
bool enable_fric_comp = false;
int ramp_mode = 0;
bool enable_stand_mode = false;
bool enable_waist_control = false;
bool enable_remote_control = false;

double pitch_dist_lpf_freq;
double whtor_dist_lpf_freq;
double wheel_vel_lpf_freq;
double launch_wheel_vel_ref;
double ext_wheel_vel_ref;
double max_pitch_dist;
double max_wheel_tor;
double ramp_detect_tor;
double ramp_state2_wait_time;
double ramp_prepare_dist;
double ramp_raise_dist;
double ramp_zero_speed_dist;
double ramp_top_dist;
double stand_back_time;
double standup_fr;
double stand_switch_angle;
double remote_max_waist_angle;
double waist_pos_cmd_lpf_freq;
double jump_waist_tor;
double jump_wheel_tor;
bool jump_back_mode;
int jump_mode;
double jump_long_waist_pos2;

class waistcar_control_node : public rclcpp::Node
{
public:
    waistcar_control_node() : Node("waistcar_control_node") {
        this->declare_parameter<std::vector<double>>("poles_real", {0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("poles_imag", {0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("obsv_Q", {0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("obsv_R", {0.0, 0.0, 0.0});        
        this->declare_parameter<bool>("enable_three_state", false);
        this->declare_parameter<bool>("info_debug", false);
        this->declare_parameter<bool>("enable_waist_control", false);
        this->declare_parameter<bool>("enable_remote_control", false);
        this->declare_parameter<double>("waist_pos_param", -0.01);
        this->declare_parameter<int>("cal_obsv_gain_steps", 5);
        this->declare_parameter<bool>("enable_fric_comp", false);
        this->declare_parameter<int>("ramp_mode", 0);
        this->declare_parameter<bool>("enable_stand_mode", false);
        this->declare_parameter<double>("pitch_dist_lpf_freq", 3.0);
        this->declare_parameter<double>("whtor_dist_lpf_freq", 50.0);
        this->declare_parameter<double>("wheel_vel_lpf_freq", 3.0);
        this->declare_parameter<double>("ext_wheel_vel_ref", 0.0);
        this->declare_parameter<double>("max_pitch_dist", 0.0);
        this->declare_parameter<double>("max_wheel_tor", 0.0);
        this->declare_parameter<double>("ramp_detect_tor", 0.0);
        this->declare_parameter<double>("ramp_state2_wait_time", 0.0);
        this->declare_parameter<double>("ramp_prepare_dist", 0.0);
        this->declare_parameter<double>("ramp_raise_dist", 0.0);
        this->declare_parameter<double>("ramp_zero_speed_dist", 0.0);
        this->declare_parameter<double>("ramp_top_dist", 0.0);
        this->declare_parameter<double>("stand_back_time", 0.0);
        this->declare_parameter<double>("standup_fr", 0.0);
        this->declare_parameter<double>("stand_switch_angle", 0.0);
        this->declare_parameter<double>("remote_max_waist_angle", 0.0);
        this->declare_parameter<double>("waist_pos_cmd_lpf_freq", 0.0);
        this->declare_parameter<double>("jump_waist_tor", 0.0);
        this->declare_parameter<double>("jump_wheel_tor", 0.0);
        this->declare_parameter<bool>("jump_back_mode", 0);
        this->declare_parameter<int>("jump_mode", 0);
        this->declare_parameter<double>("jump_long_waist_pos2", 0.0);

        poles_real = this->get_parameter("poles_real").as_double_array();
        poles_imag = this->get_parameter("poles_imag").as_double_array();
        obsv_Q = this->get_parameter("obsv_Q").as_double_array();
        obsv_R = this->get_parameter("obsv_R").as_double_array();
        enable_three_state = this->get_parameter("enable_three_state").as_bool();
        info_debug = this->get_parameter("info_debug").as_bool();
        enable_remote_control = this->get_parameter("enable_remote_control").as_bool();
        enable_waist_control = this->get_parameter("enable_waist_control").as_bool();
        waist_pos_param = this->get_parameter("waist_pos_param").as_double();
        cal_obsv_gain_steps = this->get_parameter("cal_obsv_gain_steps").as_int();
        enable_fric_comp = this->get_parameter("enable_fric_comp").as_bool();
        ramp_mode = this->get_parameter("ramp_mode").as_int();
        enable_stand_mode = this->get_parameter("enable_stand_mode").as_bool();
        pitch_dist_lpf_freq = this->get_parameter("pitch_dist_lpf_freq").as_double();
        whtor_dist_lpf_freq = this->get_parameter("whtor_dist_lpf_freq").as_double();
        wheel_vel_lpf_freq = this->get_parameter("wheel_vel_lpf_freq").as_double();
        ext_wheel_vel_ref = this->get_parameter("ext_wheel_vel_ref").as_double();
        launch_wheel_vel_ref = ext_wheel_vel_ref;
        max_wheel_tor = this->get_parameter("max_wheel_tor").as_double();
        max_pitch_dist = this->get_parameter("max_pitch_dist").as_double();
        ramp_detect_tor = this->get_parameter("ramp_detect_tor").as_double();
        ramp_state2_wait_time = this->get_parameter("ramp_state2_wait_time").as_double();
        ramp_prepare_dist = this->get_parameter("ramp_prepare_dist").as_double();
        ramp_raise_dist = this->get_parameter("ramp_raise_dist").as_double();
        ramp_zero_speed_dist = this->get_parameter("ramp_zero_speed_dist").as_double();
        ramp_top_dist = this->get_parameter("ramp_top_dist").as_double();
        stand_back_time = this->get_parameter("stand_back_time").as_double();
        standup_fr = this->get_parameter("standup_fr").as_double();
        stand_switch_angle = this->get_parameter("stand_switch_angle").as_double();
        remote_max_waist_angle = this->get_parameter("remote_max_waist_angle").as_double();
        waist_pos_cmd_lpf_freq = this->get_parameter("waist_pos_cmd_lpf_freq").as_double();
        jump_waist_tor = this->get_parameter("jump_waist_tor").as_double();
        jump_wheel_tor = this->get_parameter("jump_wheel_tor").as_double();
        jump_back_mode = this->get_parameter("jump_back_mode").as_bool();
        jump_mode = this->get_parameter("jump_mode").as_int();
        jump_long_waist_pos2 = this->get_parameter("jump_long_waist_pos2").as_double();

        // Register parameter-change callback.
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&waistcar_control_node::on_param_change, this, std::placeholders::_1)
        );

        std::cout << "enable_three_state: " << enable_three_state << std::endl;
        std::cout << "waist_pos_param: " << waist_pos_param << std::endl;

        sub_remotemsg = this->create_subscription<waistcar_msgs::msg::RemoteMsg>(
            "remote_topic", 1, std::bind(&waistcar_control_node::update_remotemsg, this, _1));
        sub_imumsg = this->create_subscription<waistcar_msgs::msg::ImuMsg>(
            "imu_msg", 1, std::bind(&waistcar_control_node::update_imumsg, this, _1));
        sub_modemsg = this->create_subscription<waistcar_msgs::msg::ModeCommandMsg>(
            "mode_cmd", 1, std::bind(&waistcar_control_node::update_modemsg, this, _1));

        pub_array = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_cmd_array", 1);
        sub_array = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "motor_msg_array", 1, std::bind(&waistcar_control_node::update_motormsg_array, this, _1));
        sub_waist_array = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "waist_motor_status_array", 1, std::bind(&waistcar_control_node::update_waist_motormsg_array, this, _1));

        // Subscribe to obstacle_info.
        sub_obstacle_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "obstacle_info",         // Must match the publisher topic name.
            10,
            std::bind(&waistcar_control_node::update_obstacle_info, this, _1)
        );

        control_frequency_ = 200.0;                   // Hz
        control_period_ = std::chrono::duration<double>(1.0 / control_frequency_); // seconds -> period duration

        RCLCPP_INFO(this->get_logger(), "Realtime Controller Node Started (%.1f Hz)", control_frequency_);

        // Create the control timer (e.g., 200 Hz).
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(control_period_),
            std::bind(&waistcar_control_node::controlCallback, this)
        );
        start_time = std::chrono::steady_clock::now();
        car.initial(1.0 / control_frequency_);
    }

    rcl_interfaces::msg::SetParametersResult on_param_change(
        const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : params) {
            if (param.get_name() == "ext_wheel_vel_ref") {
                double new_value = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated parameter 'ext_wheel_vel_ref': %.2f", new_value);
                ext_wheel_vel_ref = new_value;
            }
        }
        return result;
    }

    rclcpp::Subscription<waistcar_msgs::msg::MotorFeedbackMsg>::SharedPtr sub_motormsg;
    rclcpp::Subscription<waistcar_msgs::msg::ImuMsg>::SharedPtr sub_imumsg;
    rclcpp::Subscription<waistcar_msgs::msg::RemoteMsg>::SharedPtr sub_remotemsg;
    rclcpp::Subscription<waistcar_msgs::msg::ModeCommandMsg>::SharedPtr sub_modemsg;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_array;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_array;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_waist_array;

    // Subscribe to obstacle_info.
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_obstacle_;

    // Store obstacle information.
    int obstacle_type_ = 0;       // 0 none, 1 negative obstacle, 2 positive obstacle
    double obstacle_distance_ = 0.0;   // x distance
    double obstacle_zmin_ = 0.0;       // zmin
    double obstacle_zmax_ = 0.0;       // zmax

    double control_frequency_;
    std::chrono::duration<double> control_period_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point start_time;
    double real_time;
    double imu_update_time;
    car_jump car;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

private:

    void controlCallback()
    {
        auto now = std::chrono::steady_clock::now();
        real_time = std::chrono::duration<double>(now - start_time).count();
        car.t = real_time;
        car.run();

        if (car.mode_msg.car_mode == 0) {
            car.motor_cmd_array.mode0 = 0;
            car.motor_cmd_array.mode2 = 0; 
        } else if (car.mode_msg.car_mode == 1) {
            car.motor_cmd_array.mode0 = 5;
            car.motor_cmd_array.mode2 = 7;
            car.motor_cmd_array.tff2 = 0;
            car.motor_cmd_array.pos2 = 0;
            car.motor_cmd_array.vel2 = 0;
        } else if (car.mode_msg.car_mode == 2) {
            car.motor_cmd_array.mode0 = 0;
            car.motor_cmd_array.mode2 = 7;
            car.motor_cmd_array.tff2 = 0;
            car.motor_cmd_array.pos2 = 0;
            car.motor_cmd_array.vel2 = 0;
        } else if (car.mode_msg.car_mode == 3) {
            car.motor_cmd_array.mode0 = 10;
            car.motor_cmd_array.tff0 = 0;
            car.motor_cmd_array.pos0 = 0;
            car.motor_cmd_array.vel0 = 0;
            car.motor_cmd_array.kp0 = 0.2;
            car.motor_cmd_array.kd0 = 3.0;
        } else if (car.mode_msg.car_mode == 4) {
            car.motor_cmd_array.mode0 = 10;
            car.motor_cmd_array.tff0 = car.fr / car.belt_ratio /  car.motor_gear_ratio;
            car.motor_cmd_array.pos0 = 0;
            car.motor_cmd_array.vel0 = 0;
            car.motor_cmd_array.kp0 = 0;
            car.motor_cmd_array.kd0 = 0;

            // Waist motor
            car.motor_cmd_array.mode2 = car.waist_mode_cmd;
            car.motor_cmd_array.tff2 = car.waist_tor_cmd;
            car.motor_cmd_array.pos2 = car.waist_pos_cmd;
            car.motor_cmd_array.vel2 = 0;
        } else if (car.mode_msg.car_mode == 5) {
            car.motor_cmd_array.mode0 = 0;
            car.motor_cmd_array.tff0 = 0;
            car.motor_cmd_array.pos0 = 0;
            car.motor_cmd_array.vel0 = 0;
            car.motor_cmd_array.kp0 = 0;
            car.motor_cmd_array.kd0 = 0;
            car.motor_cmd_array.mode2 = 7;
            car.motor_cmd_array.tff2 = 0;
            car.motor_cmd_array.pos2 = car.waist_pos_cmd;
            car.motor_cmd_array.vel2 = 0;
        } else {
            car.motor_cmd_array.mode0 = 0;
            car.motor_cmd_array.mode2 = 0; 
        }

        std_msgs::msg::Float64MultiArray cmd_array;
        cmd_array.data = {
            static_cast<double>(car.motor_cmd_array.mode0),
            car.motor_cmd_array.pos0,
            car.motor_cmd_array.vel0,
            car.motor_cmd_array.tff0,
            car.motor_cmd_array.kp0,
            car.motor_cmd_array.kd0,
            static_cast<double>(car.motor_cmd_array.mode2),
            car.motor_cmd_array.pos2,
            car.motor_cmd_array.vel2,
            car.motor_cmd_array.tff2,
            car.motor_cmd_array.kp2,
            car.motor_cmd_array.kd2,
        };
        this->pub_array->publish(cmd_array);
    }

    void update_imumsg(waistcar_msgs::msg::ImuMsg imu_fdmsg) {
        car.imu_msg = imu_fdmsg;
        auto now = std::chrono::steady_clock::now();
        imu_update_time = std::chrono::duration<double>(now - start_time).count();
        car.imu_update_time = imu_update_time;
    }

    void update_remotemsg(waistcar_msgs::msg::RemoteMsg remote_fdmsg) {
        car.remote_msg = remote_fdmsg;
        waistcar_msgs::msg::ModeCommandMsg fake_mode_msg;
        if (remote_fdmsg.buttons[0] == 3) {
            fake_mode_msg.car_mode = 4;
        } else if (remote_fdmsg.buttons[0] == 2) {
            fake_mode_msg.car_mode = 5;
        } else {
            fake_mode_msg.car_mode = 0;
        }
// --- Remote control mapping / feature toggles ---------------------------------
        // Right-most switch = 1: button2 toggles long-jump; button3 toggles high-jump.
        // Right-most switch = 2: button2 toggles uphill; button3 toggles downhill.
        // Right-most switch = 3: button2 toggles stand-up routine.


        // Right-most switch = 1: button2 toggles long-jump; button3 toggles high-jump.
         if(remote_fdmsg.buttons[3] == 1){
            // button2 up + button3 up: disable long/high jump.
            if (remote_fdmsg.buttons[1] == 1 && remote_fdmsg.buttons[2] == 1)  {
                jump_mode = 0;
            }
            // button2 down + button3 up: long jump.
            else if (remote_fdmsg.buttons[1] == 3 &&  remote_fdmsg.buttons[2] == 1) {
                jump_mode = 1;
            }
            // button2 up + button3 down: high jump.
            else if(remote_fdmsg.buttons[1] == 1 &&  remote_fdmsg.buttons[2] == 3) {
                jump_mode = 2;
            }
        }
        // // Right-most switch = 2: reserved for 4-wheel driving (future use).
        // if(remote_fdmsg.buttons[3] == 2){
        //     // enable_waist_control = true;
        //     // waist_pos_param = 0.0;
        //     // // button2 up + button3 up: disable slope detection.
        //     // if (remote_fdmsg.buttons[1] == 1 && remote_fdmsg.buttons[2] == 1)  {
        //     //     ramp_mode = 0;
        //     //     ext_wheel_vel_ref = -8.0;
        //     // }
        //     // // button2 down + button3 up: uphill mode.
        //     // else if (remote_fdmsg.buttons[1] == 3 &&  remote_fdmsg.buttons[2] == 1) {
        //     //     ramp_mode = 1;
        //     //     ext_wheel_vel_ref = -8.0;
        //     // }
        //     // // button2 up + button3 down: downhill mode.
        //     // else if(remote_fdmsg.buttons[1] == 1 &&  remote_fdmsg.buttons[2] == 3) {
        //     //     ramp_mode = 2;
        //     //     ext_wheel_vel_ref = 8.0;
        //     // }
        // }
        // // Right-most switch = 3: currently unused.
        // if(remote_fdmsg.buttons[3] == 3){
        //     // enable_waist_control = false;
        //     // waist_pos_param = -2.35;

        //     // if (remote_fdmsg.buttons[1] == 1)  {
        //     // enable_stand_mode = false;
        //     // }
        //     // else if (remote_fdmsg.buttons[1] == 3) {
        //     // enable_stand_mode = true;
        //     // }
        // }
// --- End remote control mapping --------------------------------------------------

        update_modemsg(fake_mode_msg);
        if (enable_remote_control) {
            ext_wheel_vel_ref = car.remote_msg.left_updown * launch_wheel_vel_ref;
        }
    }

    void update_modemsg(waistcar_msgs::msg::ModeCommandMsg mode_fdmsg) {
        car.last_mode_msg = car.mode_msg;
        car.mode_msg = mode_fdmsg;
        if (car.last_mode_msg.car_mode != 4  && car.mode_msg.car_mode == 4) {
            car.wheel_pos_offset = car.motor_msg_array.wheel_pos;
            car.wheel_pos = 0.0;
            car.est_pitch_dist = 0.0;
            car.est_whtor_dist = 0.0;
            car.last_est_whtor_dist = 0.0;
            car.pitch_dist_filter.reset();
            car.whtor_dist_filter.reset();
            car.est_state(0) = car.c_pitch_pos;
            car.est_state(1) = car.c_pitch_vel;
            car.est_state(2) = car.wheel_vel;
            car.est_state(3) = 0.0;
            car.est_state(4) = 0.0;
            car.control_start_time = car.t;
            car.control_time = 0.0;
            car.ecbc_start_time = car.t;
            car.ecbc_time = 0.0;
            car.last_output = 0.0;
            car.ct_cnt = 0;
            car.ramp_state = 0;
        }
        if (mode_fdmsg.waist_int_clear) {
            car.waist_int = 0;
        }
    }

    // Receive obstacle_info: [type, distance, zmin, zmax]
    void update_obstacle_info(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "obstacle_info data size < 4");
            return;
        }

        obstacle_type_     = static_cast<int>(msg->data[0]);
        obstacle_distance_ = msg->data[1];
        obstacle_zmin_     = msg->data[2];
        obstacle_zmax_     = msg->data[3];
        car.obstacle_distance = obstacle_distance_;
        car.obstacle_zmin = obstacle_zmin_;
        car.obstacle_zmax = obstacle_zmax_;

        RCLCPP_INFO(this->get_logger(),
                    "Obstacle type: %d, dist: %.3f, zmin: %.3f, zmax: %.3f",
                    obstacle_type_, obstacle_distance_, obstacle_zmin_, obstacle_zmax_);

        // If the right-most switch is in the lowest position, trigger a long jump when a negative obstacle is detected with depth in [-0.3, -0.1].
        if (car.remote_msg.buttons[3] == 3 && obstacle_type_ == 1 && obstacle_zmin_ > -0.4 && obstacle_distance_ < 0.5) {
            car.jump_long_flag = 1;
        }
    }

    void update_motormsg_array(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            car.motor_msg_array.wheel_pos = msg->data[0] / car.motor_gear_ratio / car.belt_ratio;
            car.motor_msg_array.wheel_vel = msg->data[1] / car.motor_gear_ratio / car.belt_ratio;
            car.motor_msg_array.wheel_tff = msg->data[2] * car.motor_gear_ratio * car.belt_ratio;
        }
    }

    void update_waist_motormsg_array(std_msgs::msg::Float64MultiArray::SharedPtr waist_msg){
        if (waist_msg->data.size() >= 3) {
            car.motor_msg_array.waist_pos = waist_msg->data[0]; 
            car.motor_msg_array.waist_vel = waist_msg->data[1];
            car.motor_msg_array.waist_tff = waist_msg->data[2];
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<waistcar_control_node>());
    rclcpp::shutdown();
    return 0;
}
