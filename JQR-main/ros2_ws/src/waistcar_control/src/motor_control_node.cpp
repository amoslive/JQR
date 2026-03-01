#include "serialPort/SerialPort.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include "utils.h"

#include "rclcpp/rclcpp.hpp"
#include "waistcar_msgs/msg/motor_command_msg.hpp"
#include "waistcar_msgs/msg/motor_feedback_msg.hpp"

using std::placeholders::_1;

waistcar_msgs::msg::MotorFeedbackMsg motor_msg;
waistcar_msgs::msg::MotorCommandMsg motor_cmd;
MotorCmd cmd0;
// MotorCmd cmd0_protect;
MotorData data0;
double waist_base;
double wheel_base;
// origin_motor_vel/motor_gear_ratio = motor_output_axis_vel
// motor_output_axis_vel/belt_ratio = wheel_axis_vel
double belt_ratio = 0.6111; // 44/72 = 0.6111
double motor_gear_ratio = 9.1;
class motor_control_node : public rclcpp::Node
{
    public:
        motor_control_node() : Node("motor_control_node"){
            pub = this->create_publisher<waistcar_msgs::msg::MotorFeedbackMsg>("motor_msg",1);
            sub = this->create_subscription<waistcar_msgs::msg::MotorCommandMsg>("motor_cmd",1,std::bind(&motor_control_node::updatecmd,this,_1));


        };
        rclcpp::Publisher<waistcar_msgs::msg::MotorFeedbackMsg>::SharedPtr pub;
        rclcpp::Subscription<waistcar_msgs::msg::MotorCommandMsg>::SharedPtr sub;
    private:
        void updatecmd(waistcar_msgs::msg::MotorCommandMsg cmd_msg)
        {
            motor_cmd = cmd_msg;

            cmd0.mode = cmd_msg.mode0;
            cmd0.Pos = wheel_base + cmd_msg.pos0 * motor_gear_ratio * belt_ratio;
            cmd0.W = cmd_msg.vel0 * motor_gear_ratio * belt_ratio;
            cmd0.T = cmd_msg.tff0;
            cmd0.K_P = cmd_msg.kp0;
            cmd0.K_W = cmd_msg.kd0;

        }
};

int main(int argc, char *argv[])
{
    TicToc tt;

    rclcpp::init(argc,argv);
    auto node = std::make_shared<motor_control_node>();
    SerialPort serial(argv[1]);
    cmd0.motorType = MotorType::A1;
    data0.motorType = MotorType::A1;

    cmd0.id = 1;
    cmd0.mode = 0;

// cmd0_protect.id = 1;
// cmd0_protect.mode = 10;
// cmd0_protect.K_P = 0;
// cmd0_protect.K_W = 10;
// cmd0_protect.W = 0;

// cmd0_protect.T = 0;


    double i = 0;
    serial.sendRecv(&cmd0,&data0);

    wheel_base = data0.Pos;
    double wheel_maxpos = wheel_base + 3.0*2*3.14*9.1* belt_ratio;
    double wheel_minpos = wheel_base - 3.0*2*3.14*9.1* belt_ratio;



    std::chrono::time_point<std::chrono::steady_clock>wake_up_time(std::chrono::steady_clock::now());
    std::chrono::milliseconds period{1}; // 1000Hz
    while(rclcpp::ok())
    {

        i = 0;
        cmd0.T = std::max(-3.0,std::min((double)cmd0.T,3.0));

        serial.sendRecv(&cmd0, &data0);

        motor_msg.wheel_pos = -(data0.Pos - wheel_base)/ motor_gear_ratio / belt_ratio;
        motor_msg.wheel_vel = -data0.W/ motor_gear_ratio / belt_ratio;
        motor_msg.wheel_tff = data0.T;


        std::cout << "reference cmd0.T: " << cmd0.T << std::endl;
        std::cout << "acutal motor0.torque: " << motor_msg.wheel_tff << std::endl;

        node->pub->publish(motor_msg);
        rclcpp::spin_some(node);

        auto now = std::chrono::steady_clock::now();
        while(wake_up_time < now){
            wake_up_time += period;
            i++;
        }
        std::this_thread::sleep_until(wake_up_time);
    }

    cmd0.id = 0xBB;
    cmd0.mode = 0;
    cmd0.modify_data(&cmd0);
    serial.send((uint8_t *)&(cmd0.motor_send_data), 34);

    rclcpp::shutdown();
    return 0;

}