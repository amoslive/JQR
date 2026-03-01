#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "waistcar_msgs/msg/mode_command_msg.hpp"


#include "utils.h"

using std::placeholders::_1;


class motor_command_node : public rclcpp::Node
{
    public:
        motor_command_node() : Node("motor_command_node"){
            pub = this->create_publisher<waistcar_msgs::msg::ModeCommandMsg>("mode_cmd",1);
        }
        rclcpp::Publisher<waistcar_msgs::msg::ModeCommandMsg>::SharedPtr pub;

    private:

};

int main(int argc, char *argv[])
{

    TicToc tt;
    rclcpp::init(argc,argv);
    auto node = std::make_shared<motor_command_node>();

    rclcpp::Rate loop_rate(1000);
    waistcar_msgs::msg::ModeCommandMsg mode_cmd;

    while (rclcpp::ok())
    {
        std::string cmd;
        std::string last_cmd = "";


// std::cout << "0 for stop, 1 for slowly run mode, 2 for waist control mode:" << std::endl;
        std::cout << std::endl;
        std::cout << "0: stop!" << std::endl;
        std::cout << "1: wheel slowly run!" << std::endl;
        std::cout << "2: waist position control!" << std::endl;
        std::cout << "3: wheel torque!" << std::endl;
        std::cout << "4: balance control!" << std::endl;
        std::cout << std::endl;

        std::getline(std::cin,cmd);
        if (cmd == "0")
        {
            mode_cmd.car_mode = 0;
            mode_cmd.waist_int_clear = true;

            RCLCPP_INFO(node->get_logger(),"MODE: stop!");
        }
        else if (cmd == "1")
        {
            mode_cmd.car_mode = 1;
            mode_cmd.waist_int_clear = true;
            RCLCPP_INFO(node->get_logger(),"MODE: slowly run!");
        }
        else if (cmd == "2")
        {
// std::cout << "please input waist target pos:" << std::endl;
// std::string pos_input;
// std::getline(std::cin,pos_input);

// mode_cmd.car_mode = 2;
// mode_cmd.waist_int_clear = true;
// mode_cmd.waist_pos_ref = std::stod(pos_input);


        }
        else if (cmd == "3")
        {
// std::cout << "please input wheel target torque:" << std::endl;
// std::string torque_input;
// std::getline(std::cin,torque_input);

            mode_cmd.car_mode = 3;
            mode_cmd.waist_int_clear = true;
// mode_cmd.wheel_torque_ref = std::stod(torque_input);
        }
        else if (cmd == "4")
        {
            RCLCPP_INFO(node->get_logger(),"Balance control!");
            mode_cmd.car_mode = 4;
            mode_cmd.waist_int_clear = false;
        }


        else
        {
            cmd = last_cmd;
        }

        node->pub->publish(mode_cmd);
        last_cmd = cmd;


        loop_rate.sleep();
    }


    rclcpp::shutdown();
    return 0;

}