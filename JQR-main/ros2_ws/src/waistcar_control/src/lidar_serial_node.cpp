#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <thread>
#include <atomic>

#include <std_msgs/msg/float32_multi_array.hpp>

int OpenSerial(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port " << device << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:    speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// Comment removed (non-English).
// Comment removed (non-English).
bool ParseLine(
    rclcpp::Logger logger,
    const std::string& line,
    float &obstacle_distance,
    float &zmin_out,
    float &zmax_out,
    int &obstacle_type)
{
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;

    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }

    if (tokens.size() != 8) {
        RCLCPP_WARN(logger, "Invalid data: '%s'", line.c_str());
        return false;
    }

// Comment removed (non-English).
    RCLCPP_INFO(logger, "Frame ID    : %s", tokens[0].c_str());
    RCLCPP_INFO(logger, "Cluster ID  : %s", tokens[1].c_str());
    RCLCPP_INFO(
        logger, "Xmin,Ymin,Zmin : %s, %s, %s",
        tokens[2].c_str(), tokens[3].c_str(), tokens[4].c_str());
    RCLCPP_INFO(
        logger, "Xmax,Ymax,Zmax : %s, %s, %s",
        tokens[5].c_str(), tokens[6].c_str(), tokens[7].c_str());
    RCLCPP_INFO(logger, "-----------------------------");

    try {
        float x    = std::stof(tokens[2]); // Comment removed (non-English).
        float zmin = std::stof(tokens[4]);
        float zmax = std::stof(tokens[7]);

        obstacle_distance = x;
        zmin_out = zmin;
        zmax_out = zmax;

        const float z_negative_thresh = -0.15f;
        const float z_positive_thresh =  0.15f;

        if (zmax > z_positive_thresh) {
            obstacle_type = 2; // Comment removed (non-English).
        } else if (zmin < z_negative_thresh) {
            obstacle_type = 1; // Comment removed (non-English).
        } else {
            obstacle_type = 0; // Comment removed (non-English).
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(logger, "Failed to convert to float: %s", e.what());
        return false;
    }

    return true;
}

class LidarSerialNode : public rclcpp::Node
{
public:
    LidarSerialNode()
    : Node("lidar_serial_node"), running_(true), fd_(-1)
    {
        this->declare_parameter<std::string>("port", "/dev/com_lidar_pi_usb_right_up");
        this->declare_parameter<int>("baudrate", 115200);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        RCLCPP_INFO(this->get_logger(), "Opening serial port: %s, baudrate: %d",
                    port_.c_str(), baudrate_);

        fd_ = OpenSerial(port_.c_str(), baudrate_);
        if (fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port, exit.");
            throw std::runtime_error("Failed to open serial port");
        }

// Comment removed (non-English).
// data[0] = type (0/1/2)
// data[1] = distance (x)
// data[2] = zmin
// data[3] = zmax
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "obstacle_info", 10);

        read_thread_ = std::thread(&LidarSerialNode::readLoop, this);
    }

    ~LidarSerialNode() override
    {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    void readLoop()
    {
        char buffer[256];
        std::string line;

        while (rclcpp::ok() && running_) {
            int n = read(fd_, buffer, sizeof(buffer) - 1);
            if (n > 0) {
                buffer[n] = '\0';
                line += buffer;

                size_t pos;
                while ((pos = line.find('\n')) != std::string::npos) {
                    std::string row = line.substr(0, pos);
                    line.erase(0, pos + 1);

                    float distance = 0.0f;
                    float zmin = 0.0f;
                    float zmax = 0.0f;
                    int type = 0;

                    if (ParseLine(this->get_logger(), row, distance, zmin, zmax, type)) {
                        std_msgs::msg::Float32MultiArray msg;
                        msg.data.resize(4);
                        msg.data[0] = static_cast<float>(type);
                        msg.data[1] = distance;
                        msg.data[2] = zmin;
                        msg.data[3] = zmax;
                        obstacle_pub_->publish(msg);
                    }
                }
            } else {
                usleep(1000);
            }
        }
    }

    std::string port_;
    int baudrate_;
    int fd_;
    std::thread read_thread_;
    std::atomic<bool> running_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<LidarSerialNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
