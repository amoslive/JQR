#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

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
        default: speed = B115200; break;
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
void ParseAndPrint(const std::string& line) {
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;

    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }

    if (tokens.size() != 8) {
        std::cerr << "Invalid data: " << line << std::endl;
        return;
    }

// Comment removed (non-English).
    std::cout << "Frame ID    : " << tokens[0] << std::endl;
    std::cout << "Cluster ID  : " << tokens[1] << std::endl;
    std::cout << "Xmin,Ymin,Zmin : "
              << tokens[2] << ", " << tokens[3] << ", " << tokens[4] << std::endl;
    std::cout << "Xmax,Ymax,Zmax : "
              << tokens[5] << ", " << tokens[6] << ", " << tokens[7] << std::endl;
    std::cout << "-----------------------------" << std::endl;
}

int main() {
    const char* serial_dev = "/dev/com_lidar_pi_usb_right_up";
    int baudrate = 115200;

    int fd = OpenSerial(serial_dev, baudrate);
    if (fd < 0) return -1;

    std::cout << "Serial port opened: " << serial_dev << std::endl;

    char buffer[256];
    std::string line;

    while (true) {
        int n = read(fd, buffer, sizeof(buffer)-1);
        if (n > 0) {
            buffer[n] = '\0';
            line += buffer;

            size_t pos;
            while ((pos = line.find('\n')) != std::string::npos) {
                std::string row = line.substr(0, pos);
                line.erase(0, pos + 1);

// Comment removed (non-English).
                ParseAndPrint(row);
            }
        } else {
            usleep(1000);
        }
    }

    close(fd);
    return 0;
}
