#include <boost/asio.hpp>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <termios.h>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "waistcar_msgs/msg/remote_msg.hpp"

using namespace boost::asio;
using waistcar_msgs::msg::RemoteMsg;

static float _CHMax = 1695.0;
static float _CHMin = 353.0;
static float _left_updown_Max = 1.0;
static float _left_side_Max = 1.0;
static float _right_updown_Max = 1.0;
static float _right_side_Max = 1.0;
static float _Rotate_Value_Max = 1.0;
static float _left_updown_Scale = 0.5 * (_CHMax - _CHMin) / _left_updown_Max;
static float _left_side_Scale = 0.5 * (_CHMax - _CHMin) / _left_side_Max;
static float _right_updown_Scale = 0.5 * (_CHMax - _CHMin) / _right_updown_Max;
static float _right_side_Scale = 0.5 * (_CHMax - _CHMin) / _right_side_Max;
static float _Rotate_Value_Scale = 0.5 * (_CHMax - _CHMin) / _Rotate_Value_Max;
static float _bt3thigh = _CHMax * 0.8;
static float _bt3thlow = _CHMax * 0.5;

enum flush_type {
  flush_receive = TCIFLUSH,
  flush_send = TCOFLUSH,
  flush_both = TCIOFLUSH
};

/// @brief Flush a serial port's buffers.
void flush_serial_port(boost::asio::serial_port &serial_port, flush_type what,
                       boost::system::error_code &error)
{
  if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what)) {
    error = boost::system::error_code();
  } else {
    error = boost::system::error_code(
        errno, boost::asio::error::get_system_category());
  }
}

boost::system::error_code init_hardware(serial_port& sp,
                                        boost::system::error_code& ec,
                                        const char* port,
                                        const rclcpp::Logger& logger)
{
  if (sp.is_open()) {
    sp.close();
    usleep(5000);
  }

  for (int i = 0; ; i++) {
    if (!sp.is_open()) {
      sp.open(port, ec);
    }
    if (ec) {
      if (ec && (i % 1000 == 0)) {
        RCLCPP_ERROR(logger, "Failed to open serial port %s: %s",
                     port, ec.message().c_str());
      }
      i++;
      usleep(5000);
    } else {
      break;
    }
    if (!rclcpp::ok()) {
      return ec;
    }
  }

  RCLCPP_INFO(logger, "Open port : %s", port);

// Configure serial port parameters.
  sp.set_option(serial_port::baud_rate(115200)); 
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none)); 
  sp.set_option(serial_port::parity(serial_port::parity::none));
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp.set_option(serial_port::character_size(8));

  flush_serial_port(sp, flush_receive, ec);
  RCLCPP_INFO(logger, "Flush port : %s", port);

  return ec;
}

// Parse one remote-control frame.
bool parseDataPacket(const uint8_t* buffer, RemoteMsg& msg)
{
// Compute XOR checksum.
  uint8_t xorCheck = 0;
  for (size_t i = 1; i < 34; ++i) {
    xorCheck ^= buffer[i];
  }

  if (xorCheck != buffer[34]) {
return false; // Checksum mismatch.
  }

float channels[16]; // Parsed channel values (16 channels).
  for (size_t i = 0; i < 16; ++i) {
    int highByte = buffer[1 + 2 * i];
    int lowByte  = buffer[2 + 2 * i];
    channels[i] = static_cast<float>((highByte << 8) | lowByte);
  }


  // msg.left_updown  = -((channels[1] - 0.5f * (_CHMax + _CHMin)) / _left_updown_Scale);
  // msg.left_side    =  ((channels[0] - 0.5f * (_CHMax + _CHMin)) / _left_side_Scale);
  // msg.right_updown = -((channels[2] - 0.5f * (_CHMax + _CHMin)) / _right_updown_Scale);
  // msg.right_side   =  ((channels[3] - 0.5f * (_CHMax + _CHMin)) / _right_side_Scale);
  // msg.left_updown  = channels[1];
  // msg.left_side    = channels[0];
  // msg.right_updown = channels[2];
  // msg.right_side   = channels[3];
  // msg.right_side = (channels[0]-749.0)/(924.0-574.0)*2;
  // msg.left_updown = (channels[1]-749.0)/(924.0-574.0)*2;
  // msg.right_updown = (channels[2]-749.0)/(999.0-499.0)*2;
  // msg.left_side = (channels[3]-749.0)/(924.0-574.0)*2;
  msg.right_side = (channels[0]-992.0)/(1552.0-432.0)*2;
  msg.left_updown = (channels[1]-992.0)/(1552.0-432.0)*2;
  msg.right_updown = (channels[2]-950.0)/(1792.0-192.0)*2;
  msg.left_side = (channels[3]-992.0)/(1552.0-432.0)*2;


  for (int k = 0; k < 4; ++k) {
    float value = channels[4 + k];
    if (value > _bt3thigh) {
      msg.buttons[k] = 3;
    } else if (value > _bt3thlow) {
      msg.buttons[k] = 2;
    } else {
      msg.buttons[k] = 1;
    }
  }

  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("remoter_read_node");
  auto logger = node->get_logger();

auto pub = node->create_publisher<RemoteMsg>("remote_topic", 10); // Topic: remote_topic

  io_service iosev;
  serial_port sp(iosev);
  boost::system::error_code ec;
  const char* port = "/dev/com_mcu_remoter";
  init_hardware(sp, ec, port, logger);

  unsigned char buff[35];

  int cnt = 0;
// bool found_header = false; // Unused legacy variable (can be removed).

  while (rclcpp::ok()) {
    try {
      read(sp, buffer(buff, 1));
      while (buff[0] != 0x0F) {
        read(sp, buffer(buff, 1));
      }
      read(sp, buffer(buff + 1, 34));

      RemoteMsg msg;
      if (parseDataPacket(buff, msg)) {
pub->publish(msg);  // Publish message.
      } else {
        RCLCPP_WARN(logger, "Invalid data packet received");
      }

      cnt++;
      if (cnt % 100 == 0) {
        RCLCPP_INFO(
          logger,
          "REMOTE:: <:%.2f, ^:%.2f, >:%.2f, v:%.2f, "
          "btn: [%d,%d,%d,%d]",
          msg.left_side, msg.left_updown, msg.right_side, msg.right_updown,
          msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3]);
      }
    } catch (boost::system::system_error& e) {
      RCLCPP_ERROR(logger, "Error reading from serial port: %s, code: %d",
                   e.what(), e.code().value());
      if (rclcpp::ok()) {
        init_hardware(sp, ec, port, logger);
      }
    }
  }

  sp.close();
  rclcpp::shutdown();
  return 0;
}
