#pragma once
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <libserial/SerialPort.h>

// #include "rclcpp/rclcpp.hpp"
#include "marvin/hardware_interface.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

std::vector<std::string> split(const std::string& input, char delimiter) {
    std::vector<std::string> answer;
    std::stringstream ss(input);
    std::string temp; 
    while (std::getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }
    return answer;
}

class SerialPort
{
  LibSerial::SerialPort serial;
  int32_t timeout_ms_;
  std::tuple<int, int, int, int> init_encoder = {0, 0, 0, 0};
public:
  void connect(const std::string& port, int baud_rate, int timeout_ms) {
    RCLCPP_INFO(
      rclcpp::get_logger("SerialPort"),
      "Port: %s, Baudrate: %d", 
      port.c_str(), baud_rate);
    timeout_ms_ = timeout_ms;
    serial.Open(port.c_str());
    serial.SetBaudRate(convert_baud_rate(baud_rate));
    // init_encoder = get_encoder_value();
  }

  void disconnect() {
    serial.Close();
  }

  bool connected() {
    return serial.IsOpen();
  }

  void send_msg(const std::string &msg_to_send)
  {
    serial.FlushIOBuffers(); // Just in case
    serial.Write(msg_to_send);
  }

  std::string read_msg(char line_terminator = '\r')
  {
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial.ReadLine(response, line_terminator, timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }
    return response;
  }

  void read_encoder_values(int &val_1, int &val_2){
    // Request main data from the controller
    PutMdData(REQUEST_PNT_MAIN_DATA, MID_MDUI, NULL, 0);

    // Receive and process data from the controller
    ReceiveDataFromController();

    // Assuming the received data is stored in a global variable of type PID_PNT_MAIN_DATA_t
    extern PID_PNT_MAIN_DATA_t received_data;

    // Get the motor positions
    val_1 = received_data.mtr_pos_id1;
    val_2 = received_data.mtr_pos_id2;
  }

  // -1000 ~ 1000 %
  void set_motor_values(int val_1, int val_2){
    PID_PNT_VEL_CMD_t motor_values;
    motor_values.enable_id1 = 1; // Assuming motor 1 is always enabled
    motor_values.rpm_id1 = val_1; // Set RPM for motor 1
    motor_values.enable_id2 = 1; // Assuming motor 2 is always enabled
    motor_values.rpm_id2 = val_2; // Set RPM for motor 2
    motor_values.req_monitor_id = 0; // Assuming no monitor request

    // Send the motor values to the controller
    PutMdData(PID_PNT_VEL_CMD, MID_MDUI, (uint8_t*)&motor_values, sizeof(motor_values));
  }

  void set_max_motor_rpm(int v1, int v2) {
    std::stringstream ss;
    ss << "^MMOD 1 1_^MMOD 2 1_^MXRPM 1 " << v1 << "_^MXRPM 2 " << v2 << char(0x0d);
    send_msg(ss.str());
    auto response = read_msg();
  }
};
