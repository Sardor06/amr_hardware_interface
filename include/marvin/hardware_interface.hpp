// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MARVIN__DIFFBOT_SYSTEM_HPP_
#define MARVIN__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "marvin/visibility_control.h"
#include "marvin/serial_comms.hpp"
#include "marvin/wheel.hpp"
#include "marvin/data_structures.hpp"
#include "marvin/hardware_interface.hpp"
#include "marvin/global.hpp"

#define MD_PROTOCOL_POS_PID             3
#define MD_PROTOCOL_POS_DATA_LEN        4
#define MD_PROTOCOL_POS_DATA_START      5

PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;
PID_GAIN_t curr_pid_gain;

uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

extern INIT_SETTING_STATE_t fgInitsetting;
/*
extern int CalRobotPoseFromRPM(PID_PNT_MAIN_DATA_t *pData);
extern void MakeMDRobotMessage1(PID_PNT_MAIN_DATA_t *pData);
extern void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData);
extern void PubRobotPose(void);
extern void PubMDRobotMessage2();
extern void PubMDRobotMessage1();
*/

SerialPort comms_;

uint8_t CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint8_t sum;

    sum = 0;
    for(int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum

    return sum;
}

int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length)
{
    uint8_t *p;
    uint16_t len;

    len = 0;
    serial_comm_snd_buff[len++] = rmid;
    serial_comm_snd_buff[len++] = robotParamData.nIDPC;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = (uint8_t)pid;
    serial_comm_snd_buff[len++] = length;

    p = (uint8_t *)&serial_comm_snd_buff[len];
    memcpy((char *)p, (char *)pData, length);
    len += length;

    serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff, len);

#if 0
    {
        int i;

        for(i = 0; i < len; i++) {
            printf("%d ", serial_comm_snd_buff[i]);
        }
        printf("\r\n");
    }
#endif

    if(comms_.connected() == true)
    {
        std::string data_to_send(reinterpret_cast<char*>(serial_comm_snd_buff), sizeof(serial_comm_snd_buff));
        comms_.Write(data_to_send);
        // comms_.Write(serial_comm_snd_buff);
    }
    return 1;
}

int MdReceiveProc() //save the identified serial data to defined variable according to PID NUMBER data
{
    uint8_t *pRcvBuf;
    uint8_t *pRcvData;
    uint8_t byRcvPID;
    uint8_t byRcvDataSize;
    bool pubmd1_flag=0;
    bool pubmd2_flag=0;

    pRcvBuf = serial_comm_rcv_buff;

    byRcvPID      = pRcvBuf[MD_PROTOCOL_POS_PID];
    byRcvDataSize = pRcvBuf[MD_PROTOCOL_POS_DATA_LEN];
    pRcvData      = &pRcvBuf[MD_PROTOCOL_POS_DATA_START];

    switch(byRcvPID)
    {
        case PID_GAIN:
        {
            if(byRcvDataSize == sizeof(PID_GAIN_t)) {
                memcpy((char *)&curr_pid_gain, (char *)pRcvData, sizeof(PID_GAIN_t));

#if 0
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gain 1:%d", curr_pid_gain.position_proportion_gain);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gain 2:%d", curr_pid_gain.speed_proportion_gain);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gain 3:%d", curr_pid_gain.integral_gain);
#endif
            }
            break;
        }

        case PID_PNT_MAIN_DATA: // 210
        {
            if(fgInitsetting != INIT_SETTING_STATE_OK) {
                break;
            }

            if(byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
                memcpy((char *)&curr_pid_pnt_main_data, (char *)pRcvData, sizeof(PID_PNT_MAIN_DATA_t));

                MakeMDRobotMessage1(&curr_pid_pnt_main_data);

                // PubMDRobotMessage1(); // md_robot_message1을 publish하는 함수 main.cpp에 정의되어 있음
            }
            break;
        }

        case PID_ROBOT_PARAM:  // 247
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
                PID_ROBOT_PARAM_t *p;

                p = (PID_ROBOT_PARAM_t *)pRcvData;
                robotParamData.sSetDia      = p->nDiameter;             // mm unit
                robotParamData.sSetWheelLen = p->nWheelLength;          // mm unit
                robotParamData.sSetGear     = p->nGearRatio;
            }
            break;
        }

        case PID_ROBOT_MONITOR2:        // 224
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR2_t)) {
                memcpy((char *)&curr_pid_robot_monitor2, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR2_t));
#if 0
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.sVoltIn);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.byUS1);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.byUS2);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.byUS3);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.byUS4);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.byPlatStatus);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.byDockingStatus);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", curr_pid_robot_monitor2.head_motor_speed_ctrl_signal);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "");
#endif                
            }
            break;
        }

        case PID_PNT_IO_MONITOR:        // 241
        {
            if(byRcvDataSize == sizeof(PID_PNT_IO_MONITOR_t)) {
                memcpy((char *)&curr_pid_pnt_io_monitor, (char *)pRcvData, sizeof(PID_PNT_IO_MONITOR_t));
            }
            break;
        }

        case PID_ROBOT_MONITOR:        // 253
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR_t)) {
                memcpy((char *)&curr_pid_robot_monitor, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR_t));

#if 0
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.lTempPosi_x);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.lTempPosi_y);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.sTempTheta);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.battery_percent);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.byUS1);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.byUS2);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.byUS3);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.byUS4);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0x%x", (uint8_t)curr_pid_robot_monitor.byPlatStatus.val);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.linear_velocity);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", curr_pid_robot_monitor.angular_velocity);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " ");
#endif                

                if(robotParamData.use_MDUI == 1) {  // If using MDUI
                    MakeMDRobotMessage2(&curr_pid_robot_monitor);

                    // PubMDRobotMessage2();
                }
            }
            break;
        }
    }
    return 1;
}

int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum) //Analyze the communication data
{
    // ros::NodeHandle n;
    rclcpp::Time stamp;
    uint8_t i, j;
    uint8_t data;
    static uint8_t byChkSec;
    static long lExStampSec, lExStampNsec;
    static uint32_t byPacketNum;
    static uint32_t rcv_step;
    static uint8_t byChkSum;
    static uint16_t byMaxDataNum;
    static uint16_t byDataNum;

    if(byPacketNum >= MAX_PACKET_SIZE)
    {
        rcv_step = 0;
        byPacketNum = 0;


        return 0;
    }
    
    for(j = 0; j < byBufNum; j++)
    {
        data = byArray[j];
#if 0
        printf("%02x(%d) ", data, data);
#endif
        switch(rcv_step) {
            case 0:    //Put the reading machin id after checking the data
                if(data == robotParamData.nIDPC)
                {
                    byPacketNum = 0;
                    byChkSum = data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    byPacketNum = 0;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error.ser: %s, %d", __FILE__, __LINE__);
                }
                break;
            case 1:    //Put the transmitting machin id after checking the data
                if((data == robotParamData.nIDMDUI) || (data == robotParamData.nIDMDT))
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error.ser: %s, %d", __FILE__, __LINE__);
                }
                break;

            case 2:    //Check ID
                if(data == 1 || data == ID_ALL)
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error.ser: %s, %d", __FILE__, __LINE__);
                }
                break;
             case 3:    //Put the PID number into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
                break;

             case 4:    //Put the DATANUM into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                byMaxDataNum = data;
                byDataNum = 0;

                rcv_step++;
                break;

             case 5:    //Put the DATA into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(++byDataNum >= MAX_DATA_SIZE)
                {
                    rcv_step = 0;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error.ser: %s, %d", __FILE__, __LINE__);
                    break;
                }

                if(byDataNum >= byMaxDataNum) {
                    rcv_step++;
                }
                break;

             case 6:    //Put the check sum after Checking checksum
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(byChkSum == 0)
                {
                    MdReceiveProc();                                 //save the identified serial data to defined variable
                }
                else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error.ser: %s, %d", __FILE__, __LINE__);
                }

                byPacketNum = 0;

                rcv_step = 0;
                break;

            default:
                rcv_step = 0;
                break;
        }
    }
    return 1;
}

int ReceiveDataFromController() //Analyze the communication data
{
    // uint8_t byRcvBuf[250];
    string byRcvBuf;
    uint8_t byBufNumber;

    static uint8_t tempBuffer[250];
    static uint8_t tempLength;

    byBufNumber = comms_.IsDataAvailable();
    if(byBufNumber != 0)
    {
        if(byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        // ser.Read(byRcvBuf, byBufNumber);
        comms_.ReadLine(byRcvBuf, '\r');
        // uint8_t byRcvBuf_2[250] = stoi(byRcvBuf);
        std::string token;
        std::string tokens[250];
        int tokenCount = 0;
        for (char c : byRcvBuf)
        {
            if(c==' ' || c== '\t' || c=='\n' || c=='\r')
            {
                if(!token.empty())
                {
                    tokens[tokenCount++]=token;
                    token.clear();
                }
            }
            else
            {
                token += c;
            }
        }
        if(!token.empty())
        {
            tokens[tokenCount++]=token;
        }
        uint8_t uint8_array[250];
        for (size_t i = 0; i < 4; ++i) {
        try {
            // Try to convert the string to uint8_t
            int value = std::stoi(tokens[i]);

            // Check if the value is within the valid range for uint8_t
            if (value >= 0 && value <= 255) {
                uint8_array[i] = static_cast<uint8_t>(value);
            } else {
                // Handle the case where the value is out of range (e.g., set to 0 or 255)
                if (value < 0) {
                    uint8_array[i] = 0;
                } else {
                    uint8_array[i] = 255;
                }
            }
        } catch (const std::invalid_argument& e) {
            // Handle the case where the conversion fails (e.g., non-integer string)
            std::cerr << "Conversion error: " << e.what() << std::endl;
        }
    }

        
        memcpy(tempBuffer, uint8_array, byBufNumber);
        tempLength = byBufNumber;

        AnalyzeReceivedData(tempBuffer, tempLength);
    }

    return 1;
}


namespace marvin
{
class MarvinHardware : public hardware_interface::SystemInterface
{

struct Config 
{
  int use_MDUI;
  int nIDPC;
  int nIDMDUI;
  int nIDMDT;
  int nBaudrate;
  int nDiameter;
  double wheel_radius;
  double nWheelLength;
  int nRMID;
  int nGearRatio;
  int reverse_direction;
  int motor_position_type;
  int encoder_PPR;
  int nMaxRPM;
  int position_proportion_gain;
  int speed_proportion_gain;
  int integral_gain;
  int nSlowstart;
  int nSlowdown;
  int motor_pole;
  std::string device;
  int timeout_ms;
  uint16_t sSetDia;
  uint16_t sSetWheelLen;
  uint16_t sSetGear;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware);

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MARVIN_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MARVIN_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MARVIN_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // SerialPort comms_; // to be changed
  Config robotParamData;

  // Store the commands 
  Wheel wheel_l_;
  Wheel wheel_r_;

};

}  // namespace marvin

#endif  // MARVIN__DIFFBOT_SYSTEM_HPP_