/*******************************************************************************
* Copyright 201 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: JaySong, Kayman Jung */

#include "op3_action_starter/action_starter.h"
#include <signal.h>
#include <termios.h>
#include <string>


const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;

void sighandler(int sig)
{
  struct termios term;
  tcgetattr( STDIN_FILENO, &term);
  term.c_lflag |= ICANON | ECHO;
  tcsetattr( STDIN_FILENO, TCSANOW, &term);

  system("clear");
  exit(0);
}

bool turnOnDynamixelPower(const std::string &device_name, const int &baud_rate)
{
  // power on
  dynamixel::PortHandler *_port_h = (dynamixel::PortHandler *) dynamixel::PortHandler::getPortHandler(device_name.c_str());
  bool _set_port = _port_h->setBaudRate(baud_rate);
  if (_set_port == false)
  {
    ROS_ERROR("Error Set port");
    return false;
  }
  dynamixel::PacketHandler *_packet_h = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int _return = _packet_h->write1ByteTxRx(_port_h, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

  if(_return != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to turn on the Power of DXLs!");
    return false;
  }
  else
  {
    ROS_INFO("Power on DXLs!");
  }


  usleep(100 * 1000);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "THORMANG3 Action starter");
  ros::NodeHandle nh;

  std::string offset_file   = nh.param<std::string>("offset_table", "");
  std::string robot_file    = nh.param<std::string>("robot_file_path", "");
  std::string dxl_init_file = nh.param<std::string>("init_file_path", "");
  std::string _device_name = nh.param<std::string>("device_name", SUB_CONTROLLER_DEVICE);
  int _baud_rate = nh.param<int>("baudrate", BAUD_RATE);

  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGQUIT, &sighandler);
  signal(SIGINT, &sighandler);

  if(turnOnDynamixelPower(_device_name, _baud_rate) == false)
    return 0;

  robotis_op::ActionStarter starter;
  if (starter.initializeActionStarter(robot_file, dxl_init_file, offset_file) == false)
  {
    ROS_ERROR("Failed to Initialize");
    return 0;
  }

  ros::spin();

  return 0;
 }
