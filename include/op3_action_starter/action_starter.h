
#ifndef OP3_ACTION_STARTER_H_
#define OP3_ACTION_STARTER_H_

#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "robotis_controller/robotis_controller.h"
#include "op3_action_module/action_module.h"
#include "op3_base_module/base_module.h"

#define ROBOT_NAME "OP3"

namespace robotis_op
{

class ActionStarter
{
public:


  ActionStarter();
  ~ActionStarter();

  bool initializeActionStarter(std::string robot_file_path, std::string init_file_path, std::string offset_file_path);

  void playActionCallback(const std_msgs::Int32 msg);
  bool selectPage(const int index);

private:
 
  ros::Subscriber play_action_sub_;

  action_file_define::Page page_;
 
  robotis_framework::RobotisController* ctrl_;
  robotis_framework::Robot* robot_;
};

}

#endif /* OP3_ACTION_STARTER_H_ */
