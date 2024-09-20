

#include "op3_action_starter/action_starter.h"

using namespace robotis_op;

ActionStarter::ActionStarter()
{
  ctrl_ = 0;
  robot_ = 0;
}

ActionStarter::~ActionStarter()
{

}

bool ActionStarter::initializeActionStarter(std::string robot_file_path, std::string init_file_path,
                                          std::string offset_file_path)
{
  ctrl_ = robotis_framework::RobotisController::getInstance();

  //Controller Initialize with robot file info
  if (ctrl_->initialize(robot_file_path, init_file_path) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return false;
  }

  ctrl_->loadOffset(offset_file_path);
  ctrl_->addMotionModule((robotis_framework::MotionModule*) ActionModule::getInstance());
  ActionModule::getInstance()->enableAllJoints();

  robot_ = ctrl_->robot_;

  //Initialize Publisher
  ros::NodeHandle nh;

  play_action_sub_ = nh.subscribe("/robotis/play_action", 10, &ActionStarter::playActionCallback, this);

  return true;
}

void ActionStarter::playActionCallback(const std_msgs::Int32 msg)
{
  if (!selectPage(msg.data))
    return;

  ctrl_->startTimer();
  ros::Duration(0.03).sleep();  // waiting for timer start

  std::string module_name = "action_module";
  ctrl_->setCtrlModule(module_name);
  ros::Duration(0.03).sleep(); // waiting for enable

  if (ActionModule::getInstance()->start(msg.data, &page_) == false)
  {
    ROS_ERROR("Failed to play this page!");
    ctrl_->setCtrlModule("none");
    ctrl_->stopTimer();
    return;
  }

  while (1)
  {
    if (ActionModule::getInstance()->isRunning() == false)
      break;

    usleep(10000);
  }

  ctrl_->setCtrlModule("none");
  ctrl_->stopTimer();
}

bool ActionStarter::selectPage(const int index)
{

  if (index > 0 && index < action_file_define::MAXNUM_PAGE)
  {
    ActionModule::getInstance()->loadPage(index, &page_);
    return true;
  }
  else
  {
    ROS_ERROR("Invalid page index");
    return false;
  }
}