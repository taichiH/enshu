/*
  In this demo, a robot will pick up a caffelatte
  or enter an error recovery mode when nothing is found.
*/

#include "negomo/NegomoLib2.hh"
#include "aero_devel_lib/devel_lib.hh"

aero::interface::AeroMoveitInterfacePtr robot_; // controller
aero::DevelLibPtr lib_; // development library
negomo_lib::NegomoBridge2Ptr planner_; // planner


// when entering error
int error(int _inhands, int &_nexttask) {
  ROS_ERROR("%s", planner_->getBackTrack().c_str());
  robot_->speakAsync(planner_->getBackTrack());
  return _inhands;
};


// picking action
int pick(int _inhands, int &_nexttask) {
  lib_->setFCNModel("container"); // set recognition model

  // get position of item
  Eigen::Vector3d pos;
  bool found = lib_->poseAndRecognize("container", "caffelatte", pos, -0.12);

  if (found)
    found = lib_->pickCoffeeFront(pos, 0.93);

  if (!found) {
    planner_->setBackTrack("failed coffee!");
    _nexttask = -404;
  }

  return lib_->getUsingHandsNum();
};


// main
int main(int argc, char **argv) {
  // init ros
  ros::init(argc, argv, "sample");
  ros::NodeHandle nh("~");

  // init variables
  robot_.reset(new aero::interface::AeroMoveitInterface(nh));
  lib_.reset(new aero::DevelLib(nh, robot_));
  planner_.reset(new negomo_lib::NegomoBridge2(nh, "/negomo/", nullptr));

  // create actions for planner
  negomo_lib::ActionList exceptions =
    {std::make_tuple(0, 0, error, planner_->emptyAction, "error"),
     std::make_tuple(0, 0, planner_->uiExceptionHandle, planner_->emptyAction, "onHelp")};
  negomo_lib::ActionList temps = {};
  negomo_lib::ActionList task1 =
    {std::make_tuple(0, 1, pick, planner_->emptyAction, "pick")};

  // add actions to planner
  std::vector<negomo_lib::ActionList> tasks = {task1};

  // set entity values for each task
  std::vector<std::vector<std::string> > task_entities = {{"task1","iOff=true"}};

  // init planner
  planner_->init(task_entities); // add default entities
  std::vector<int> defaultq = {0}; // default task to run
  planner_->setDefaultQueue(defaultq); // set default task

  // run planner
  planner_->run(tasks, &exceptions, &temps);
}
