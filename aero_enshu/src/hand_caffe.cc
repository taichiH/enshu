/*
  In this demo, a robot will switch between tasks;
  in task1, the robot will pick up a caffelatte and
  in task2, the robot will handover the caffelatte
  only when interrupted by a user.
*/

#include "negomo/NegomoLib2.hh"
#include "aero_devel_lib/devel_lib.hh"

aero::interface::AeroMoveitInterfacePtr robot_; // controller
aero::DevelLibPtr lib_; // development library
negomo_lib::NegomoBridge2Ptr planner_; // planner
std::map<std::string, std::function<void(int)> > pre_;


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

  // set interaction settings, don't care about hands
  negomo_lib::waitSettings ws;
  ws.interaction_flag =
    negomo_enshu::PlannerBridgeRequest::Request::IGNORE;

  robot_->setTrackingMode(true); // enable background head move
  robot_->setLookAt(pos, false, true); // look at target

  // start interaction on background
  planner_->iStart(negomo_lib::jumpSettings(), ws);

  if (found)
    found = lib_->pickCoffeeFront(pos, 0.93);
  _inhands = lib_->getUsingHandsNum(); // update hand usage

  if (!found)
    planner_->setError(true, "failed coffee!");

  // finish interaction on background
  planner_->iJoin(_inhands, _nexttask);
  robot_->setTrackingMode(false); // disable background head move

  return _inhands;
};


// pre-interactions and callback
void proactive0(int _target) {}; // not used in this demo
void proactive1(int _target) {}; // not used in this demo
void reactive0(int _target) {
  robot_->setLookAt(Eigen::Vector3d(1.0, 0.0, 1.8), false, true, false);
  usleep(2000 * 1000);
};
void returnPlanner(int _target) {
  robot_->setLookAtTopic("/look_at/previous");
  usleep(2000 * 1000);
};
bool preinteractionCb(negomo_enshu::RobotAction::Request &_req,
                      negomo_enshu::RobotAction::Response &_res) {
  ROS_INFO("looking for action %s", _req.action.c_str());
  pre_[_req.action](_req.target_id);
  return true;
}


// handover action
int handover(int _inhands, int &_nexttask) {
  usleep(1000 * 1000); // pause a while
  std::map<aero::joint, double> av;
  robot_->getResetManipPose(av);
  av.at(aero::joint::r_shoulder_r) = 0.0;
  av.at(aero::joint::r_shoulder_p) = -20.0 * M_PI / 180.0;
  av.at(aero::joint::r_shoulder_y) = 0.0;
  av.at(aero::joint::r_elbow) = -70.0 * M_PI / 180.0;
  av.at(aero::joint::r_wrist_y) = -M_PI / 2.0;
  av.at(aero::joint::r_wrist_r) = 0.0;
  robot_->setNeck(0.0, 0.0, 0.0);
  av.at(aero::joint::lifter_x) = 0.0;
  av.at(aero::joint::lifter_z) = 0.0;
  robot_->sendAngleVector
    (av, lib_->calcPathTime(av, 0.5), aero::ikrange::lifter);
  usleep(1000 * 1000); // wait handover finish
  robot_->openHand(aero::arm::rarm);
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

  // add pre-interactions
  pre_["/negomo/reset"] = returnPlanner;
  pre_["/negomo/proactive0"] = proactive0; // not used in this demo
  pre_["/negomo/proactive1"] = proactive1; // not used in this demo
  pre_["/negomo/reactive0"] = reactive0;
  // add preinteraction callback to planner
  planner_->initlib(boost::bind(&preinteractionCb, _1, _2));

  // create actions for planner
  negomo_lib::ActionList exceptions =
    {std::make_tuple(0, 0, error, planner_->emptyAction, "error"),
     std::make_tuple(0, 0, planner_->uiExceptionHandle, planner_->emptyAction, "onHelp")};
  negomo_lib::ActionList temps = {};
  negomo_lib::ActionList task1 =
    {std::make_tuple(0, 1, pick, planner_->emptyAction, "pick")};

  // create a second task
  negomo_lib::ActionList task2 =
    {std::make_tuple(1, 0, handover, planner_->emptyAction, "give")};

  // add actions to planner
  std::vector<negomo_lib::ActionList> tasks = {task1, task2};

  // set entity values for each task
  // set priority to 0 if robot should handover despite failure
  std::vector<std::vector<std::string> > task_entities = {{"task1","iOff=false"}, {"task2","priority=2"}};

  // init planner
  planner_->init(task_entities); // add default entities
  std::vector<int> defaultq = {0}; // default task to run
  planner_->setDefaultQueue(defaultq); // set default task

  // run planner
  planner_->run(tasks, &exceptions, &temps);
}
