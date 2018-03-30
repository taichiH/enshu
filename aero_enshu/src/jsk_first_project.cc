/*
  In this demo, a robot will pick up a caffelatte
  or enter an error recovery mode when nothing is found.
*/

#define MAX 5

#include "negomo/NegomoLib2.hh"
#include "aero_devel_lib/devel_lib.hh"

aero::interface::AeroMoveitInterface::Ptr robot_; // controller
aero::DevelLibPtr lib_; // development library
negomo_lib::NegomoBridge2Ptr planner_; // planner
Eigen::Vector3d pos_;
std::vector<Eigen::Vector3d> results_buf_;
int index_;

// when entering error
int error(int _inhands, int &_nexttask) {
  ROS_ERROR("%s", planner_->getBackTrack().c_str());
  lib_->speakAsync(planner_->getBackTrack());
  return _inhands;
};

int resetPose(int _inhands, int &_nexttask){
  lib_->lookContainerFront();
  return _inhands;
}

int watch(int _inhands, int &_nexttask) {
  lib_->setFCNModel("final"); // set recognition model
  std::vector<Eigen::Vector3d> results;
  bool found = lib_->findItem("pie", results);
  ROS_INFO("finished");
  if(!found){
    ROS_INFO("not found");
    return _inhands;
  }
  ROS_INFO("hoge hoge");
  if(results_buf_.empty()){
    results_buf_.push_back(results.at(0));
    return _inhands;
  }
  ROS_INFO("piyo piyo");

  const double diff_min = 0.08;
  const double diff_max = 0.13;
  for(int i=0; i<results.size(); ++i){
    Eigen::Vector3d diff = results.at(i) - results_buf_.at(0);
    if(diff.norm() > diff_min && diff.norm() < diff_max){
      results_buf_.push_back(results.at(i));
      for(int j=2; j<MAX; ++j){
        results_buf_.push_back(results_buf_.at(j-1) + diff);
      }
      planner_->getEntities().put("loopCondition", false);
      break;
    }
  }
  ROS_INFO("fuga fuga");
  return _inhands;
};

int put(int _inhands, int &_nexttask) {
  int index = planner_->getEntities().get<int>("index", 2);
  bool found = lib_->placeCoffee(Eigen::Vector3d(results_buf_.at(index).x(), results_buf_.at(index).y(), results_buf_.at(index).z()), 0);
  if(index == MAX-1){
    return lib_->getUsingHandsNum();
  }
  planner_->getEntities().put("index", index+1);
  return lib_->getUsingHandsNum();
};

// picking action
int pick(int _inhands, int &_nexttask) {
  // lib_->setFCNModel("container"); // set recognition model
  lib_->setFCNModel("final"); // set recognition model

  // get position of item
  // Eigen::Vector3d pos;
  // bool found = lib_->poseAndRecognize("container", "caffelatte", pos, -0.12);
  bool found = lib_->poseAndRecognize("container", "pie", pos_, -0.12);

  if (found)
    found = lib_->pickCoffeeFront(pos_, 0.83);

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
  ros::NodeHandle nh;

  // init variables
  robot_.reset(new aero::interface::AeroMoveitInterface(nh));
  lib_.reset(new aero::DevelLib(nh, robot_));
  planner_.reset(new negomo_lib::NegomoBridge2(nh, "/negomo/", nullptr));
  
  // create actions for planner
  negomo_lib::ActionList exceptions =
    {std::make_tuple(0, 0, error, planner_->emptyAction, "error"),
     std::make_tuple(0, 0, planner_->uiExceptionHandle, planner_->emptyAction, "onHelp")};
  negomo_lib::ActionList temps = {};
  negomo_lib::ActionList task0 =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0, resetPose, planner_->emptyAction, "reset"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, watch, planner_->emptyAction, "watch"),
     std::make_tuple(0, 0, planner_->loopEnd, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finish"),};
  negomo_lib::ActionList task1 =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     std::make_tuple(0, 1, pick, planner_->emptyAction, "pick"),
     std::make_tuple(1, 0, put, planner_->emptyAction, "put"),
     std::make_tuple(0, 0, planner_->loopEnd, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finishTask"),};

  
  // add actions to planner
  std::vector<negomo_lib::ActionList> tasks = {task0, task1};

  // set entity values for each task
  std::vector<std::vector<std::string> > task_entities = {{"task0","iOff=true", "loopCondition=true"}, {"task1","iOff=true", "loopCondition=true", "index=2"}};

  // init planner
  planner_->init(task_entities); // add default entities
  std::vector<int> defaultq = {0, 1}; // default task to run
  planner_->setDefaultQueue(defaultq); // set default task

  // run planner
  planner_->run(tasks, &exceptions, &temps);
}
