/*
  In this demo, a robot will pick up a caffelatte
  or enter an error recovery mode when nothing is found.
*/
#define MAX 4
#include "negomo/NegomoLib2.hh"
#include "aero_devel_lib/devel_lib.hh"

aero::interface::AeroMoveitInterface::Ptr robot_; // controller
aero::DevelLibPtr lib_; // development library
negomo_lib::NegomoBridge2Ptr planner_; // planner
Eigen::Vector3d pos_;
std::vector<Eigen::Vector3d> results_buf_;
std::vector<std::vector<Eigen::Vector3d>> interaction_buf_;
std::map<std::string, std::function<void(int)> > pre_;
int interaction_index_ = 0;
int index_ = 2;

// when entering error
int error(int _inhands, int &_nexttask) {
  ROS_ERROR("%s", planner_->getBackTrack().c_str());
  lib_->speakAsync(planner_->getBackTrack());
  return _inhands;
};

int containerPose(int _inhands, int &_nexttask){
  robot_->moveTo("s_shelf_container");
  while(robot_->isMoving() && ros::ok()){
    usleep(200*1000);
  }
  lib_->lookContainerFront();
  return _inhands;
}

int shelfPose(int _inhands, int &_nexttask){
  lib_->lookShelfFront();
  robot_->moveTo("s_shelf");
  while(robot_->isMoving() && ros::ok()){
    usleep(200*1000);
  }
  return _inhands;
}

void visualizeMarker(){
  aero::Transform obj_pos;
  for(int i=0; i<results_buf_.size(); ++i){
    obj_pos = aero::Translation(results_buf_.at(i));
    lib_->features_->setMarker(obj_pos, i+5);
  }
}

int watchInteraction(int _inhands, int &_nexttask) {
  ROS_INFO("watchInteraction start: %d", results_buf_.size());
  if(interaction_index_ > 0){
    ROS_INFO("watch interaction");
    ROS_INFO("index_: %d", index_);
    std::vector<Eigen::Vector3d> tmp_vec;
    for(int i=0; i<(index_ - 1); ++i){
      tmp_vec.push_back(results_buf_.at(i));
    }
    interaction_buf_.push_back(tmp_vec);
    auto last_pos = tmp_vec.back();
    results_buf_.clear();
    results_buf_.push_back(last_pos);
  }
  ++interaction_index_;
  return _inhands;
}

int watch(int _inhands, int &_nexttask) {
  ROS_INFO("watch start: %d", results_buf_.size());
  lib_->setFCNModel("final");
  std::vector<aero_recognition_msgs::Scored2DBox>
    hand_recognition_result = lib_->recognizeHand();

  std::vector<Eigen::Vector3d> results;
  bool found = lib_->findItem("pie", results);

  const double diff_min = 0.09;

  // not first time
  if(!interaction_buf_.empty()){
    std::vector<Eigen::Vector3d> last_results_buf = interaction_buf_.back();
    for(int i=0; i<last_results_buf.size(); ++i){
      for(int j=0; j<results.size(); ++j){
        Eigen::Vector3d diff = last_results_buf.at(i) - results.at(j);
        ROS_INFO("diff.norm(): ", diff.norm());
        if(diff.norm() < diff_min){
          ROS_INFO("find new item");
          ROS_INFO("erase index: ", (results.begin() + j));
          results.erase(results.begin() + j);
        }
      }
    }
  }

  if(!hand_recognition_result.empty())
    return _inhands;

  if(!found || results.empty())
    return _inhands;

  if(results_buf_.empty()){
    results_buf_.push_back(results.at(0));
    return _inhands;
  }

  for(int i=0; i<results.size(); ++i){
    ROS_INFO("create put plan");
    Eigen::Vector3d diff = results.at(i) - results_buf_.at(0);
    if(diff.norm() > diff_min){
      results_buf_.push_back(results.at(i));
      for(int j=2; j<MAX; ++j){
        results_buf_.push_back(results_buf_.at(j-1) + diff);
      }
      ROS_INFO("done put planning");
      planner_->getEntities().put("loopCondition", false);
      break;
    }
  }
  visualizeMarker();
  return _inhands;
};

int put(int _inhands, int &_nexttask) {
  ROS_INFO("put start: %d", results_buf_.size());
  Eigen::Vector3d shelf_initial(results_buf_.at(index_).x(), results_buf_.at(index_).y(), results_buf_.at(index_).z());

  aero::Transform obj_pos;
  for(int i=0; i<results_buf_.size(); ++i){
    obj_pos = aero::Translation(results_buf_.at(i));
    lib_->features_->setMarker(obj_pos, i+5);
  }

  negomo_lib::waitSettings ws;
  ws.interaction_flag =
    negomo_enshu::PlannerBridgeRequest::Request::IGNORE;

  Eigen::Vector3d look_pos(0.65, 0.0, 0.50 );
  
  // start tracking object
  robot_->setTrackingMode(true);
  // look at target
  robot_->setLookAt(look_pos, false, true);
  // start interaction on background
  planner_->iStart(negomo_lib::jumpSettings(), ws);
  ROS_INFO("place coffee reach");
  bool found = lib_->placeCoffeeReach(shelf_initial, 0);
  if (!found) {
    planner_->setError(true, "failed put");
    _nexttask = -404;
  }
  // finish interaction on background
  planner_->iJoin(_inhands, _nexttask); // set next task

  if (_nexttask != -1)
    return lib_->getUsingHandsNum();
  ROS_INFO("open hand");
  lib_->openHand(aero::arm::rarm);
  ROS_INFO("place coffee return");
  lib_->placeCoffeeReturn();
  lib_->sendResetPose();

  // finich tracking object
  robot_->setTrackingMode(false);
  if(index_ == MAX-1){
    ROS_INFO("put loop end");
    planner_->getEntities().put("loopCondition", false);
    return lib_->getUsingHandsNum();
  }
  ++index_;
  return lib_->getUsingHandsNum();
};

int pick(int _inhands, int &_nexttask) {
  lib_->setFCNModel("final");
  // get position of item
  bool found = lib_->poseAndRecognize("container", "pie", pos_, -0.2);

  if (found)
    found = lib_->pickCoffeeFront(pos_, 0.83);
  if (!found) {
    planner_->setBackTrack("failed coffee!");
    _nexttask = -404;
  }

  lib_->sendResetPose();
  return lib_->getUsingHandsNum();
};

// pre-interactions and callback
void proactive0(int _target) {}; // not used in this demo
void proactive1(int _target) {}; // not used in this demo
void reactive0(int _target) {
  // robot_->setLookAt(Eigen::Vector3d(1.0, 0.0, 1.8), false, true, false);
  ROS_INFO("in reactive 0");
  usleep(1000 * 1000);
};
void returnPlanner(int _target) {
  // robot_->setLookAtTopic("/look_at/previous");
  // usleep(2000 * 1000);
};
bool preinteractionCb(negomo_enshu::RobotAction::Request &_req,
                      negomo_enshu::RobotAction::Response &_res) {
  ROS_INFO("looking for action %s", _req.action.c_str());
  pre_[_req.action](_req.target_id);
  return true;
}

// main
int main(int argc, char **argv) {
  // init ros
  ros::init(argc, argv, "sample");
  ros::NodeHandle nh;

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
  negomo_lib::ActionList task0 =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 1, shelfPose, planner_->emptyAction, "shelf"),
     std::make_tuple(0, 0, watchInteraction, planner_->emptyAction, "interaction"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, watch, planner_->emptyAction, "watch"),
     std::make_tuple(0, 0, planner_->loopEnd, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finish"),};
  negomo_lib::ActionList task1 =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, containerPose, planner_->emptyAction, "container"),
     std::make_tuple(0, 1, pick, planner_->emptyAction, "pick"),
     std::make_tuple(0, 1, shelfPose, planner_->emptyAction, "shelf"),
     std::make_tuple(1, 0, put, planner_->emptyAction, "put"),
     std::make_tuple(0, 0, planner_->loopEnd, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finishTask"),};

  // add actions to planner
  std::vector<negomo_lib::ActionList> tasks = {task0, task1};

  // set entity values for each task
  std::vector<std::vector<std::string> > task_entities = {{"task2","iOff=false", "loopCondition=true","priority=0"}, {"task1","iOff=false", "loopCondition=true", "index=2"}};

  // init planner
  planner_->init(task_entities); // add default entities
  std::vector<int> defaultq = {0, 1}; // default task to run
  planner_->setDefaultQueue(defaultq); // set default task

  // run planner
  planner_->run(tasks, &exceptions, &temps);
}
