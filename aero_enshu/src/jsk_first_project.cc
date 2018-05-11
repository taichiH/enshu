/*
  In this demo, a robot will pick up a pie
  and wait for human instraction. When robot is 
  instracted enter an interactive mode.
*/
#define MAX 4
#define FEATURES_VARIABLE 7
#include "negomo/NegomoLib2.hh"
#include "aero_devel_lib/devel_lib.hh"

aero::interface::AeroMoveitInterface::Ptr robot_; // controller
aero::DevelLibPtr lib_; // development library
negomo_lib::NegomoBridge2Ptr planner_; // planner
Eigen::Vector3d pos_;
std::vector<Eigen::Vector3d> pre_results_;
std::vector<Eigen::Vector3d> results_buf_;
std::vector<std::vector<Eigen::Vector3d>> interaction_buf_;
std::map<std::string, std::function<void(int)> > pre_;
int interaction_index_ = 0;
int index_ = 2;
int watch_index_ = 0;
const double diff_min = 0.15;
const double diff_max = 0.30;
const double put_pos_y_offset = -0.05;
const double put_pos_z_offset = 0.05;

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
  ROS_INFO("start shelfPose");
  lib_->lookShelfFront(-0.1);
  robot_->moveTo("s_shelf");
  while(robot_->isMoving() && ros::ok()){
    usleep(200*1000);
  }
  lib_->adjustShelfArMarker();
  robot_->moveTo("s_shelf");
  while(robot_->isMoving() && ros::ok()){
    usleep(1000*1000);
  }
  return _inhands;
}

int watchInteraction(int _inhands, int &_nexttask) {
  // ignore first interaction
  if(interaction_index_ > 0){
    std::vector<Eigen::Vector3d> tmp_vec;
    for(int i=0; i<index_; ++i){
      tmp_vec.push_back(results_buf_.at(i));
    }
    interaction_buf_.push_back(tmp_vec);
    results_buf_.clear();
    results_buf_.push_back(tmp_vec.back());
  }
  ++interaction_index_;
  return _inhands;
}

int watchOnce(int _inhands, int &_nexttask) {
  lib_->setFCNModel("final");
  bool found = lib_->findItem("pie", pre_results_);
  return _inhands;
}

int watch(int _inhands, int &_nexttask) {
  ROS_INFO("watch ------ ");
  lib_->setFCNModel("final");
  std::vector<Eigen::Vector3d> results;
  bool found = lib_->findItem("pie", results);

  bool new_pos = true;
  if(!interaction_buf_.empty())
    new_pos = lib_->getNewPutPos(results, pre_results_);

  if(!found || !new_pos){
    ROS_INFO("keep watching --- ");
    return _inhands;
  }

  if(results_buf_.empty()){
    ROS_INFO("results_buf_.empty()");
    results_buf_.push_back(results.at(0));
    return _inhands;
  }

  ROS_INFO("----- debug -----");
  for(result : results){
    ROS_INFO("result: ", result.x(), result.y(), result.z());
  }
  ROS_INFO("----- debug -----");

  for(int i=0; i<results.size(); ++i){
    Eigen::Vector3d diff = results.at(i) - results_buf_.at(0);
    ROS_INFO("diff.norm: %f", diff.norm());

    if((diff.norm() > diff_min) && (diff.norm() < diff_max) && (interaction_index_ < 2)){
      lib_->createResultsBuf(results.at(i), results_buf_, diff, MAX);

      planner_->getEntities().put("loopCondition", false);
      break;
    } else if(diff.norm() > diff_max && (interaction_index_ > 1)) {
      lib_->interactionResultsBuf(results.at(i), results_buf_, interaction_buf_);

      ROS_INFO("           away");
      planner_->away();
      planner_->peekout();

      //todo debug
      planner_->getEntities().put("loopCondition", false);
      break;
    } else {
      ROS_INFO("nothing -----");
    }
  }

  pre_results_.clear();
  std::copy(results.begin(), results.end(), std::back_inserter(pre_results_));

  return _inhands;
};

int put(int _inhands, int &_nexttask) {
  lib_->setFCNModel("final");

  ROS_INFO("befpre results_buf_ -------");
  for(int i=0; i<results_buf_.size(); i++){
    ROS_INFO("results_buf_.at(%d): %f", i, results_buf_.at(i).y());
  }

  std::vector<Eigen::Vector3d> results;
  bool item_found = lib_->findItem("pie", results);

  lib_->createResultsBuf(results, results_buf_, MAX);

  ROS_INFO("after results_buf_ -------");
  for(int i=0; i<results_buf_.size(); i++){
    ROS_INFO("results_buf_.at(%d): %f", i, results_buf_.at(i).y());
  }

  ROS_INFO("results -------");
  for(int i=0; i<results.size(); i++){
    ROS_INFO("results.at(%d): %f", i, results.at(i).y());
  }

  Eigen::Vector3d put_pos(results_buf_.at(index_).x(), results_buf_.at(index_).y(), results_buf_.at(index_).z());

  negomo_lib::waitSettings ws;
  ws.interaction_flag =
    negomo_enshu::PlannerBridgeRequest::Request::IGNORE;

  Eigen::Vector3d look_pos(0.65, 0.0, 0.50);

  ROS_INFO("start tracking mode");
  robot_->setTrackingMode(true);
  robot_->setLookAt(look_pos, false, true);

  robot_->goPos(0, put_pos.y(), 0);
  put_pos.y() = put_pos_y_offset;
  put_pos.z() += put_pos_z_offset;

  ROS_INFO("start interaction moode");
  planner_->iStart(negomo_lib::jumpSettings(), ws);
  bool found = lib_->placeCoffeeReach(put_pos, 0, aero::arm::larm);

  if (!found) {
    robot_->moveTo("s_shelf_container");
    planner_->setError(true, "failed put");
    _nexttask = -404;
  }

  // finish interaction on background
  usleep(10000 * 1000);
  planner_->iJoin(_inhands, _nexttask);

  lib_->flag_mutex.lock();
  lib_->interaction_flag = false;
  lib_->flag_mutex.unlock();

  ROS_INFO("end interaction moode");

  // finish tracking object
  robot_->setTrackingMode(false);
  ROS_INFO("end tracking mode");

  if (_nexttask != -1)
    return lib_->getUsingHandsNum();

  lib_->openHand(aero::arm::larm);
  bool flag = lib_->placeCoffeeReturn();

  if(flag)
    ROS_INFO("end place coffee return");

  lib_->sendResetPose();

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
  bool found = lib_->poseAndRecognize("container", "pie", pos_, -0.2);

  if (found){
    robot_->goPos(0, pos_.y(), 0);
    found = lib_->poseAndRecognize("container", "pie", pos_, -0.2);
    pos_.y() += 0.02;
    found = lib_->pickCoffeeFront((pos_), 0.85, aero::arm::larm);
  }
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
  auto human_face_pos = planner_->getHeadPos(_target);;
  robot_->setLookAt(std::get<0>(human_face_pos), std::get<1>(human_face_pos), std::get<2>(human_face_pos), true, true, false);
  ROS_INFO("in reactive 0");
  lib_->flag_mutex.lock();
  lib_->interaction_flag = true;
  lib_->flag_mutex.unlock();
  ROS_INFO("flag is seted");
  robot_->ri->cancel_angle_vector();
  ROS_INFO("cance_angle_vector");
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
     std::make_tuple(0, 0, watchOnce, planner_->emptyAction, "watchOnce"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     //todo debug end loop implimentation
     std::make_tuple(0, 0, watch, planner_->emptyAction, "watch"),
     std::make_tuple(0, 0, planner_->loopEnd, planner_->emptyAction, "loop"),
     //todo pick and place after interaction
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finish"),};
  negomo_lib::ActionList task1 =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     // std::make_tuple(0, 0, containerPose, planner_->emptyAction, "container"),
     // std::make_tuple(0, 1, pick, planner_->emptyAction, "pick"),
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
