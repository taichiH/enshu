#include "aero_devel_lib/devel_lib.hh"

namespace aero {

  //////////////////////////////////////////////////////////
  DevelLib::DevelLib(ros::NodeHandle _nh, aero::interface::AeroMoveitInterface::Ptr _controller, Eigen::Vector3d _cam_pos, Eigen::Quaterniond _cam_qua)
    : nh_(_nh), controller_(_controller)
  {
    usleep(1000 * 1000);
    // controller_->setPoseVariables(aero::pose::reset);
    // controller_->sendModelAngles(2000);
    controller_->setRobotStateToCurrentState();
    features_.reset(new aero::vision::ObjectFeatures(nh_, controller_));
    features_->setCameraTransform("head_base_link", _cam_pos, _cam_qua);
    // fcn_sub_ = nh_.subscribe("/object_3d_projector/output", 1, &DevelLib::fcnCallback_, this);
    fcn_sub_ = nh_.subscribe("/object_3d_projector_/output", 1, &DevelLib::fcnCallback_, this);
    hand_sub_ = nh_.subscribe("/hand_detector/boxes", 1, &DevelLib::handCallback_, this);
    fcn_starter_ = nh_.serviceClient<std_srvs::SetBool>("/object_detector/set_mode");
    ar_sub_ = nh_.subscribe("/ar_pose_marker", 1, &DevelLib::arMarkerCallback_, this);
    ar_start_pub_ = nh_.advertise<std_msgs::Bool>("/ar_track_alvar/enable_detection", 1);
    initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    speak_pub_ = nh_.advertise<std_msgs::String>("/windows/voice", 10);
  }

  //////////////////////////////////////////////////////////
  DevelLib::~DevelLib(){}

  //////////////////////////////////////////////////////////
  void DevelLib::speak(const std::string &_speech, const float &_wait_sec) {
    if (_wait_sec > 500) { // obviously too long for a speech
      ROS_WARN("detected a large speaking time! mistaken seconds as milliseconds?");
      _wait_sec / 1000.0f; // forcefully change to seconds
    }
    speakAsync(_speech);
    usleep(static_cast<int>(_wait_sec * 1000) * 1000);
  }

  //////////////////////////////////////////////////////////
  void DevelLib::speakAsync(const std::string &_speech) {
    ROS_INFO("speak: %s", _speech.c_str());
    std_msgs::String msg;
    msg.data = _speech;
    speak_pub_.publish(msg);
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::makeTopGrasp(const aero::arm _arm, const Eigen::Vector3d _pos, aero::trajectory& _tra) {
    // diffs
    Eigen::Vector3d end_diff, mid_diff, mid2_diff, mid3_diff, mid4_diff, entry_diff, offset;
    end_diff = {0.0 ,0.0, -0.03};
    mid_diff = {0.0,0.0,0.01};
    mid2_diff = {-0.02,0.0,0.05};
    mid3_diff = {-0.05,0.0,0.10};
    mid4_diff = {-0.075,0.0,0.125};
    entry_diff = {-0.10,0.0,0.15};
    offset = {0.0,0.0,0.0};

    // rotations
    Eigen::Quaterniond end_qua, mid_qua, entry_qua;
    if(_arm == aero::arm::larm) {
      end_qua = getRotationQuaternion("y", 45.0 * M_PI / 180.0)
        * getRotationQuaternion("x", 90.0 * M_PI / 180.0);
      mid_qua = getRotationQuaternion("y", 45.0 * M_PI / 180.0)
        * getRotationQuaternion("x", 90.0 * M_PI / 180.0);
      entry_qua = getRotationQuaternion("x", 90.0 * M_PI / 180.0);
    } else {
      end_qua = getRotationQuaternion("y", 45.0 * M_PI / 180.0)
        * getRotationQuaternion("x", -90.0 * M_PI / 180.0);
      mid_qua = getRotationQuaternion("y", 45.0 * M_PI / 180.0)
        * getRotationQuaternion("x", -90.0 * M_PI / 180.0);
      entry_qua = getRotationQuaternion("x", -90.0 * M_PI / 180.0);
    }

    aero::Transform end_pose, mid_pose, mid2_pose, mid3_pose, mid4_pose, entry_pose;
    end_pose = aero::Translation(_pos + offset + end_diff) * end_qua;
    mid_pose = aero::Translation(_pos + offset + mid_diff) * mid_qua;
    mid2_pose = aero::Translation(_pos + offset + mid2_diff) * mid_qua;
    mid3_pose = aero::Translation(_pos + offset + mid3_diff) * mid_qua;
    mid4_pose = aero::Translation(_pos + offset + mid4_diff) * mid_qua;
    entry_pose = aero::Translation(_pos + offset + entry_diff) * entry_qua;
    features_->setMarker(entry_pose, 1);
    features_->setMarker(mid_pose, 2);
    features_->setMarker(mid2_pose, 3);
    features_->setMarker(mid3_pose, 4);
    features_->setMarker(mid4_pose, 5);
    features_->setMarker(end_pose, 6);
    ROS_WARN("entry: x:%f y:%f z:%f", entry_pose.translation().x(), entry_pose.translation().y(), entry_pose.translation().z());
    ROS_WARN("mid  : x:%f y:%f z:%f", mid_pose.translation().x(), mid_pose.translation().y(), mid_pose.translation().z());
    ROS_WARN("mid2  : x:%f y:%f z:%f", mid2_pose.translation().x(), mid2_pose.translation().y(), mid2_pose.translation().z());
    ROS_WARN("mid3  : x:%f y:%f z:%f", mid3_pose.translation().x(), mid3_pose.translation().y(), mid3_pose.translation().z());
    ROS_WARN("mid4  : x:%f y:%f z:%f", mid4_pose.translation().x(), mid4_pose.translation().y(), mid4_pose.translation().z());
    ROS_WARN("end  : x:%f y:%f z:%f", end_pose.translation().x(), end_pose.translation().y(), end_pose.translation().z());

    aero::trajectory tra;
    bool res = fastestTrajectory3(_arm, {entry_pose, mid4_pose, mid3_pose, mid2_pose, mid_pose, end_pose}, aero::eef::pick, tra);
    if (!res) {
      ROS_INFO("%s: ik failed", __FUNCTION__);
      return false;
    }
    ROS_INFO("%s: success!", __FUNCTION__);
    _tra = tra;
    return true;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::pickCoffeeFront(Eigen::Vector3d _pos, float _container_height, aero::arm _arm, Eigen::Vector3d _offset) {
    double factor = 0.7;
    // open hand
    openHand(_arm);

    // make grasp trajectory
    aero::trajectory tra;
    Eigen::Vector3d pos = _pos + _offset;
    pos.z() = _container_height;

    bool ik_res = makeTopGrasp(_arm, pos, tra);
    if(!ik_res) {
      ROS_INFO("%s:ik failed", __FUNCTION__);
      return false;
    }

    // reach
    controller_->sendTrajectory(tra, calcTrajectoryTimes(tra, factor), aero::ikrange::wholebody);
    controller_->waitInterpolation();

    // grasp
    bool res = graspCoffee(aero::arm::larm);

    // unreach
    aero::trajectory tra_back = {tra.at(1), tra.at(0)};
    controller_->sendTrajectory(tra_back, calcTrajectoryTimes(tra_back, factor), aero::ikrange::wholebody);
    controller_->waitInterpolation();

    // success or fail form grasp
    return res;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::visualServo(aero::Vector3 &_pos, double &_offset, aero::arm _arm){
    aero::Translation r_pos(_pos.x(), _pos.y() - _offset, _pos.z());
    aero::Quaternion  r_rot(1.0, 0.0, 0.0, 0.0);
    aero::Transform   r_pose = r_pos * r_rot;
    bool r_ik_result = controller_->setFromIK(aero::arm::rarm, aero::ikrange::wholebody, r_pose, aero::eef::grasp);

    bool l_ik_result = false;
    if(_arm == aero::arm::both_arms){
      aero::Translation l_pos(_pos.x(), _pos.y() + _offset, _pos.z());
      aero::Quaternion  l_rot(1.0, 0.0, 0.0, 0.0);
      aero::Transform   l_pose = l_pos * l_rot;
      l_ik_result = controller_->setFromIK(aero::arm::larm, aero::ikrange::arm, l_pose, aero::eef::grasp);
    }

    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);

    if (r_ik_result && l_ik_result) {
      ROS_WARN("dual arm ik success !");
      controller_->sendModelAngles(calcPathTime(av, 0.8));
    } else if(r_ik_result){
      ROS_WARN("single rarm ik success !");
      controller_->sendModelAngles(calcPathTime(av, 0.8));
    } else {
      ROS_WARN("ik failed");
      return false;
    }

    return true;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::setBothArm(aero::Transform &_r_pose, aero::Transform &_l_pose){
    bool r_ik_result = controller_->setFromIK(aero::arm::rarm, aero::ikrange::wholebody, _r_pose, aero::eef::grasp);
    bool l_ik_result = controller_->setFromIK(aero::arm::larm, aero::ikrange::arm, _l_pose, aero::eef::grasp);

    if(r_ik_result)
      ROS_WARN("r_ik: true");
    if(l_ik_result)
      ROS_WARN("l_ik: true");

    if (r_ik_result && l_ik_result) {
      ROS_INFO("dual arm ik success !");
      std::map<aero::joint, double> av;
      controller_->getRobotStateVariables(av);
      controller_->sendModelAngles(calcPathTime(av, 0.8));
      controller_->waitInterpolation();
    } else {
      ROS_WARN("ik failed");
    }
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::graspCoffee(aero::arm _arm) {
#ifdef DUMMY_MODE
    ROS_INFO("%s: DUMMY_MODE, grasping always returns true", __FUNCTION__);
      setUsingArm(_arm);
    return true;
#endif
    controller_->sendGrasp(_arm, 50);
    usleep(500 * 1000);
    controller_->setRobotStateToCurrentState();
    double index = controller_->kinematic_state->getVariablePosition("r_indexbase_joint");
    double thumb = controller_->kinematic_state->getVariablePosition("r_thumb_joint");
    ROS_INFO("%s: index %f", __FUNCTION__, index);
    if (index < 0.05) {
      ROS_INFO("%s: success", __FUNCTION__);
      setUsingArm(_arm);
      return true;
    }
    ROS_WARN("%s: failed", __FUNCTION__);
    openHand(_arm);
    unsetUsingArm(_arm);
    return false;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::createResultsBuf(const Eigen::Vector3d &_result, std::vector<Eigen::Vector3d> &_results_buf, Eigen::Vector3d &_diff, const int &_max){
    ROS_INFO("first teaching");
    _results_buf.push_back(_result);
    for(int j=2; j<_max; ++j){
      _results_buf.push_back(_results_buf.at(j-1) + _diff);
    }
    visualizeMarker(_results_buf);
    return true;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::createResultsBuf(std::vector<Eigen::Vector3d> &_results, std::vector<Eigen::Vector3d> &_results_buf, const int &_max){
    ROS_INFO("creat results_buf (put mode)");

    _results_buf.clear();

    std::sort(_results.begin(), _results.end(),
              [](const Eigen::Vector3d &left, const Eigen::Vector3d &right){return left.y() > right.y();}
              );

    Eigen::Vector3d diff = _results.back() - _results.at(0);
    diff = diff / (_results.size() - 1);
    ROS_INFO("diff.norm (put mode) : %f", diff.norm());

    std::copy(_results.begin(), _results.end(), std::back_inserter(_results_buf));

    if(_results.size() != _max){
      for(int i=_results.size(); i<_max; ++i){
        _results_buf.push_back(_results_buf.at(i-1) + diff);
      }
    } else {
      return false;
    }

    visualizeMarker(_results_buf);
    return true;
  }


  //////////////////////////////////////////////////////////
  bool DevelLib::interactionResultsBuf(const Eigen::Vector3d &_result, std::vector<Eigen::Vector3d> &_results_buf, std::vector<Eigen::Vector3d> &_last_buf){
    _results_buf.clear();
    Eigen::Vector3d diff = _result -  _last_buf.at(0);
    for(int i=0; i<_last_buf.size(); ++i){
      _results_buf.push_back(_last_buf.at(i) + diff);
      ROS_INFO("_results_buf.at(%d): %f, %f, %f", i, _results_buf.at(i).x(), _results_buf.at(i).y(), _results_buf.at(i).z());
    }
    visualizeMarker(_results_buf);
    return true;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::visualizeMarker(const std::vector<Eigen::Vector3d> &_results_buf){
    const int features_var = 7;
    aero::Transform obj_pos;
    for(int i=0; i<_results_buf.size(); ++i){
      obj_pos = aero::Translation(_results_buf.at(i));
      features_->setMarker(obj_pos, i+features_var);
    }
    return true;
  }
  //////////////////////////////////////////////////////////
  bool DevelLib::getNewPutPos(std::vector<Eigen::Vector3d> &_results, std::vector<Eigen::Vector3d> &_pre_results){
    std::sort(_results.begin(), _results.end(),
              [](const Eigen::Vector3d &left, const Eigen::Vector3d &right){return left.y() > right.y();});

    std::sort(_pre_results.begin(), _pre_results.end(),
              [](const Eigen::Vector3d &left, const Eigen::Vector3d &right){return left.y() > right.y();});

    if(_results.size() == _pre_results.size()){
      _results.clear();
      return false;
    }

    if(_results.size() > _pre_results.size()) {
      //get nearest x item
      std::sort(_results.begin(), _results.end(),
                [](const Eigen::Vector3d &left, const Eigen::Vector3d &right){return left.x() < right.x();});
      auto tmp = _results.at(0);
      _results.clear();
      _results.push_back(tmp);
      return true;
    }
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::watchFlag(const double &_norm, const double &_min, double _max, const int &_index){
    if((_norm > _min) && (_norm < _max) && (_index < 2)){
      return true;
    } else {
      return false;
    }
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::watchFlag(const double &_norm, const double &_min, const int &_index){
    if((_norm > _min) && (_index > 1)){
      return true;
    } else {
      return false;
    }
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::placeCoffee(Eigen::Vector3d _pos, double _offset_y, aero::arm _arm) {
    placeCoffeeReach(_pos, _offset_y, _arm);
    openHand(_arm);
    placeCoffeeReturn();
    return true;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::placeCoffeeReach(Eigen::Vector3d _pos, double _offset_y, aero::arm _arm) {
    controller_->setPoseVariables(aero::pose::reset_manip);
    controller_->resetLookAt();
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendModelAngles(calcPathTime(av, 0.8));
    bool wait_flag = controller_->waitInterpolation(0.1);

    double factor = 0.8;

    tra_.clear();
    // make trajectory
    bool res = makeTopGrasp(_arm, _pos, tra_);

    if (!res) {
      ROS_WARN("%s: place ik failed", __FUNCTION__);
      return false;
    }

    if (_offset_y != 0) {
      ROS_INFO("%s: adjust y with wheel %f [m]", __FUNCTION__, _offset_y);
      controller_->goPos(0.0, _offset_y, 0.0);
    }

    flag_mutex.lock();
    bool flag = interaction_flag;
    flag_mutex.unlock();

    ROS_INFO("> tra size is %d", static_cast<int>(tra_.size()));
    if(!flag){
      ROS_INFO("flag: %d", static_cast<int>(flag));
      controller_->sendTrajectory(tra_, calcTrajectoryTimes(tra_, 0.8), aero::ikrange::wholebody);
      controller_->waitInterpolation();
    }
    // usleep(200 * 1000);
    // controller_->stopMotion();
    return true;
  }

  bool DevelLib::placeCoffeeReturn() {
    std::vector<double> factors;
    aero::trajectory tra_release;
    ROS_INFO("< tra size is %d", static_cast<int>(tra_.size()));
    tra_release.push_back(tra_.at(1));
    tra_release.push_back(tra_.at(0));
    controller_->sendTrajectory(tra_release, calcTrajectoryTimes(tra_release, 0.8), aero::ikrange::wholebody);
    controller_->waitInterpolation();
    return true;
  }

  void DevelLib::sendResetPose() {
    controller_->setPoseVariables(aero::pose::reset_manip);
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendModelAngles(calcPathTime(av, 0.7));
    controller_->waitInterpolation();
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::poseAndRecognize(const std::string _location, const std::string _item, Eigen::Vector3d& _pos, float _lifter_z) {
    if(_location == "container") {
      lookContainerFront(_lifter_z);
    } else if (_location == "shelf") {
      lookShelf();
    } else {
      ROS_WARN("%s: where is %s ?", __FUNCTION__, _location.c_str());
      return false;
    }
    usleep(1000 * 1000);
    bool res = false;
    res = findNearestItem(_item, _pos);
    return res;
  }

  //////////////////////////////////////////////////////////
  void DevelLib::lookContainerFront(float _lifter_z) {
    controller_->setPoseVariables(aero::pose::reset_manip);
    controller_->setLifter(0.05, _lifter_z);
    controller_->setJoint(aero::joint::r_shoulder_y, -0.3);
    controller_->setJoint(aero::joint::l_shoulder_y, 0.3);
    controller_->setNeck(0.0, M_PI/ 2.0 ,0.0);
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendModelAngles(calcPathTime(av, 0.8),aero::ikrange::wholebody);
    controller_->waitInterpolation();
 }


    //////////////////////////////////////////////////////////
  void DevelLib::lookShelfFront(float _lifter_z) {
    controller_->setPoseVariables(aero::pose::reset_manip);
    // controller_->setLifter(0.0, _lifter_z);
    controller_->setLifter(0.05, _lifter_z);
    controller_->setJoint(aero::joint::r_shoulder_y, -0.3);
    controller_->setJoint(aero::joint::l_shoulder_y, 0.3);
    //controller_->setJoint(aero::joint::waist_y, -M_PI / 2.0);
    controller_->setNeck(0.0, M_PI/ 2.0 ,0.0);
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    ROS_INFO("   time is %d", calcPathTime(av, 0.8));
    controller_->sendModelAngles(5000,aero::ikrange::wholebody);
    controller_->waitInterpolation();
 }

  //////////////////////////////////////////////////////////
  void DevelLib::lookShelf() {
    controller_->setPoseVariables(aero::pose::reset_manip);
    controller_->setLifter(0.0, -0.1);
    controller_->setJoint(aero::joint::r_shoulder_y, -0.3);
    controller_->setJoint(aero::joint::l_shoulder_y, 0.3);
    controller_->setNeck(0.0,M_PI/2.0,0.0);
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendModelAngles(std::max(calcPathTime(av, 0.8), 1000), aero::ikrange::wholebody);
    controller_->waitInterpolation();
  }

  //////////////////////////////////////////////////////////
  void DevelLib::startFCN() {
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (!fcn_starter_.call(srv))
      ROS_WARN("%s: start service call failed", __FUNCTION__);
    else
      ROS_INFO("%s: FCN start", __FUNCTION__);
    return;
  }

  //////////////////////////////////////////////////////////
  void DevelLib::setFCNModel(std::string _name, int _projector_id) {
    setObject3DProjectorMode(_projector_id);
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::StrParameter str_param;
    dynamic_reconfigure::Config conf;

    str_param.name = "model_name";
    str_param.value = _name;

    conf.strs.push_back(str_param);
    srv_req.config = conf;
    ros::service::call("/object_detector/set_parameters", srv_req, srv_resp);
  }

  //////////////////////////////////////////////////////////
  void DevelLib::setObject3DProjectorMode(int _id) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    int_param.name = "mode";
    int_param.value = _id;

    conf.ints.push_back(int_param);
    srv_req.config = conf;
    ros::service::call("/object_3d_projector/set_parameters", srv_req, srv_resp);
  }

  //////////////////////////////////////////////////////////
  std::vector<aero::item> DevelLib::recognizeItems() {
    startFCN();
    fcn_msg_ = aero_recognition_msgs::LabeledPoseArray();
    std::vector<aero::item> result;
    ros::Time now = ros::Time::now();
    for (int i = 0; i < 20; ++i) {
      ros::spinOnce();
      if (fcn_msg_.header.stamp > now) break;
      usleep(100 * 1000);
    }
    stopFCN();
    controller_->setRobotStateToCurrentState();
    int id =0;
    for (auto box : fcn_msg_.poses) {
      Eigen::Vector3d vec_tmp;
      vec_tmp.x() = box.pose.position.x;
      vec_tmp.y() = box.pose.position.y;
      vec_tmp.z() = box.pose.position.z;
      aero::item item_tmp;
      item_tmp.position = features_->convertWorld(vec_tmp, false);
      item_tmp.label = box.label;
      result.push_back(item_tmp);
      ROS_INFO("%s:item %s x:%f y:%f z:%f", __FUNCTION__, box.label.c_str(), item_tmp.position.x(), item_tmp.position.y(), item_tmp.position.z());
    }
    ROS_INFO("%s:%d items are found", __FUNCTION__, id);
    // refresh msg
    fcn_msg_ = aero_recognition_msgs::LabeledPoseArray();

    return result;
  }

  //////////////////////////////////////////////////////////
  std::vector<aero_recognition_msgs::Scored2DBox> DevelLib::recognizeHand() {
    hand_msg_ = aero_recognition_msgs::Scored2DBoxArray();
    ros::Time now = ros::Time::now();
    for (int i = 0; i < 20; ++i) {
      ros::spinOnce();
      if (hand_msg_.header.stamp > now) break;
      usleep(100 * 1000);
    }
    std::vector<aero_recognition_msgs::Scored2DBox> result;
    aero_recognition_msgs::Scored2DBox hands;
    for (auto box : hand_msg_.boxes) {
      result.push_back(hands);
    }
    return result;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::findItem(std::string _label, std::vector<Eigen::Vector3d> &_positions) {
    _positions.clear();
    std::vector<aero::item> items = recognizeItems();

    if (items.empty()) {
      ROS_INFO("%s: no items found", __FUNCTION__);
      return false;
    }

    for (auto item: items)
      if (item.label == _label)
        _positions.push_back(item.position);

    if (_positions.empty()) {
      ROS_INFO("%s: item %s is not found", __FUNCTION__, _label.c_str());
      return false;
    }
    ROS_INFO("%s: %d %ss are found", __FUNCTION__, static_cast<int>(_positions.size()), _label.c_str());

    return true;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::findNearestItem(std::string _label, Eigen::Vector3d &_position) {
    std::vector<Eigen::Vector3d> positions;
    bool found=false;
    for (int i =0; i < 5; ++i) {
      if (findItem(_label, positions)) {
        found = true;
        break;
      }
      usleep(100 * 1000);
    }

    if (!found) return false;

    Eigen::Vector3d res;
    double dis_cube = std::numeric_limits<double>::max();
    for(auto &pos: positions) {
      double tmp = std::pow(pos.x(),2.0) + std::pow(pos.y(), 2.0);
      if(dis_cube > tmp) {
        res = pos;
        dis_cube = tmp;
      }
    }
    _position = res;
    return true;
  }

  //////////////////////////////////////////////////////////
  void DevelLib::stopFCN() {
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (!fcn_starter_.call(srv))
      ROS_WARN("%s: stop service call failed", __FUNCTION__);
    else
      ROS_INFO("%s: FCN stop", __FUNCTION__);
    return;
  }

  //////////////////////////////////////////////////////////
  void DevelLib::fcnCallback_(const aero_recognition_msgs::LabeledPoseArray::ConstPtr _msg) {
    fcn_msg_ = *_msg;
  }

  //////////////////////////////////////////////////////////
  void DevelLib::handCallback_(const aero_recognition_msgs::Scored2DBoxArray::ConstPtr _hand_boxes) {
    hand_msg_ = *_hand_boxes;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::goTo(std::string _location) {
    controller_->moveTo(_location);

    while (controller_->isMoving() && ros::ok()) {
      ROS_INFO("i'm going to %s", _location.c_str());
      usleep(200 * 1000);
    }
    usleep(1000 * 1000);
    if (!controller_->isAt(_location, 0.1)) {
      ROS_WARN("i can't move to %s", _location.c_str());
      return false;
    }
    return true;
  }

  //////////////////////////////////////////////////////////
  void DevelLib::triggerArMarker(bool _trigger) {
    std_msgs::Bool msg;
    msg.data = _trigger;
    ar_start_pub_.publish(msg);
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::adjustShelfArMarker(std::vector<std::string> _markernames) {
    triggerArMarker(true); // start AR marker

    ros::Time now = ros::Time::now();
    std::map<int, Eigen::Vector3d> ref_markers, markers;
    std::vector<int> indexes;

    for (auto tmp_name : _markernames) {
      Eigen::Vector3d tmp_position;
      tf::StampedTransform tr;
      try {
        tf_listener_.waitForTransform("/map", tmp_name, ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform("/map", tmp_name, ros::Time(0), tr);
      } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        triggerArMarker(false); // stop AR marker
        return false;
      }
      tmp_position.x() = tr.getOrigin().x();
      tmp_position.y() = tr.getOrigin().y();
      tmp_position.z() = tr.getOrigin().z();
      // tf name -> AR id
      std::size_t pos;
      while ((pos = tmp_name.find("_")) != std::string::npos) {
        std::string str = tmp_name.substr(0, pos);
        tmp_name.erase(0, pos + 1);
      }
      ref_markers[std::stoi(tmp_name)] = tmp_position;
    }


    // get markers from cv
    bool marker_found = false;
    for (int i = 0; i < 5; ++i) {
      ros::spinOnce();
      if (ar_msg_.header.stamp > now) {
        marker_found = true;
        break;
      }
      usleep(100 * 1000);
    }


    controller_->setRobotStateToCurrentState();
    for (auto it : ar_msg_.markers) // match markers id with reference
      if (ref_markers.find(it.id) != ref_markers.end()) {
        Eigen::Vector3d tmp_position;
        tf::pointMsgToEigen(it.pose.pose.position, tmp_position);
        markers[it.id] = features_->convertWorld(tmp_position);
        indexes.push_back(it.id);
      }

    // triggerArMarker(false); // stop AR marker

    if (static_cast<int>(markers.size()) < 2) {
      ROS_WARN("%s: not enough markers for matching shelf", __FUNCTION__);
      return false;
    }

    // marker adjustment from here
    std::vector<Eigen::Vector3d> globals, locals;
    for (int i : indexes) {
      globals.push_back(ref_markers.at(i));
      locals.push_back(markers.at(i));
    }


    // compute transform estimation SVDXY
    Eigen::Vector3d local_com = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d global_com = Eigen::VectorXd::Zero(3);
    int num = static_cast<int>(locals.size());
    Eigen::MatrixXd X_dash(2, num), Y_dash(2, num), R(2,2), U(2,2), V(2,2), H(2,2);
    // compute center of mass
    for (int i = 0; i < num; ++i) {
      local_com += locals.at(i);
      global_com += globals.at(i);
    }
    local_com /= num;
    global_com /= num;

    // make matrix
    for (int i = 0; i < num; ++i) {
      X_dash(0,i) = locals.at(i).x() - local_com.x();
      X_dash(1,i) = locals.at(i).y() - local_com.y();
      Y_dash(0,i) = globals.at(i).x() - global_com.x();
      Y_dash(1,i) = globals.at(i).y() - global_com.y();
    }

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(X_dash * Y_dash.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();

    // make rotation matrix
    Eigen::Matrix3d rot;
    H = Eigen::MatrixXd::Identity(2,2);
    H(1,1) = (V * U.transpose()).determinant();
    R = V * H * U.transpose();
    rot = Eigen::MatrixXd::Identity(3,3);
    rot(0,0) = R(0,0);
    rot(1,0) = R(1,0);
    rot(0,1) = R(0,1);
    rot(1,1) = R(1,1);

    // calc trans
    Eigen::Vector3d trans = global_com - rot * local_com;

    Eigen::Quaterniond qua(rot);

    if (std::isnan(trans.x()) || std::isnan(trans.y())) {
      ROS_WARN("%s: nan found", __FUNCTION__);
      return false;
    }

    geometry_msgs::PoseWithCovarianceStamped initialpose;
    initialpose.header.stamp = now;
    initialpose.header.frame_id = "/map";
    initialpose.pose.pose.position.x = trans.x();
    initialpose.pose.pose.position.y = trans.y();
    initialpose.pose.pose.position.z = 0.0;
    initialpose.pose.pose.orientation.w = qua.w();
    initialpose.pose.pose.orientation.x = qua.x();
    initialpose.pose.pose.orientation.y = qua.y();
    initialpose.pose.pose.orientation.z = qua.z();
    initialpose.pose.covariance[0] = 0.25;
    initialpose.pose.covariance[7] = 0.25;
    initialpose.pose.covariance[35] = 0.06853891945200942;
    initialpose_pub_.publish(initialpose);

    return true;
  }



  void DevelLib::arMarkerCallback_(const ar_track_alvar_msgs::AlvarMarkersPtr _msg) {
    ar_msg_ = *_msg;
  }

  //////////////////////////////////////////////////////////
  Eigen::Quaterniond DevelLib::getRotationQuaternion(std::string _axis, double _radian)
  {
    Eigen::Quaterniond qua;
    Eigen::Vector3d unit;
    if(_axis == "x") {
      unit = Eigen::Vector3d::UnitX();
    } else if(_axis == "y") {
      unit = Eigen::Vector3d::UnitY();
    } else if(_axis == "z") {
      unit = Eigen::Vector3d::UnitZ();
    } else {
      ROS_WARN("%s: axis is x|y|x", __FUNCTION__);
    }
    qua = Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(_radian, unit)));
    return qua;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::fastestTrajectory3(const aero::arm _arm, const std::vector<aero::Transform> _poses, const aero::eef _eef, aero::trajectory& _tra, bool _lock_lifter) {
    std::map<aero::joint, double> reset_pose;
    controller_->setPoseVariables(aero::pose::reset_manip);
    controller_->getRobotStateVariables(reset_pose);

    std::vector<std::map<aero::joint, double> > poses;
    // solve all poses
    for (auto ee = _poses.begin(); ee != _poses.end(); ++ee) {
      controller_->setRobotStateVariables(reset_pose);
      if (!controller_->setFromIK(_arm ,aero::ikrange::wholebody, *ee, _eef)) {
        ROS_WARN("%s: ik %d failed", __FUNCTION__, static_cast<int>(ee - _poses.begin()));
        ROS_WARN("pos: x:%f y:%f z:%f", ee->translation().x(), ee->translation().y(), ee->translation().z());
        return false;
      } else {
        std::map<aero::joint, double> pose;
        controller_->getRobotStateVariables(pose);
        poses.push_back(pose);
      }
    }

    // make iks from another state
    std::vector<aero::trajectory> trajs_per_pose;
    for (auto ref_pose = poses.begin(); ref_pose != poses.end(); ++ref_pose) {
      bool has_solution = true;
      aero::trajectory tmpv(poses.size());
      auto tmp = tmpv.begin();
      controller_->setRobotStateVariables(*ref_pose);
      int j = static_cast<int>(ref_pose - poses.begin());
      for (int i = 0; i < _poses.size(); ++i) {
        auto ee = _poses.begin() + i;
        if (i != j)
          if (!controller_->setFromIK(_arm, aero::ikrange::upperbody, *ee, _eef)) {
            has_solution = false;
            break;
          }
        controller_->getRobotStateVariables(*tmp);
        controller_->setRobotStateVariables(*ref_pose);
        ++tmp;
      }
      if (has_solution)
        trajs_per_pose.push_back(tmpv);
    }

    if (trajs_per_pose.size() == 0) {
      ROS_WARN("%s: could not solve for any of the actions", __FUNCTION__);
      return false;
    }

    // search fastest path
    double time_factor = 0.8;
    int min_time = std::numeric_limits<int>::max();
    int index = 0;
    if (_lock_lifter) {
      for (int i = 0; i < static_cast<int>(trajs_per_pose.size()); ++i) {
        int time = 0;
        auto traj = trajs_per_pose.at(i);
        for (auto pose = traj.begin() + 1; pose != traj.end(); ++pose)
          time += calcPathTime(*(pose - 1), *pose, time_factor);
        if (time < min_time) {
          min_time = time;
          index = i;
        }
      }
    } else {
      // TODO
      return false;
    }

    // return
    _tra.clear();
    _tra.assign(trajs_per_pose.at(index).begin(), trajs_per_pose.at(index).end());

    return true;
  }

  //////////////////////////////////////////////////////////
  int DevelLib::calcPathTime(std::map<aero::joint, double> _av_from, std::map<aero::joint, double> _av_to, double _factor) {
    std::string rate_limit_joint;
    double time_sec=0.0;
    for(auto j: _av_from) {
      aero::joint joint = j.first;
      if (_av_to.find(joint) == _av_to.end())
        continue;

      double angle_from = _av_from.at(joint);
      double angle_to = _av_to.at(joint);
      if(angle_from == angle_to)
        continue;

      auto bounds = controller_->kinematic_model->getVariableBounds(aero::joint_map.at(joint));
      double vel = _factor * bounds.max_velocity_;
      if (vel == 0.0) {
        ROS_WARN("%s: joint %s 's max velocity is zero!!", __FUNCTION__, aero::joint_map.at(joint).c_str());
        ROS_WARN("pleese check the urdf and secify the velocity");
        ros::shutdown();
        return 1000 * 1000;
      }
      double tmp_time = std::fabs(angle_from - angle_to) / vel; // [sec]
      if (tmp_time > time_sec) {
        time_sec = tmp_time; // choose longest time
        rate_limit_joint = aero::joint_map.at(joint);
      }
    }

    int msec = static_cast<int>(1000 * time_sec);
    if (msec < 21) msec = 21; // controller limit
    // ROS_INFO("%s: rate limit joint :%s :time %d", __FUNCTION__, rate_limit_joint.c_str(), msec);
    return msec; // to mili sec
  }

  //////////////////////////////////////////////////////////
  int DevelLib::calcPathTime(std::map<aero::joint, double> _av, double _factor) {
    std::map<aero::joint, double> av_current;
    controller_->getCurrentState(av_current);
    return calcPathTime(av_current, _av, _factor);
  }

  //////////////////////////////////////////////////////////
  std::vector<int> DevelLib::calcTrajectoryTimes(aero::trajectory _trajectory, double _factor) {
    std::vector<int> result;
    bool first_loop = true;
    for (auto it = _trajectory.begin(); it != _trajectory.end(); ++it) {
      if (first_loop) {
        result.push_back(calcPathTime(*it, _factor));
        first_loop = false;
      } else {
        result.push_back(calcPathTime(*(it-1), *it, _factor));
      }
    }

    return result;
  }

  //////////////////////////////////////////////////////////
  std::vector<int> DevelLib::calcTrajectoryTimes(aero::trajectory _trajectory, std::vector<double> _factors) {
    if(_trajectory.size() != _factors.size()) {
      ROS_WARN("%s:trajectory size %d and factor size %d differ", __FUNCTION__, static_cast<int>(_trajectory.size()), static_cast<int>(_factors.size()));
      return std::vector<int>(_trajectory.size(), 10000);
    }
   std::vector<int> result;
   bool first_loop = true;
   int index = 0;
   for (auto it = _trajectory.begin(); it != _trajectory.end(); ++it) {
     if (first_loop) {
       result.push_back(calcPathTime(*it, _factors.at(index)));
       first_loop = false;
     } else {
       result.push_back(calcPathTime(*(it-1), *it, _factors.at(index)));
     }
     ++index;
   }
   return result;
  }



  //////////////////////////////////////////////////////////
  int DevelLib::getUsingHandsNum() {
    int res = 0;
    if (isUsingArm(aero::arm::rarm)) ++res;
    if (isUsingArm(aero::arm::larm)) ++res;
    return res;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::isUsingArm(aero::arm _arm) {
    if (_arm == aero::arm::rarm) {
      return using_rarm_;
    } else {
      return using_larm_;
    }
  }

  //////////////////////////////////////////////////////////
  void DevelLib::setUsingArm(aero::arm _arm) {
    if (_arm == aero::arm::rarm) {
      using_rarm_ = true;
    } else {
      using_larm_ = true;
    }
  }

  //////////////////////////////////////////////////////////
  void DevelLib::unsetUsingArm(aero::arm _arm) {
    if(_arm == aero::arm::rarm) {
      using_rarm_ = false;
    } else {
      using_larm_ = false;
    }
  }

  //////////////////////////////////////////////////////////
  void DevelLib::openHand(aero::arm _arm) {
    controller_->openHand(_arm);
    unsetUsingArm(_arm);
  }
}
