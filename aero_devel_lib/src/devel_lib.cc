#include "aero_devel_lib/devel_lib.hh"

namespace aero {

  //////////////////////////////////////////////////////////
  DevelLib::DevelLib(ros::NodeHandle _nh, aero::interface::AeroMoveitInterface::Ptr _controller, Eigen::Vector3d _cam_pos, Eigen::Quaterniond _cam_qua)
    : nh_(_nh), controller_(_controller)
  {
    controller_->setRobotStateToCurrentState();
    features_.reset(new aero::vision::ObjectFeatures(nh_, controller_));
    features_->setCameraTransform("head_base_link", _cam_pos, _cam_qua);
    fcn_sub_ = nh_.subscribe("/object_3d_projector/output", 1, &DevelLib::fcnCallback_, this);
    fcn_starter_ = nh_.serviceClient<std_srvs::SetBool>("/object_detector/set_mode");
    speak_pub_ = nh_.advertise<std_msgs::String>("/windows/voice", 10);
    usleep(1000 * 1000);
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
    Eigen::Vector3d end_diff, mid_diff, entry_diff, offset;
    end_diff = {0.0 ,0.0, -0.04};
    mid_diff = {-0.0,0.0,0.1};
    entry_diff = {-0.1,0.0,0.15};
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

    aero::Transform end_pose, mid_pose, entry_pose;
    end_pose = aero::Translation(_pos + offset + end_diff) * end_qua;
    mid_pose = aero::Translation(_pos + offset + mid_diff) * mid_qua;
    entry_pose = aero::Translation(_pos + offset + entry_diff) * entry_qua;
    features_->setMarker(entry_pose, 1);
    features_->setMarker(mid_pose, 2);
    features_->setMarker(end_pose, 3);
    ROS_WARN("entry: x:%f y:%f z:%f", entry_pose.translation().x(), entry_pose.translation().y(), entry_pose.translation().z());
    ROS_WARN("mid  : x:%f y:%f z:%f", mid_pose.translation().x(), mid_pose.translation().y(), mid_pose.translation().z());
    ROS_WARN("end  : x:%f y:%f z:%f", end_pose.translation().x(), end_pose.translation().y(), end_pose.translation().z());

    // desired hand position from waist
    Eigen::Vector3d des_pos = {0.6, 0.2, 0.1};
    if (end_pose.translation().x() > 0.75)
      des_pos = {0.6, 0.2, end_pose.translation().z() - 0.9 + 0.15}; // dirty
    if (_arm == aero::arm::rarm)
      des_pos.y() = -0.2;

    aero::trajectory tra;
    bool res = fastestTrajectory3(_arm, {entry_pose, mid_pose, end_pose}, aero::eef::pick, des_pos, tra);
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
    std::vector<double> factors = {0.8, 0.8, 0.5};
    controller_->sendTrajectory(tra, calcTrajectoryTimes(tra, factors), aero::ikrange::wholebody);
    controller_->waitInterpolation();

    // grasp
    bool res = graspCoffee();

    // unreach
    aero::trajectory tra_back = {tra.at(1), tra.at(0)};
    controller_->sendTrajectory(tra_back, calcTrajectoryTimes(tra_back, factor), aero::ikrange::wholebody);
    controller_->waitInterpolation();

    // success or fail form grasp
    return res;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::graspCoffee() {
#ifdef DUMMY_MODE
    ROS_INFO("%s: DUMMY_MODE, grasping always returns true", __FUNCTION__);
      setUsingArm(aero::arm::rarm);
    return true;
#endif
    controller_->sendGrasp(aero::arm::rarm, 50);
    usleep(500 * 1000);
    controller_->setRobotStateToCurrentState();
    double index = controller_->kinematic_state->getVariablePosition("r_indexbase_joint");
    double thumb = controller_->kinematic_state->getVariablePosition("r_thumb_joint");
    ROS_INFO("%s: index %f", __FUNCTION__, index);
    if (index < 0.05) {
      ROS_INFO("%s: success", __FUNCTION__);
      setUsingArm(aero::arm::rarm);
      return true;
    }
    ROS_WARN("%s: failed", __FUNCTION__);
    openHand(aero::arm::rarm);
    unsetUsingArm(aero::arm::rarm);
    return false;
  }

  //////////////////////////////////////////////////////////
  bool DevelLib::placeCoffee(Eigen::Vector3d _pos, double _offset_y, aero::arm _arm) {
    controller_->setPoseVariables(aero::pose::reset);
    controller_->resetLookAt();
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendAngleVector(calcPathTime(av, 0.7));
    controller_->waitInterpolation();

    double factor = 0.7;

    // make trajectory
    bool res; aero::trajectory tra;
    res = makeTopGrasp(_arm, _pos, tra);

    if (!res) {
      ROS_WARN("%s: place ik failed", __FUNCTION__);
      return false;
    }

    if (_offset_y != 0) {
      ROS_INFO("%s: adjust y with wheel %f [m]", __FUNCTION__, _offset_y);
      controller_->goPos(0.0, _offset_y, 0.0);
    }

    std::vector<double> factors = {0.8, 0.8, 0.5};
    controller_->sendTrajectory(tra, calcTrajectoryTimes(tra, factors), aero::ikrange::wholebody);

    openHand(_arm);

    factors.clear();
    factors = {0.5, 0.8};
    aero::trajectory tra_release;
    tra_release.push_back(tra.at(1));
    tra_release.push_back(tra.at(0));
    controller_->sendTrajectory(tra_release, calcTrajectoryTimes(tra_release, factors), aero::ikrange::wholebody);

    return true;
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
    controller_->setPoseVariables(aero::pose::reset);
    controller_->setLifter(0.05, _lifter_z);
    controller_->setJoint(aero::joint::r_shoulder_y, -0.3);
    controller_->setJoint(aero::joint::l_shoulder_y, 0.3);
    controller_->setNeck(0.0, M_PI/ 2.0 ,0.0);
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendAngleVector(calcPathTime(av, 0.8),aero::ikrange::wholebody);
    controller_->waitInterpolation();
 }

  //////////////////////////////////////////////////////////
  void DevelLib::lookShelf() {
    controller_->setPoseVariables(aero::pose::reset);
    controller_->setLifter(0.0, -0.1);
    controller_->setJoint(aero::joint::r_shoulder_y, -0.3);
    controller_->setJoint(aero::joint::l_shoulder_y, -0.1);
    controller_->setNeck(0.0,M_PI/2.0,0.0);
    std::map<aero::joint, double> av;
    controller_->getRobotStateVariables(av);
    controller_->sendAngleVector(std::max(calcPathTime(av, 0.8), 1000), aero::ikrange::wholebody);
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
  bool DevelLib::fastestTrajectory3(const aero::arm _arm, const std::vector<aero::Transform> _poses, const aero::eef _eef, const Eigen::Vector3d _des_pos, aero::trajectory& _tra) {
    // check poses length
    if (static_cast<int>(_poses.size()) != 3) {
      ROS_WARN("%s: poses legnth must be three! now %d", __FUNCTION__, static_cast<int>(_poses.size()));
      return false;
    }

    // make desired lifter
    Eigen::Vector3d last_pos = _poses.at(2).translation();
    // lifter limit
    double z_max = 0.0, z_min= -0.4;
    // when use lifter x
    double z_op_max = -0.08, z_op_min = -0.25, x_op_max = 0.17, x_op_min = -0.17;
    controller_->setLifter(0.0, 0.0);
    Eigen::Vector3d waist = controller_->getWaistPosition();
    //desired on lifter top
    double x_des = _des_pos.x(), y_des = _des_pos.y(), z_des = _des_pos.z() + waist.z();

    // calc ideal lifter position
    double lifter_x_ideal = last_pos.x() - x_des;
    double lifter_z_ideal = last_pos.z() - z_des;
    // limit actual lifter restriction
    double lifter_x, lifter_z;
    if (lifter_z_ideal <= z_op_max &&  lifter_z_ideal >= z_op_min) {
      // in on plane mode
      lifter_z = lifter_z_ideal;
      lifter_x = std::max(x_op_min, std::min(x_op_max, lifter_x_ideal));
    } else { // in height only mode
      lifter_z = std::max(z_min, std::min(z_max, lifter_z_ideal));
      lifter_x = 0.0;
    }
    // set to model and get initial pose
    std::map<aero::joint, double> reset_pose;
    controller_->getResetManipPose(reset_pose);
    controller_->setLifter(lifter_x, lifter_z);

    // solve entry
    aero::trajectory ends, mids, entrys;
    controller_->setRobotStateVariables(reset_pose);
    if (!controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(0), _eef)) {
      ROS_WARN("%s: entry ik failed", __FUNCTION__);
      ROS_WARN("pos: x:%f y:%f z:%f", _poses.at(0).translation().x(), _poses.at(0).translation().y(), _poses.at(0).translation().z());
      return false;
    } else {
      std::map<aero::joint, double> tmp;
      controller_->getRobotStateVariables(tmp);
      entrys.push_back(tmp);
    }

    // solve mid
    controller_->setRobotStateVariables(reset_pose);
    if (!controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(1), _eef)) {
      ROS_WARN("%s: mid ik failed", __FUNCTION__);
      ROS_WARN("pos: x:%f y:%f z:%f", _poses.at(1).translation().x(), _poses.at(1).translation().y(), _poses.at(1).translation().z());
      return false;
    } else {
      std::map<aero::joint, double> tmp;
      controller_->getRobotStateVariables(tmp);
      mids.push_back(tmp);
    }

    // solve end
    controller_->setRobotStateVariables(reset_pose);
    if (!controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(2), _eef)) {
      ROS_WARN("%s: end ik failed", __FUNCTION__);
      ROS_WARN("pos: x:%f y:%f z:%f", _poses.at(2).translation().x(), _poses.at(2).translation().y(), _poses.at(2).translation().z());
      return false;
    } else {
      std::map<aero::joint, double> tmp;
      controller_->getRobotStateVariables(tmp);
      ends.push_back(tmp);
    }

    // make iks from another state
    controller_->setRobotStateVariables(ends.at(0));
    if (controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(1), _eef)) {
      std::map<aero::joint, double> tmp1,tmp2;
      controller_->getRobotStateVariables(tmp1);
      mids.push_back(tmp1);
      if (controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(0), _eef)) {
        controller_->getRobotStateVariables(tmp2);
        entrys.push_back(tmp2);
      }
    }

    controller_->setRobotStateVariables(entrys.at(0));
    if (controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(1), _eef)) {
      std::map<aero::joint, double> tmp1,tmp2;
      controller_->getRobotStateVariables(tmp1);
      mids.push_back(tmp1);
      if (controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(2), _eef)) {
        controller_->getRobotStateVariables(tmp2);
        ends.push_back(tmp2);
      }
    }

    controller_->setRobotStateVariables(mids.at(0));
    if (controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(0), _eef)) {
      std::map<aero::joint, double> tmp;
      controller_->getRobotStateVariables(tmp);
      entrys.push_back(tmp);
    }

    controller_->setRobotStateVariables(mids.at(0));
    if (controller_->setFromIK(_arm ,aero::ikrange::upperbody, _poses.at(2), _eef)) {
      std::map<aero::joint, double> tmp;
      controller_->getRobotStateVariables(tmp);
      ends.push_back(tmp);
    }

    // search fastest path
    double time_factor = 0.5;
    int min_time = std::numeric_limits<int>::max();
    std::vector<int> indices;
    for (int i = 0; i < static_cast<int>(entrys.size()); ++i)
      for (int j = 0; j < static_cast<int>(mids.size()); ++j)
        for (int k = 0; k < static_cast<int>(ends.size()); ++k) {
          int time = calcPathTime(entrys.at(i), mids.at(j), time_factor);
          time += calcPathTime(mids.at(j), ends.at(k), time_factor);
          if (time < min_time) {
            min_time = time;
            indices = {i, j, k};
          }
        }

    // return
    _tra.clear();
    _tra.reserve(3);
    _tra.push_back(entrys.at(indices.at(0)));
    _tra.push_back(mids.at(indices.at(1)));
    _tra.push_back(ends.at(indices.at(2)));

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
