#include "negomo/NegomoLib.hh"

using namespace negomo_lib;

//////////////////////////////////////////////////
NegomoBridge::NegomoBridge
(ros::NodeHandle _nh, std::string _ns, size_t _p, size_t _r, bool _peekauto)
  : nh_(_nh), action_spinner_(1, &action_queue_),
    status_spinner_(1, &status_queue_),
    partial_result_spinner_(1, &partial_result_queue_),
    peekauto_spinner_(1, &peekauto_queue_),
    optional_headpos_spinner_(1, &optional_headpos_queue_),
    lib_spinner_(1, &lib_queue_),
    optional_speech_spinner_(1, &optional_speech_queue_)
{
  ROS_WARN("Remember to set key 'return' in action callback.");
  for (int i = 0; i < _p; ++i)
    ROS_WARN("Remember to set key 'proactive%d' in action callback.", i);
  for (int i = 0; i < _r; ++i)
    ROS_WARN("Remember to set key 'reactive%d' in action callback.", i);

  ns_ = _ns;

  do_action_ =
    nh_.serviceClient<negomo_enshu::RobotAction>("/negomo/preinteraction");

  negomo_bridge_ = nh_.serviceClient<negomo_enshu::NegomoService>(ns_ + "call");
  negomo_intermediate_bridge_ =
    nh_.serviceClient<negomo_enshu::NegomoService>(ns_ + "intermediate/call");
  negomo_action_finished_ =
    nh_.advertise<std_msgs::Empty>(ns_ + "done_action", 1);
  force_client_ =
    nh_.serviceClient<negomo_enshu::NegomoService>(ns_ + "force/during");

  ros::AdvertiseServiceOptions action_ops =
    ros::AdvertiseServiceOptions::create<negomo_enshu::RobotAction>(
        ns_ + "do_action",
        boost::bind(&NegomoBridge::ActionCallback, this, _1, _2),
        ros::VoidPtr(),
        &action_queue_
        );
  negomo_actions_ = nh_.advertiseService(action_ops);
  action_spinner_.start();

  status_.negotiating = false;
  status_.state = "perceptual";
  ros::SubscribeOptions status_ops =
    ros::SubscribeOptions::create<negomo_enshu::NegomoStatus>(
        ns_ + "flag",
        10,
        boost::bind(&NegomoBridge::StatusCallback, this, _1),
        ros::VoidPtr(),
        &status_queue_
        );
  negomo_status_ = nh_.subscribe(status_ops);
  status_spinner_.start();

  ros::SubscribeOptions partial_result_ops =
    ros::SubscribeOptions::create<negomo_enshu::NegomoPlot>(
        ns_ + "plot",
        10,
        boost::bind(&NegomoBridge::PartialResultCallback, this, _1),
        ros::VoidPtr(),
        &partial_result_queue_
        );
  negomo_partial_result_ = nh_.subscribe(partial_result_ops);
  partial_result_spinner_.start();

  peekauto_setup_ = false;
  peekauto_timer_ = std::chrono::high_resolution_clock::now();
  peekauto_result_ = -4;
  ros::SubscribeOptions peekauto_ops =
    ros::SubscribeOptions::create<std_msgs::String>(
        ns_ + "setduringaction",
        10,
        boost::bind(&NegomoBridge::SetDuringActionCallback, this, _1),
        ros::VoidPtr(),
        &peekauto_queue_
        );
  set_during_action_ = nh_.subscribe(peekauto_ops);
  peekauto_spinner_.start();

  peekauto_ = _peekauto;
  preinteractionid_ = 0;

  // head pos listener no longer optional, used for checking no person in check()
  startHeadposListener();
}

//////////////////////////////////////////////////
NegomoBridge::~NegomoBridge()
{
}

//////////////////////////////////////////////////
negomo_enshu::BridgeRequest::Response NegomoBridge::breakfrom
(std::string _target, std::string _method)
{
  negomo_enshu::BridgeRequest::Response res;
  bool flush_interaction = false; // const variable

  // Set default parameters.
  if (_target == "")
    _target = "anonymous_score"; // select anonymous target by score

  // Bridge to negomo situation engine.
  negomo_enshu::NegomoService srv;
  srv.request.timestamp = ros::Time::now();
  srv.request.context = "continue task?";
  srv.request.target = _target;
  srv.request.function = "reactive";
  srv.request.method = _method;
  srv.request.flush_interaction = flush_interaction;
  if (!negomo_bridge_.call(srv)) {
    ROS_ERROR("[breakfrom] Fatal! Call to negomo has failed!");
    res.proceed = -4;
    res.status_change = 0;
    return res;
  }

  // If possibility of conflict detected, negomo process required.
  if (srv.response.result == "postpone") {
    // Negomo under process.
    bool status = true;
    std::string state = "";
    while (status) {
      status_mutex_.lock();
      status = status_.negotiating;
      state = status_.state;
      status_mutex_.unlock();
    }

    // If human > robot situation, robot should postpone task.
    if (state == "proactive") {
      // Tell client to proceed to reactive task.
      res.proceed = 0;
      res.status_change = 1;
      // Usually, breakfrom should be called again in this situation.
      return res;
    }

    // No conflict, however, robot state has changed during negomo.
    // Tell client to proceed but make sure to conduct state-return if required.
    res.proceed = 1;
    res.status_change = 1;
    // Usually, BreakRequest should be called again in this situation.
    return res;
  }

  // Conflict not detected, negomo process can be skipped.

  // If break called during proactive task and proactive task is able to proceed
  if (srv.response.result == "push") {
    res.proceed = 2;
    res.status_change = 0;
    return res;
  }

  // If after interaction or any other unusual state.
  if (flush_interaction || !(srv.response.result == "proceed")) {
    // Tell client to proceed but make sure to conduct state-return if required.
    res.proceed = 1;
    res.status_change = 1;
    // Usually, BreakRequest should be called again in this situation.
    return res;
  }

  // All is going well.
  res.proceed = 1;
  res.status_change = 0;
  return res;
}

//////////////////////////////////////////////////
int NegomoBridge::peek(std::string _action)
{
  // if (peekauto_) {
  //   ROS_WARN("illegal flag at initialization to call peek(std::string).");
  //   std::exit(0);
  // }

  // Target must be current target under interaction.

  // Bridge to negomo situation engine.
  negomo_enshu::NegomoService srv;
  srv.request.timestamp = ros::Time::now();
  // must pass action type
  srv.request.function = _action;
  if (!negomo_intermediate_bridge_.call(srv)) {
    ROS_ERROR("[peek] Fatal! Call to negomo has failed!");
    return -4;
  } else if (srv.response.result == "bad") {
    return -1;
  }

  // Get response.
  auto delim_pos = srv.response.result.find(":");
  std::string state = srv.response.result.substr(0, delim_pos);
  float confidence = std::stof(srv.response.result.substr
                               (delim_pos + 1, srv.response.result.length()));

  if (state == "n" && confidence > 0.95) {
    // automatic flushing of interaction (somewhat essential)
    srv.request.flush_interaction = true;
    if (!negomo_intermediate_bridge_.call(srv)) {
      ROS_ERROR("[peek] Fatal! Call to negomo has failed!");
      return -4;
    }
    return 0;
  }

  return 1;
}

//////////////////////////////////////////////////
bool NegomoBridge::peek()
{
  if (!peekauto_) {
    ROS_WARN("illegal call to peek(). flag not set at initialization.");
    std::exit(0);
  }

  peekauto_mutex_.lock();
  int proceed = peekauto_result_;
  peekauto_mutex_.unlock();

  return proceed;
}

//////////////////////////////////////////////////
void NegomoBridge::PeekInOut(bool _flag)
{
  peekauto_mutex_.lock();
  if (_flag)
    peekauto_result_ = 1; // set value for first 3 sec before update
  peekauto_setup_ = _flag;
  peekauto_timer_ = std::chrono::high_resolution_clock::now();
  peekauto_mutex_.unlock();
}

//////////////////////////////////////////////////
int NegomoBridge::force()
{
  negomo_enshu::NegomoService srv;
  srv.request.timestamp = ros::Time::now();
  if (!force_client_.call(srv)) {
    ROS_ERROR("[force] Fatal! Call to negomo has failed!");
    return -4;
  } else if (srv.response.result == "no target") {
    ROS_WARN("[force] no one to interact with in forcing");
    return 0;
  } else if (srv.response.result == "not found") {
    // currently not supported
    ROS_WARN("[force] the target to interact was not found in forcing");
    return -1;
  }

  if (peekauto_)
    peekin();

  return 1;
}

//////////////////////////////////////////////////
int NegomoBridge::away()
{
  if (peekauto_)
    peekout();

  // Bridge to negomo situation engine and force-end during interaction.
  negomo_enshu::NegomoService srv;
  srv.request.timestamp = ros::Time::now();
  srv.request.flush_interaction = true;
  if (!negomo_intermediate_bridge_.call(srv)) {
    ROS_ERROR("[away] Fatal! Call to negomo has failed!");
    return -4;
  } else if (srv.response.result == "bad") {
    ROS_WARN("[away] during interaction already finished, but away was called");
    return 0;
  }

  return 1;
}

//////////////////////////////////////////////////
std::string NegomoBridge::prepare
(std::string _target, bool _start_negotiation, std::string _method)
{
  if (_target == "")
    _target = "anonymous";

  negomo_enshu::NegomoService srv;
  srv.request.timestamp = ros::Time::now();
  srv.request.context = "prepare interaction?";
  srv.request.target = _target;
  if (_start_negotiation) {
    srv.request.function = "proactive";
    srv.request.method = _method;
  }
  if (!negomo_bridge_.call(srv)) {
    ROS_ERROR("[prepare] unexpected proactive prepare error!");
    return "";
  } else if (srv.response.result == "abort") {
    return "";
  }

  return srv.response.result;
}

//////////////////////////////////////////////////
int NegomoBridge::tryto(std::string _target, std::string _method)
{
  // For reactive check, please call Break before Try.
  // Unlike negomo_smach, negomo_bridge allows Try without reactive check.
  // However, if negotiation fails, node will instruct to proceed to reactive.

  // Set default parameters.
  if (_target == "")
    _target = "anonymous"; // select anonymous target

  // Bridge to negomo situation engine.
  negomo_enshu::NegomoService srv;
  srv.request.timestamp = ros::Time::now();
  srv.request.context = "interaction ok?";
  srv.request.target = _target;
  srv.request.function = "proactive";
  srv.request.method = _method;
  srv.request.flush_interaction = false;
  if (!negomo_bridge_.call(srv)) {
    ROS_ERROR("[tryto] Fatal! Call to negomo has failed!");
    return -4;
  }

  // If interaction target was not found.
  if (srv.response.result == "abort")
    return -2;

  // If target was found, wait for negomo result.
  bool status = true;
  std::string state = "";
  while (status) {
    status_mutex_.lock();
    status = status_.negotiating;
    state = status_.state;
    status_mutex_.unlock();
  }

  // If human -> robot situation, robot should postpone task.
  if (state == "proactive")
    // Tell client to proceed to reactive task.
    return 0;
    // Do a return_func, re Try, etc.
  else if (state == "reactive")
    // Tell client to proceed to a proactive subtask.
    return 1;
  else
    // Proceeding to subtask had unexpected error (person busy or lost). Postpone.
    return -1;
}

//////////////////////////////////////////////////
bool NegomoBridge::check(float _threshold, int _target, int _options)
{
  negomo_enshu::PartialResultRequest::Response res;
  return check(res, _threshold, _target, _options);
}

//////////////////////////////////////////////////
bool NegomoBridge::check(negomo_enshu::PartialResultRequest::Response &_res,
                         float _threshold, int _target, int _options)
{
  partial_result_mutex_.lock();
  for (auto it = partial_result_.begin(); it != partial_result_.end(); ++it) {
    if (it->first == std::numeric_limits<int>::max()) // no person
      continue;
    _res.ids.push_back(it->first);
    _res.states.push_back(it->second.first);
    _res.confidences.push_back(it->second.second);
  }
  for (auto it = partial_is_target_.begin(); it != partial_is_target_.end(); ++it) {
    if (it->second) { // there is a current target, set this as default target
      if (_target < 0 && it->first != std::numeric_limits<int>::max())
        _target = it->first;
      break;
    }
  }
  partial_result_mutex_.unlock();

  std::function<bool(int, float)> condition;
  if (_options == 0) // confidence from how much person is interested
    condition = [&](int _state, float _confidence) {
      return (_confidence > _threshold && (_state == 1 || _state == -1));
    };
  else // confidence from how much person is not interested
    condition = [&](int _state, float _confidence) {
      return (_confidence < _threshold && _state == 0);
    };

  _res.status = false;
  if (_target < 0) { // any target
    for (size_t i = 0; i < _res.ids.size(); ++i) {
      int state = _res.states.at(i);
      float confidence = _res.confidences.at(i);
      ROS_INFO("    found: %d, %f", state, confidence);
      if (condition(state, confidence)) {
        _res.status = true;
        break;
      }
    }
  } else {
    auto id_p =
      std::find(_res.ids.begin(), _res.ids.end(), _target);
    if (id_p != _res.ids.end()) {
      int id = static_cast<int>(id_p - _res.ids.begin());
      int state = _res.states.at(id);
      float confidence = _res.confidences.at(id);
      if (condition(state, confidence))
        _res.status = true;
    }
  }

  return _res.status;
}

//////////////////////////////////////////////////
void NegomoBridge::StatusCallback(const negomo_enshu::NegomoStatus::ConstPtr _status)
{
  status_mutex_.lock();
  status_.negotiating = _status->negotiating;
  status_.state = _status->state;
  status_mutex_.unlock();
}

//////////////////////////////////////////////////
void NegomoBridge::PartialResultCallback(const negomo_enshu::NegomoPlot::ConstPtr _msg)
{
  partial_result_mutex_.lock();
  if (_msg->timelinep.size() == 0) {
    partial_result_[_msg->target] = {0, _msg->intentr.back()};
  } else if (_msg->timeliner.size() == 0) {
    partial_result_[_msg->target] = {1, _msg->intentp.back()};
  } else if (_msg->timelinep.back() >= _msg->timeliner.back()) {
    if (_msg->intentp.back() > 0) // proactive
      partial_result_[_msg->target] = {1, _msg->intentp.back()};
    else
      partial_result_[_msg->target] = {0, _msg->intentr.back()};
  } else {
    if (_msg->intentr.back() < 0) // reactive
      partial_result_[_msg->target] = {-1, -_msg->intentr.back()};
    else
      partial_result_[_msg->target] = {0, _msg->intentr.back()};
  }
  // check who is target
  if (_msg->color == "b")
    partial_is_target_[_msg->target] = false;
  else
    partial_is_target_[_msg->target] = true;
  partial_result_mutex_.unlock();
}

//////////////////////////////////////////////////
void NegomoBridge::SetDuringActionCallback(const std_msgs::String::ConstPtr _msg)
{
  // check if interaction is in during
  peekauto_mutex_.lock();
  bool setup =
    (peekauto_setup_ &&
     std::chrono::duration_cast<std::chrono::milliseconds>
     (std::chrono::high_resolution_clock::now() - peekauto_timer_).count()
     > 3000); // must omit first 3 seconds or observation will be null
  peekauto_mutex_.unlock();
  if (!setup) // interaction not in during, return
    return;

  int proceed = peek(_msg->data);
  peekauto_mutex_.lock();
  peekauto_result_ = proceed;
  if (peekauto_result_ <= 0) // escape during
    peekauto_setup_ = false;
  peekauto_mutex_.unlock();
}

//////////////////////////////////////////////////
bool NegomoBridge::ActionCallback(negomo_enshu::RobotAction::Request &_req,
                                  negomo_enshu::RobotAction::Response &_res)
{
  if (_req.action == "nperceptual0") {
    // _res.time_ms = 0.0;
    std_msgs::Empty msg;
    negomo_action_finished_.publish(msg);
    return true;
  }

  std::string action;
  int level;

  // assumes action level to be less than 100
  if (isdigit(_req.action.at(_req.action.length() - 2))) {
    action = _req.action.substr(0, _req.action.length() - 2);
    level = std::stoi(_req.action.substr(_req.action.length() - 2, 2));
  } else {
    action = _req.action.substr(0, _req.action.length() - 1);
    level = std::stoi(_req.action.substr(_req.action.length() - 1, 1));
  }

  preinteractionid_mutex_.lock();
  std::string preinteractionid = std::to_string(preinteractionid_);
  preinteractionid_mutex_.unlock();

  negomo_enshu::RobotAction srv;
  srv.request.target_id = _req.target_id;
  if (preinteractionid == "0")
    srv.request.action = ns_ + action + std::to_string(level);
  else
    srv.request.action = ns_ + action + std::to_string(level) + "/task" + preinteractionid;

  // run on background, as negomo must handle observations while doing action
  std::thread th([&](negomo_enshu::RobotAction _srv){
      if (!do_action_.call(_srv))
        ROS_ERROR("[ActionCallback] Failed to call pre-interaction action.");
      std_msgs::Empty msg;
      negomo_action_finished_.publish(msg); // notify action finish
    }, srv);
  th.detach();

  return true;
}

//////////////////////////////////////////////////
void NegomoBridge::startHeadposListener()
{
  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<geometry_msgs::PoseArray>(
        "/negomo/sensor/face/position/global",
        10,
        boost::bind(&NegomoBridge::OptionalHeadPositionCallback, this, _1),
        ros::VoidPtr(),
        &optional_headpos_queue_);
  optional_headpos_sub_ = nh_.subscribe(ops);
  optional_headpos_spinner_.start();
}

//////////////////////////////////////////////////
std::tuple<double,double,double> NegomoBridge::getHeadPos(int _i)
{
  std::tuple<double,double,double> res;
  optional_headpos_mutex_.lock();
  if (_i < optional_headpos_.size() && _i >= 0)
    res = optional_headpos_.at(_i);
  optional_headpos_mutex_.unlock();

  return res;
}

//////////////////////////////////////////////////
void NegomoBridge::OptionalHeadPositionCallback
(const geometry_msgs::PoseArray::ConstPtr &_msg)
{
  optional_headpos_mutex_.lock();
  optional_headpos_.resize(_msg->poses.size());
  auto h = optional_headpos_.begin();
  for (auto p = _msg->poses.begin(); p != _msg->poses.end(); ++p) {
    if (p->position.z < 0.001) { // no person
      partial_result_mutex_.lock();
      partial_result_[static_cast<int>(p - _msg->poses.begin())] = {std::numeric_limits<int>::max(), 0.0};
      partial_result_mutex_.unlock();
    }
    *h = std::make_tuple(p->position.x, p->position.y, p->position.z);
    ++h;
  }
  optional_headpos_mutex_.unlock();
}

//////////////////////////////////////////////////
void NegomoBridge::initlib(const boost::function<bool(negomo_enshu::RobotAction::Request &, negomo_enshu::RobotAction::Response &)> & _f)
{
  ros::AdvertiseServiceOptions ops =
    ros::AdvertiseServiceOptions::create<negomo_enshu::RobotAction>(
        "/negomo/preinteraction",
        _f,
        ros::VoidPtr(),
        &lib_queue_);
  lib_action_ = nh_.advertiseService(ops);
  lib_spinner_.start();
}

//////////////////////////////////////////////////
void NegomoBridge::startSpeechListener()
{
  optional_speech_spinner_.start();
}
