#include "negomo/NegomoPlannerLib.hh"

using namespace negomo_lib;

//////////////////////////////////////////////////
NegomoPlanner::NegomoPlanner
(ros::NodeHandle _nh, std::string _ns, ActionFunc _wsf, bool _use_default, bool _use_base)
  : nh_(_nh), uispinner_(1, &uiqueue_), defint_spinner_(1, &defint_queue_)
{
  waitinterpolation_client_ =
    nh_.serviceClient<negomo_enshu::WaitInterpolationRequest>(_ns + "interactivewi/begin");

  waitinterpolation_result_client_ =
    nh_.serviceClient<negomo_enshu::WaitInterpolationRequest>(_ns + "interactivewi/result");

  planner_interaction_client_ =
    nh_.serviceClient<negomo_enshu::PlannerDefaultInteractionCall>(_ns + "interactivewi/defaultinteraction");

  vppub_cleanall_ =
    nh_.advertise<std_msgs::Empty>(_ns + "vp/cleanall", 10);

  vppub_activate_ =
    nh_.advertise<negomo_enshu::VpActivate>(_ns + "vp/activate", 10);

  vppub_addtoqueue_ =
    nh_.advertise<std_msgs::Int32>(_ns + "vp/addToQueue", 10);

  vppub_deactivate_ =
    nh_.advertise<std_msgs::Empty>(_ns + "vp/deactivate", 10);

  vppub_update_ =
    nh_.advertise<std_msgs::Int32>(_ns + "vp/update", 10);

  vppub_toqueue_ =
    nh_.advertise<std_msgs::Int32>(_ns + "vp/toQueue", 10);

  vppub_enterinterruption_ =
    nh_.advertise<negomo_enshu::VpConnect>(_ns + "vp/enterInterruption", 10);

  vppub_enterexception_ =
    nh_.advertise<std_msgs::Empty>(_ns + "vp/enterException", 10);

  vppub_entertemp_ =
    nh_.advertise<std_msgs::String>(_ns + "vp/enterTemp", 10);

  initTask =
    std::bind(&negomo_lib::NegomoPlanner::initTask_, this,
              std::placeholders::_1, std::placeholders::_2);

  moveToWorkspace = _wsf;

  emptyAction =
    std::bind(&negomo_lib::NegomoPlanner::emptyAction_, this,
              std::placeholders::_1, std::placeholders::_2);

  loopStart =
    std::bind(&negomo_lib::NegomoPlanner::loopStart_, this,
              std::placeholders::_1, std::placeholders::_2);

  loopEnd =
    std::bind(&negomo_lib::NegomoPlanner::loopEnd_, this,
              std::placeholders::_1, std::placeholders::_2);

  finishTask =
    std::bind(&negomo_lib::NegomoPlanner::finishTask_, this,
              std::placeholders::_1, std::placeholders::_2);

  uiExceptionHandle =
    std::bind(&negomo_lib::NegomoPlanner::uiExceptionHandle_, this,
              std::placeholders::_1, std::placeholders::_2);

  tOff =
    std::bind(&negomo_lib::NegomoPlanner::tOff_, this,
              std::placeholders::_1, std::placeholders::_2);

  if (_use_default) {
    ros::AdvertiseServiceOptions interactops =
      ros::AdvertiseServiceOptions::create<negomo_enshu::PlannerInteractionCall>(
          _ns + "interactivewi/interact",
          boost::bind(&NegomoPlanner::shortInteraction, this, _1, _2),
          ros::VoidPtr(),
          &defint_queue_);
    defint_interact_server_ = nh_.advertiseService(interactops);
    defint_spinner_.start();
  }

  base_ = _use_base;

  // initiate member variables

  curentid_ = -1;
  curaction_ = -1;
  num_capabilities_ = -1;
  exception_escaped_id_ = -1;
  created_tmpentity_ = -1;
  wildid_ = 0;
  ioff_ = false;
  usedhands_ = 0;
  waitfor_iJoin = false;

  // initiate viewer

  usleep(1000 * 1000); // wait for publisher to be ready
  std_msgs::Empty msg;
  vppub_cleanall_.publish(msg);
  usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep

  // create viewer ui

  uipub_ = nh_.advertise<std_msgs::String>(_ns + "vp/uistart", 10);

  ros::SubscribeOptions ops =
    ros::SubscribeOptions::create<std_msgs::String>(
        _ns + "vp/userinput",
        10,
        boost::bind(&NegomoPlanner::uiCallback, this, _1),
        ros::VoidPtr(),
        &uiqueue_);
  uispinner_.start();
  uisub_ = nh_.subscribe(ops);

  // for node version

  callaction_client_ =
    nh_.serviceClient<negomo_enshu::PlannerActionCall>(_ns + "planner/callaction");

  callexception_client_ =
    nh_.serviceClient<negomo_enshu::PlannerActionCall>(_ns + "planner/callexception");

  calltemp_client_ =
    nh_.serviceClient<negomo_enshu::PlannerActionCall>(_ns + "planner/calltemp");

  callAction =
    std::bind(&negomo_lib::NegomoPlanner::callAction_, this,
              std::placeholders::_1, std::placeholders::_2);

  callException =
    std::bind(&negomo_lib::NegomoPlanner::callException_, this,
              std::placeholders::_1, std::placeholders::_2);

  callTemp =
    std::bind(&negomo_lib::NegomoPlanner::callTemp_, this,
              std::placeholders::_1, std::placeholders::_2);

  callRevAction =
    std::bind(&negomo_lib::NegomoPlanner::callRevAction_, this,
              std::placeholders::_1, std::placeholders::_2);

  callRevException =
    std::bind(&negomo_lib::NegomoPlanner::callRevException_, this,
              std::placeholders::_1, std::placeholders::_2);

  callRevTemp =
    std::bind(&negomo_lib::NegomoPlanner::callRevTemp_, this,
              std::placeholders::_1, std::placeholders::_2);

  callMoveTo =
    std::bind(&negomo_lib::NegomoPlanner::callMoveTo_, this,
              std::placeholders::_1, std::placeholders::_2);
}

//////////////////////////////////////////////////
NegomoPlanner::~NegomoPlanner()
{
}

//////////////////////////////////////////////////
void NegomoPlanner::iOff()
{
  if (waitfor_iJoin) {
    ROS_ERROR("Illegal iOff called between iStart and iJoin!");
    std::exit(0);
  }
  ioff_ = true;
}

//////////////////////////////////////////////////
void NegomoPlanner::iOn()
{
  if (waitfor_iJoin) {
    ROS_ERROR("Illegal iOn called between iStart and iJoin!");
    std::exit(0);
  }
  ioff_ = false;
}

//////////////////////////////////////////////////
void NegomoPlanner::iOnTimered(int _ms)
{
  if (waitfor_iJoin) {
    ROS_ERROR("Illegal iOnTimered called between iStart and iJoin!");
    std::exit(0);
  }
  timer_ = true;
  timertime_ = _ms;
  timerstart_ = std::chrono::high_resolution_clock::now();
}

//////////////////////////////////////////////////
void NegomoPlanner::run(std::vector<negomo_lib::ActionList> _als, ActionList *_el, ActionList *_tl)
{
  while (ros::ok()) {
    int task, from=0;
    if (getTaskFromQueue(task, from)) {
      ROS_INFO("          task is %d", task);
      ROS_INFO("          from is %d", from);
      runTask(task, _als, _el, _tl, from);
    } else {
      ROS_WARN("          task is %d", task);
      ROS_WARN("          from is %d", from);
      break;
    }
  }
}

//////////////////////////////////////////////////
void NegomoPlanner::runTask
(int _entid, std::vector<negomo_lib::ActionList> _als, ActionList *_el, ActionList *_tl, int _from)
{
  int next = _entid;
  int from = _from;
  while (next != -1) {
    if (next < _als.size()) { // if default task
      next = runTask(next, &_als.at(next), _el, _tl, from);
    } else {
      boost::optional<int> taskid = entities_[next].get<int>("taskid");
      if (taskid && taskid.get() < _als.size()) { // custom task
        next = runTask(next, &_als.at(taskid.get()), _el, _tl, from);
      } else { // unexpected
        ROS_ERROR("access to bad task id %d", next);
        std::exit(0);
      }
    }
    from = 0; // next task starts from head
  }
}

//////////////////////////////////////////////////
int NegomoPlanner::runTask
(int _entid, ActionList* _al, ActionList* _el, ActionList* _tl, int _from)
{
  // setup for vp debug
  negomo_enshu::VpActivate vpmsg;
  vpmsg.entityid = _entid;
  for (auto a = _al->begin(); a != _al->end(); ++a)
    vpmsg.actions.push_back(std::get<4>(*a));
  for (auto e = _el->begin(); e != _el->end(); ++e)
    vpmsg.exceptions.push_back(std::get<4>(*e));
  for (auto t = _tl->begin(); t != _tl->end(); ++t)
    vpmsg.temps.push_back(std::get<4>(*t));
  vppub_activate_.publish(vpmsg);
  usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep

  // main code starts here

  actionlist_ = std::make_shared<ActionList>(*_al);

  curentid_ = _entid;
  curaction_ = _from - 1;
  priority_ = 2;

  // if non-created task, set taskid (used for createTask)
  if (_entid < num_capabilities_)
    getEntities().put("taskid", curentid_);

  int nexttask = -1;

  // check if this task ended with a temp action
  auto tmpaction = std::find_if(tmpq_.begin(), tmpq_.end(),
      [&](std::pair<int, std::stack<ActionFunc>> a){return (a.first == curentid_);});
  if (tmpaction != tmpq_.end()) {
    setMaxPriority();
    taction_ = tmpaction->second.size(); // for node version
    while (tmpaction->second.size() > 0) {
      --taction_;
      usedhands_ = tmpaction->second.top()(usedhands_, nexttask);
      if (nexttask == -404) { // we wish this does not happen
        vpException();
        // priority is 0, cannot be interrupted as urgent
        for (auto e = _el->begin(); e != _el->end(); ++e) {
          usedhands_ = std::get<2>(*e)(usedhands_, nexttask);
          if (nexttask == -404) { // exception task finished with error
            // task is stopped at temp, queue
            taskq_.push_back({curentid_, _from});
            exception_escaped_id_ = curentid_;
            vpQueue(); // change vp state to queue
            return -1;
          }
          vpUpdate();
        } // exception
      } // -404
      tmpaction->second.pop();
      vpUpdate();
    } // tmpq_
    tmpq_.erase(tmpaction); // remove from tmpq
    priority_ = 2; // reset priority from 0
  }

  auto a = actionlist_->begin() + _from;
  while (true) {
    ++curaction_;
    a = actionlist_->begin() + curaction_;
    if (a == actionlist_->end()) {
      // finish up for vp debug
      std_msgs::Empty vpmsg;
      vppub_deactivate_.publish(vpmsg);
      usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
      return nexttask;
    }

    vpUpdate(curaction_);
    usedhands_ = std::get<2>(*a)(usedhands_, nexttask); // do action

    if (nexttask >= 0) { // task w/ higher priority came in
      vpQueue(); // change this task to queue in vp
      return nexttask;
    } else if (nexttask == -404) { // detected exception
      // when an exception is interrupted,
      // next time task will restart from action where error occured
      // why? someone else other than robot may fix state while robot gone

      // start exception
      vpException();
      nexttask = -1; // set to continue exception
      eaction_ = -1; // for node version
      for (auto e = _el->begin(); e != _el->end(); ++e) {
        ++eaction_; // for node version
        usedhands_ = std::get<2>(*e)(usedhands_, nexttask);
        if (nexttask >= 0) { // interruption during error
          if (usedhands_ > 0) { // go to temp task to set hands->0
            vpTemp();
            std::pair<int, std::stack<ActionFunc> > tmp;
            tmp.first = curentid_;
            setMaxPriority(); // do not allow further interruption
            int dummynext; // if use nexttask, will be overwritten w/ iWaitAction
            taction_ = -1; // for node version
            for (auto t = _tl->begin(); t != _tl->end(); ++t) {
              ++taction_; // for node version
              usedhands_ = std::get<2>(*t)(usedhands_, dummynext);
              tmp.second.push(std::get<3>(*t));
              if (dummynext == -404)
                // expects temp forward action never fails
                ROS_WARN("[warn] A temp forward action should never fail!");
              vpUpdate();
            }
            tmpq_.push_back(tmp); // save information that task ended with tmp
          } else {
            pushMoveTemp(nexttask); // add move action
          }
          vpQueue();
          return nexttask;
        }

        // check end of exception state
        if (nexttask == -404) { // exception task finished with error
          // likely, could not find person who could help -> postpone this task
          taskq_.push_back({curentid_, curaction_});
          exception_escaped_id_ = curentid_;
          vpQueue();
          return -1;
        }
        vpUpdate();
      } // exception

      // else, exception finished with no error, continue
    } // -404
  } // actionlist_
}

//////////////////////////////////////////////////
void NegomoPlanner::noPlanning() {
  actionlist_.reset(new ActionList({}));
  curaction_ = 0;
}

//////////////////////////////////////////////////
bool NegomoPlanner::getTaskFromQueue(int &_task, int &_from)
{
  if (taskq_.size() == 0 && defaultq_.size() == 0)
    return false;

  // first, try getting task stopped at some state
  for (auto t = taskq_.begin(); t != taskq_.end(); ++t) {
    if (t->second > 0 && t->first != exception_escaped_id_) {
      _task = t->first;
      _from = t->second;
      t = taskq_.erase(t); // erase task from queue
      exception_escaped_id_ = -1; // exception escaped task valid next time
      return true;
    }
  }

  // next, try getting task from taskq
  for (auto t = taskq_.begin(); t != taskq_.end(); ++t)
    if (t->first != exception_escaped_id_) {
      _task = t->first;
      _from = t->second;
      taskq_.erase(taskq_.begin()); // erase task from queue
      exception_escaped_id_ = -1; // exception escaped task valid next time
      return true;
    }

  // if no valid task in taskq, try getting from defaultq
  if (defaultq_.size() > 0) {
    if (taskq_.size() > 0 &&
        entities_[defaultq_.begin()->first].get<bool>("blocked", false)) {
      // if all other tasks must be finished first, exception task has priority
    } else {
      _task = defaultq_.begin()->first;
      _from = defaultq_.begin()->second;
      defaultq_.erase(defaultq_.begin()); // erase task from queue
      exception_escaped_id_ = -1; // exception escaped task valid next time
      return true;
    }
  }

  // if only tasks with exceptions
  _task = taskq_.begin()->first;
  _from = taskq_.begin()->second;
  taskq_.erase(taskq_.begin()); // erase task from queue
  exception_escaped_id_ = -1; // exception escaped task valid next time

  return true;
}

//////////////////////////////////////////////////
bool NegomoPlanner::iStartSetup
(jumpSettings _js, waitSettings _ws, negomo_enshu::WaitInterpolationRequest &_srv)
{
  ROS_WARN("called iStart action: %d, flag %d", curaction_, _ws.interaction_flag);

  if (waitfor_iJoin) {
    ROS_ERROR("Illegal call to iStart without call to iJoin!");
    std::exit(0);
  }

  waitfor_iJoin = true; // for code safety

  // set iJoin settings
  iJoinset_at_end_ = _ws.at_end;
  iJoinset_backward_allowed_ = _ws.backward_allowed;
  // reset ignore hands before exiting
  iJoinset_ignore_used_hands_ = false;

  if (ioff_)
    return false;

  if (timer_ &&
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::high_resolution_clock::now() - timerstart_).count()
      < timertime_)
    return false;
  timer_ = false;

  // below updates taskq
  _srv.request.target = _ws.target;
  _srv.request.check_threshold = _ws.threshold;
  _srv.request.check_options = _ws.options;
  _srv.request.interact_only_once = _ws.interact_only_once;
  _srv.request.enable_preinteraction = _ws.enable_preinteraction;
  _srv.request.task_priority = priority_;
  _srv.request.warn_avoid = _ws.warn_avoid;
  if (_ws.warn_avoid)
    _srv.request.exec_time_ms = -1;
  else
    _srv.request.exec_time_ms = _ws.exec_time_ms;
  _srv.request.preinteractionid = _js.preaction;
  _srv.request.postinteractionid = _js.postaction_true;
  _srv.request.postfalseinteractionid = _js.postaction_false;

  // check interaction flag
  bool hand_occupied = false;
  if (_ws.interaction_flag == negomo_enshu::PlannerBridgeRequest::Request::IGNORE)
    iJoinset_ignore_used_hands_ = true;
  else if (_ws.interaction_flag == negomo_enshu::PlannerBridgeRequest::Request::OPENONE)
    for (size_t i = 0; i < actionlist_->size() - curaction_; ++i) {
      auto a = actionlist_->begin() + curaction_ + i;
      // TODO: right should be "max_hands - 1", current assumes dual arm
      if (std::get<1>(*a) <= 1) {
        if (i == 0 && _ws.at_end == 1)
          iJoinset_ignore_used_hands_ = true; // already meets requirement
        else
          hand_occupied = true;
        break;
      }
    }

  // do not accept interaction if queue is full or current is max priority
  if (taskq_.size() >= _ws.max_queue_size || priority_ == 0 || hand_occupied) {
    _srv.request.enable_preinteraction = false;
    _srv.request.warn_avoid = true;
    _srv.request.remaining_queue = 0;
  } else { // set remaining queue and current queue
    for (auto t = taskq_.begin(); t != taskq_.end(); ++t)
      _srv.request.queue.push_back(t->first);
    _srv.request.remaining_queue = _ws.max_queue_size - taskq_.size();
  }

  return true;
}

//////////////////////////////////////////////////
void NegomoPlanner::iStart(jumpSettings _js, waitSettings _ws)
{
  negomo_enshu::WaitInterpolationRequest srv;
  if (!iStartSetup(_js, _ws, srv))
    return;

  if (!waitinterpolation_client_.call(srv)) {
    ROS_ERROR("unexpected wait interpolation error!");
    waitinterpolation_call_failed_ = true;
  } else {
    waitinterpolation_call_failed_ = false;
  }
}

//////////////////////////////////////////////////
int NegomoPlanner::iJoin(int &_usedhands, int &_nexttask)
{
  iJoin(_usedhands, _nexttask, iJoinset_at_end_, iJoinset_backward_allowed_);
}

//////////////////////////////////////////////////
int NegomoPlanner::iJoin
(int &_usedhands, int &_nexttask, int _atEnd, int _backwardAllowed)
{
  ROS_WARN("called iJoin action: %d, hands: %d, ignore: %d",
           curaction_, _usedhands, static_cast<int>(iJoinset_ignore_used_hands_));

  if (!waitfor_iJoin) {
    ROS_ERROR("Illegal call to iJoin without iStart!");
    std::exit(0);
  }

  bool error = getEntities().get<bool>("error", false);
  backtrack_ = getEntities().get<std::string>("backtrack", "");
  setError(false, ""); // reset error states

  waitfor_iJoin = false;

  if (ioff_ || timer_) {
    if (error)
      _nexttask = -404;
    // note, waitinterpolation_client_ should never be called in this condition
    return _nexttask;
  }

  if (_atEnd == -1) // when in exception, force _backwardAllowed as -1
    _backwardAllowed = -1;

  if (base_)
    _nexttask = NegomoPlanner::getWaitInterpolation();
  else
    _nexttask = getWaitInterpolation();

  if (_nexttask == -404) {
    error = true;
  } else if (_nexttask != -1) { // start different task
    bool status;
    if (iJoinset_ignore_used_hands_) {
      int ignorehands = 0; // dummy usedhands
      status = // below updates -1 -> entid
        prepareForNext(_nexttask, ignorehands, error, _atEnd, _backwardAllowed);
    } else {
      status = // below updates -1 -> entid
        prepareForNext(_nexttask, _usedhands, error, _atEnd, _backwardAllowed);
    }

    // TODO: call leave action (workspace w/ waypoints) -> if fail error=true
    // get action from keyword 'leave'
    if (!status) { // failed setting usedhands->0
      backtrack_ =
        "Got additional task" + std::to_string(_nexttask) + " but could not escape current task.";
      // TODO: enter temp task which should not fail
    } else {
      pushMoveTemp(_nexttask);
      return _nexttask;
    }
  }

  if (error) {
    _nexttask = -404;
    return _nexttask; // go to error flag
  }

  return _nexttask;
}

//////////////////////////////////////////////////
void NegomoPlanner::setError(bool _error, std::string _backtrack)
{
  if (!waitfor_iJoin) {
    ROS_ERROR("Illegal call to setError not between iStart and iJoin!");
    std::exit(0);
  }

  getEntities().put("error", _error);
  getEntities().put("backtrack", _backtrack);
}

//////////////////////////////////////////////////
void NegomoPlanner::pushMoveTemp(int _nexttask)
{
  std::string workspace_next;
  boost::optional<std::string> try_workspace_next =
    entities_[_nexttask].get_optional<std::string>("workspace");
  if (!try_workspace_next) {
    if (_nexttask < num_capabilities_) {
      auto pos = std::find_if(default_entities_.at(_nexttask).begin(),
                              default_entities_.at(_nexttask).end(),
                              [](std::pair<std::string, std::string> _a) {
                                return (_a.first == "workspace");});
      if (pos == default_entities_.at(_nexttask).end()) {
        ROS_WARN("workspace entity should always be set in every task!");
        return; // do not push move temp
      }
      workspace_next = pos->second;
    } else {
      boost::optional<int> taskid =
        entities_[_nexttask].get_optional<int>("taskid");
      if (taskid) { // sometimes entity is default and not set yet
        auto pos = std::find_if(default_entities_.at(taskid.get()).begin(),
                                default_entities_.at(taskid.get()).end(),
                                [](std::pair<std::string, std::string> _a) {
                                  return (_a.first == "workspace");});
        if (pos == default_entities_.at(taskid.get()).end()) {
          ROS_WARN("workspace entity should always be set in every task!");
          return; // do not push move temp
        }
        workspace_next = pos->second;
      } else {
        ROS_WARN("very unusual... entity has no taskid");
        return;
      }
    }
  } else { // try_workspace_next
    workspace_next = try_workspace_next.get();
  }

  if (workspace_next == "HERE")
    // next task is same as this task
    return; // no need for move, so no push
  boost::optional<std::string> workspace_now =
    getEntities().get_optional<std::string>("workspace");

  if (workspace_now && workspace_next == workspace_now.get())
    return; // same workspace as current workspace

  if (std::get<4>(actionlist_->at(curaction_)) == "move") {
    ROS_WARN("action with keyword 'move' may not return to task correctly. try different keyword if planner is not acting as expected");
    return; // move action will be triggered when returned
  }

  // else, add a move action
  std::pair<int, std::stack<ActionFunc> > tmp;
  tmp.first = curentid_;
  tmp.second.push(this->moveToWorkspace);
  tmpq_.push_back(tmp);
  vpTemp("tmpmove");
}

//////////////////////////////////////////////////
std::vector<ActionFunc> NegomoPlanner::planToNext
(bool _exception, int _backwardAllowed, bool &_forward)
{
  int requiredHands = 0;

  // try planning backward
  std::vector<ActionFunc> resb;
  bool plandone = false;
  for (auto a = actionlist_->begin() + curaction_; a != actionlist_->begin(); --a) {
    resb.push_back(std::get<3>(*a));
    if (std::get<0>(*a) == requiredHands) {
      plandone = true;
      break;
    }
  }
  if (!plandone) {
    resb.push_back(std::get<3>(actionlist_->at(0)));
  }

  if (_exception) { // always backward plan when there was exception
    _forward = false;
    return resb;
  }

  // try planning forward
  std::vector<ActionFunc> resf;
  for (auto a = actionlist_->begin() + curaction_ + 1; a != actionlist_->end(); ++a) {
    resf.push_back(std::get<2>(*a));
    if (std::get<1>(*a) == requiredHands)
      break;
  }

  // return action list with fewer steps
  if ((_backwardAllowed == 0) ||
      (resf.size() > 0 && resf.size() <= resb.size())) {
    _forward = true;
    return resf;
  } else {
    _forward = false;
    return resb;
  }
}

//////////////////////////////////////////////////
bool NegomoPlanner::prepareForNext
(int _nexttask, int &_usedhands, bool _exception, int _atEnd, int _backwardAllowed)
{
  if (_nexttask == -1) // continue current
    return true;

  int requiredHands = 0;
  if (_usedhands == requiredHands) {
    auto pos = std::find_if(taskq_.begin(), taskq_.end(),
                            [](std::pair<int, int> q){ return (q.first == -1); });
    pos->first = curentid_;
    if (_atEnd == 1 && !_exception) // prepareForNext called at end of action
      pos->second = ++curaction_;
    else // prepareForNext called in middle of action or in exception
      pos->second = curaction_;
    return true; // all state is well
  }

  if (_exception && (_backwardAllowed == 0)) {
    ROS_WARN("[warn] impossible plan! plan failed");
    taskq_.push_back({curentid_, curaction_});
    return false; // go for temp
  }

  // in case of error enter, erase current task from queue
  auto pos = std::find_if(taskq_.begin(), taskq_.end(),
                          [](std::pair<int, int> q){ return (q.first == -1); });
  taskq_.erase(pos);

  if (_backwardAllowed == -1) {
    // re-push current task to queue
    taskq_.push_back({curentid_, curaction_});
    return false; // go for temp task, not handled at this level
  }

  if (_atEnd == 0) { // in middle of action
    ROS_WARN("[warn] expects end of action when hand is in use! plan failed");
    taskq_.push_back({curentid_, curaction_});
    return false; // go for temp (TODO: allow backward from prev action option)
  }

  bool resforward; // check if result is forward
  auto plans = planToNext(_exception, _backwardAllowed, resforward);

  if (resforward)
    vpInterrupt(curaction_, static_cast<int>(plans.size()));
  else
    vpInterrupt(curaction_, -(static_cast<int>(plans.size())-1));
  bool all_is_well = true;
  setMaxPriority(); // do not allow interrupt while getting ready for next task
  for (auto p = plans.begin(); p != plans.end(); ++p) {
    int nexttask = -1;
    vpUpdate(resforward, static_cast<int>(p - plans.begin()));
    _usedhands = (*p)(_usedhands, nexttask);
    if (nexttask == -404) { // action entered exception
      all_is_well = false;
      if (resforward)
        curaction_ += static_cast<int>(p - plans.begin()) + 1;
      else
        curaction_ -= static_cast<int>(p - plans.begin());
      break;
    }
  }

  if (!all_is_well && resforward) {
    all_is_well = true;
    if (_usedhands != requiredHands) {
      auto plans = planToNext(true, _backwardAllowed, resforward);
      vpInterrupt(-1, -static_cast<int>(plans.size())+1);
      for (auto p = plans.begin(); p != plans.end(); ++p) {
        int nexttask = -1;
        _usedhands = (*p)(_usedhands, nexttask);
        if (nexttask == -404) { // action entered exception
          all_is_well = false;
          break;
        }
        vpUpdate(--curaction_);
      }
      // curaction_ -= plans.size();
    }
  } else {
    if (resforward)
      curaction_ += plans.size() + 1;
    else
      curaction_ -= plans.size() - 1;
  }

  if (!all_is_well) {
    ROS_ERROR("[error] plan failed");
    return false;
  }

  // re-push current task to queue and record action num
  taskq_.push_back({curentid_, curaction_});

  return true;
}

//////////////////////////////////////////////////
int NegomoPlanner::getWaitInterpolation()
{
  if (waitinterpolation_call_failed_) {
    ROS_ERROR("failing get as wait interpolation had error!");
    return -1; // continue current
  }

  negomo_enshu::WaitInterpolationRequest srv;
  if (!waitinterpolation_result_client_.call(srv)) {
    ROS_ERROR("failing get as call error to wait interpolation result!");
    return -1; // continue current
  }

  for (auto task = srv.response.queue.begin();
       task != srv.response.queue.end(); ++task) {
    if (*task == -1) {
      taskq_.push_back({-1, -1});
      // do not vpQueue here, as plot must be active while forward/backwarding
    } else {
      taskq_.push_back({*task, 0});
      // queue new task to vp (nodes unknown at moment)
      std_msgs::Int32 vpmsg;
      vpmsg.data = *task;
      vppub_addtoqueue_.publish(vpmsg);
      usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
    }
  }

  return srv.response.next;
}

//////////////////////////////////////////////////
boost::property_tree::ptree& NegomoPlanner::nextEntities()
{
  if (created_tmpentity_ >= 0) // already created
    return entities_[created_tmpentity_];

  if (num_capabilities_ < 0) {
    ROS_ERROR("Illegal nextEntities()! Please call init() at setup!");
    std::exit(0);
  }

  // find empty ID
  int maxval = num_capabilities_ - 1;
  for (auto ent = entities_.begin(); ent != entities_.end(); ++ent)
    if (ent->first > maxval)
      maxval = ent->first;
  created_tmpentity_ = maxval + 1;

  return entities_[created_tmpentity_];
}

//////////////////////////////////////////////////
int NegomoPlanner::createTask(int _taskid)
{
  if (created_tmpentity_ < 0) // no settings yet
    nextEntities(); // create tmpentity with default values

  for (auto e = entities_.begin(); e != entities_.end(); ++e) {
    if (e->first == created_tmpentity_)
      continue;
    if (checkTaskDuplicate(e->first, _taskid))
      return -1;
  }

  nextEntities().put("taskid", _taskid);
  int next = created_tmpentity_;
  created_tmpentity_ = -1; // reset tmpentity for next createTask

  return next;
}

//////////////////////////////////////////////////
bool NegomoPlanner::checkTaskDuplicate(int _refentid, int _taskid)
{
  // TODO: Handle ALL (if ALL in queue but new entity not ALL -> conflict)
  // TODO: Handle EACH (merged entity update -> updateTask)

  int taskid = entities_[_refentid].get<int>("taskid");
  if (taskid != _taskid)
    return false; // _taskid is not a duplicate of _refentid

  for (auto ent = default_entities_.at(_taskid).begin();
       ent != default_entities_.at(_taskid).end(); ++ent) {
    std::string entity = ent->first;
    boost::optional<std::string> enext_opt =
      nextEntities().get_optional<std::string>(entity);
    boost::optional<std::string> eref_opt =
      entities_[_refentid].get_optional<std::string>(entity);
    std::string enext = (enext_opt ? enext_opt.get() : ent->second);
    std::string eref = (eref_opt ? eref_opt.get() : ent->second);
    if (enext != eref)
      return false; // different entities = different task
  }

  // currently pushed entities are duplicates, remove
  auto tmp = entities_.find(created_tmpentity_);
  if (tmp != entities_.end())
    entities_.erase(tmp);
  created_tmpentity_ = -1; // reset tmpentity for next createTask
  return true; // same task already exists
}

//////////////////////////////////////////////////
void NegomoPlanner::init(std::vector<std::vector<std::string> > _entities)
{
  num_capabilities_ = _entities.size();
  defaultq_.clear();
  taskq_.clear();
  tmpq_.clear();
  curentid_ = -1;
  curaction_ = -1;
  exception_escaped_id_ = -1;
  created_tmpentity_ = -1;
  for (auto t = _entities.begin(); t != _entities.end(); ++t) {
    negomo_enshu::NegomoTask msg; // for default interaction
    std::vector<std::pair<std::string, std::string> > entities;
    int i = 0;
    // create task names
    if (t->size() > 0 && t->begin()->find("=", 0) == std::string::npos) {
      capability_names_.push_back(*t->begin());
      msg.task_name = *t->begin(); // for default interaction
      i = 1;
    } else {
      ROS_WARN("task name is recommended for vector element 0 in entities");
      capability_names_.push_back(std::to_string(t - _entities.begin()));
    }
    // create entities
    for (auto ent = t->begin()+i; ent != t->end(); ++ent) {
      auto pos = ent->find("=", 0);
      if (pos == std::string::npos) {
        ROS_WARN("illegal entity with no default value! expects 'key=value'");
        continue;
      }
      entities.push_back({ent->substr(0, pos), ent->substr(pos + 1)});
      msg.entity_names.push_back(ent->substr(0, pos)); // for default
      msg.entity_values.push_back(ent->substr(pos + 1)); // for default
    }
    default_entities_.push_back(entities);
    intsrv_.request.tasks.push_back(msg); // for default
  }
}

//////////////////////////////////////////////////
void NegomoPlanner::setDefaultQueue(std::vector<int> _q)
{
  defaultq_.clear();
  appendDefaultQueue(_q);
}

//////////////////////////////////////////////////
void NegomoPlanner::appendDefaultQueue(std::vector<int> _q)
{
  for (auto t = _q.begin(); t != _q.end(); ++t) {
    auto duplicate = std::find_if(defaultq_.begin(), defaultq_.end(),
        [&](std::pair<int, int> q){ return (q.first == *t); });
    if (duplicate != defaultq_.end()) {
      ROS_WARN("found task with duplicate id! ignoring");
      continue;
    }
    defaultq_.push_back({*t, 0});
  }
}

//////////////////////////////////////////////////
std::string NegomoPlanner::waitUserInput(std::string _type)
{
  std_msgs::String msg;
  msg.data = _type;
  uipub_.publish(msg);
  std::string input;
  while (input == "") {
    input_mutex_.lock();
    input = input_;
    input_mutex_.unlock();
  }
  input_mutex_.lock();
  input_ = "";
  input_mutex_.unlock();
  return input;
}

//////////////////////////////////////////////////
bool NegomoPlanner::shortInteraction
(negomo_enshu::PlannerInteractionCall::Request &_req,
 negomo_enshu::PlannerInteractionCall::Response &_res)
{
  if (capability_names_.size() != num_capabilities_) {
    ROS_ERROR("Illegal shortInteraction()! Please call init() at setup!");
    ROS_ERROR("Make sure all task are named with 0th entity!");
    return false;
  }

  if (!_req.proceed) { // notify robot is currently occupied
    negomo_enshu::PlannerDefaultInteractionCall srv;
    srv.request.warn = true;
    planner_interaction_client_.call(srv);
    _res.nonext = true;
    return true;
  }

  // else, robot may interact

  if (!planner_interaction_client_.call(intsrv_)) {
    ROS_ERROR("Failed call to interaction in default interaction.");
    _res.nonext = true; // return exception
    return true;
  }

  // no task input
  if (intsrv_.response.task.task_name == "") {
    ROS_WARN("No task was pushed.");
    _res.nonext = true; // no task queued
    return true;
  }

  // get task id from returned task name
  // TODO: handling combined tasks A+B+C
  auto pos = std::find(capability_names_.begin(), capability_names_.end(),
                       intsrv_.response.task.task_name);

  if (pos == capability_names_.end() || !intsrv_.response.status
      || intsrv_.response.task.entity_names.size()
      != intsrv_.response.task.entity_values.size()) {
    ROS_ERROR("Failed to get task in default interaction.");
    _res.nonext = true; // return exception
    return true;
  }

  int next = static_cast<int>(pos - capability_names_.begin());
  // any priority 0 task should be declared in entities
  auto ent = std::find_if(default_entities_.at(next).begin(),
                          default_entities_.at(next).end(),
                          [](std::pair<std::string, std::string> _a)
                          {return (_a.first=="priority"); });
  int priority = 2;
  if (ent != default_entities_.at(next).end())
    priority = std::stoi(ent->second);
  for (size_t i = 0; i < intsrv_.response.task.entity_names.size(); ++i) {
    std::string val = intsrv_.response.task.entity_values.at(i);
    if (val == "***") // wild character
      val = newWild();
    nextEntities().put(intsrv_.response.task.entity_names.at(i), val);
  }
  next = createTask(next);

  if (next < 0) { // is duplicate
    _res.nonext = true;
    return true;
  }

  _res.next = next;
  _res.priority = priority;
  _res.nonext = false;

  return true;
}

//////////////////////////////////////////////////
int NegomoPlanner::initTask_(int _a, int &_b)
{
  boost::optional<int> taskid = entities_[curentid_].get<int>("taskid");
  if (!taskid) {
    ROS_WARN("initiation of task %d failed: id unknown", curentid_);
    return _a;
  }
  if (taskid.get() >= num_capabilities_) {
    ROS_WARN("initiation of task %d failed: %d >= %d",
             curentid_, taskid.get(),num_capabilities_);
    return _a;
  }

  for (auto ent = default_entities_.at(taskid.get()).begin();
       ent != default_entities_.at(taskid.get()).end(); ++ent)
    getEntities().put(ent->first,
                      getEntities().get<std::string>(ent->first, ent->second));

  if (timer_ &&
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::high_resolution_clock::now() - timerstart_).count()
      < timertime_)
    return _a; // don't set interaction flag if timered iOn

  timer_ = false;
  // disable interaction mode if false
  getEntities().get<bool>("iOff", false) ? iOff() : iOn();

  return _a;
}

//////////////////////////////////////////////////
int NegomoPlanner::emptyAction_(int _a, int &_b)
{
  return _a;
}

//////////////////////////////////////////////////
int NegomoPlanner::loopStart_(int _a, int &_b)
{
  // enters once and sets loop (ignored second time with ++curaction_)
  getEntities().put("loopStart", curaction_);
  return _a;
}

//////////////////////////////////////////////////
int NegomoPlanner::loopEnd_(int _a, int &_b)
{
  if (getEntities().get<bool>("loopCondition", false))
    curaction_ = getEntities().get<int>("loopStart", 0);

  return _a;
}

//////////////////////////////////////////////////
int NegomoPlanner::finishTask_(int _a, int &_b)
{
  auto entities = entities_.find(curentid_);
  if (entities != entities_.end())
    entities_.erase(entities);

  usleep(100 * 1000); // to show viewer finish state
  return _a;
}

//////////////////////////////////////////////////
int NegomoPlanner::uiExceptionHandle_(int _a, int &_b)
{
  // user should click where to start from (TODO: or abort).
  // user should enter current hand number.
  std::string input = waitUserInput("uiExceptionHandle");
  auto pos = input.find(",", 0);
  if (input.find("tmp", 0) == std::string::npos) // not temp node
    curaction_ = std::stoi(input.substr(0,pos)) - 1;
  return std::stoi(input.substr(pos+1));
}

//////////////////////////////////////////////////
int NegomoPlanner::tOff_(int _a, int &_b)
{
  iStart();
  int sleepmilli = getEntities().get<int>("sleepMilli", 5000);
  usleep(sleepmilli * 1000);
  iJoin(_a, _b);
  return _a;
}

//////////////////////////////////////////////////
int NegomoPlanner::callActionF
(int _a, int &_b, bool _backward, ros::ServiceClient *_client, int *_action,
std::vector<std::pair<std::string, std::vector<std::string>> > *_id2str)
{
  negomo_enshu::PlannerActionCall srv;
  int taskid = getEntities().get<int>("taskid");
  srv.request.taskid = _id2str->at(taskid).first;
  srv.request.actionid = _id2str->at(taskid).second.at(*_action);
  //srv.request.entityid = curentid_;
  srv.request.usedhands = _a;
  srv.request.backward = _backward;

  if (!_client->call(srv)) {
    ROS_ERROR("unexpected action call error!");
    return -1; // continue current
  }

  _b = srv.response.nextid;
  return srv.response.usedhands;
}

//////////////////////////////////////////////////
int NegomoPlanner::callMoveTo_(int _a, int &_b)
{
  negomo_enshu::PlannerActionCall srv;
  int taskid = getEntities().get<int>("taskid");
  srv.request.taskid = id2str_.at(taskid).first;
  srv.request.actionid = "moveToWorkspace";
  //srv.request.entityid = curentid_;
  srv.request.usedhands = _a;
  srv.request.backward = false;

  if (!callaction_client_.call(srv)) {
    ROS_ERROR("unexpected action call error!");
    return -1; // continue current
  }

  _b = srv.response.nextid;
  return srv.response.usedhands;
}

//////////////////////////////////////////////////
int NegomoPlanner::callAction_(int _a, int &_b)
{
  return callActionF(_a, _b, false, &callaction_client_, &curaction_, &id2str_);
}

//////////////////////////////////////////////////
int NegomoPlanner::callException_(int _a, int &_b)
{
  return callActionF(_a, _b, false, &callexception_client_, &eaction_, &eid2str_);
}

//////////////////////////////////////////////////
int NegomoPlanner::callTemp_(int _a, int &_b)
{
  return callActionF(_a, _b, false, &calltemp_client_, &taction_, &tid2str_);
}

//////////////////////////////////////////////////
int NegomoPlanner::callRevAction_(int _a, int &_b)
{
  return callActionF(_a, _b, true, &callaction_client_, &curaction_, &id2str_);
}

//////////////////////////////////////////////////
int NegomoPlanner::callRevException_(int _a, int &_b)
{
  return callActionF(_a, _b, true, &callexception_client_, &eaction_, &eid2str_);
}

//////////////////////////////////////////////////
int NegomoPlanner::callRevTemp_(int _a, int &_b)
{
  return callActionF(_a, _b, true, &calltemp_client_, &taction_, &tid2str_);
}

//////////////////////////////////////////////////
void NegomoPlanner::uiCallback(const std_msgs::String::ConstPtr &_msg)
{
  input_mutex_.lock();
  input_ = _msg->data;
  input_mutex_.unlock();
}
