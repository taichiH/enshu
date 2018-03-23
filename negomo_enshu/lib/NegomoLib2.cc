#include "negomo/NegomoLib2.hh"

using namespace negomo_lib;

//////////////////////////////////////////////////
NegomoBridge2::NegomoBridge2
(ros::NodeHandle _nh, std::string _ns, ActionFunc _wsf, size_t _p, size_t _r, bool _peekauto, bool _use_base)
  : NegomoBridge(_nh, _ns, _p, _r, _peekauto),
    NegomoPlanner(_nh, _ns, _wsf, false, _use_base)
{
  taskinaction_ = false;

  vppub_wi_ = _nh.advertise<std_msgs::String>(ns_ + "vp/interaction", 5);
}

//////////////////////////////////////////////////
NegomoBridge2::~NegomoBridge2()
{
}

//////////////////////////////////////////////////
void NegomoBridge2::iStart(jumpSettings _js, waitSettings _ws)
{
  if (base_) { // use base method
    NegomoPlanner::iStart(_js, _ws);
    return;
  }

  negomo::WaitInterpolationRequest srv;
  if (!iStartSetup(_js, _ws, srv))
    return;

  // initiate interaction
  taskinaction_mutex_.lock();
  taskinaction_ = true;
  taskinaction_mutex_.unlock();
  results_mutex_.lock();
  wi_finished_ = false;
  results_.next = -1;
  results_.queue.clear();
  results_mutex_.unlock();
  // start interaction in background
  std::thread th = std::thread([&](negomo::WaitInterpolationRequest::Request _req)
                               {InteractiveWaitInterpolation(_req);},
                               srv.request);
  th.detach();
}

//////////////////////////////////////////////////
int NegomoBridge2::getWaitInterpolation()
{
  taskinaction_mutex_.lock();
  bool taskinaction = taskinaction_;
  taskinaction_mutex_.unlock();

  // although unexpected, make sure for safety
  if (!taskinaction) {
    ROS_WARN("getWaitInterpolation() called with no start!");
    return -1;
  }

  negomo::WaitInterpolationRequest srv;

  // end actions
  taskinaction_mutex_.lock();
  taskinaction_ = false;
  taskinaction_mutex_.unlock();
  // wait for interaction to finish
  bool finished = false;
  while (!finished) {
    results_mutex_.lock();
    finished = wi_finished_;
    results_mutex_.unlock();
  }
  results_mutex_.lock();
  // copy results
  srv.response.next = results_.next;
  srv.response.queue.assign(results_.queue.begin(), results_.queue.end());
  // initiate results
  wi_finished_ = false;
  results_.next = -1;
  results_.queue.clear();
  results_mutex_.unlock();

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
void NegomoBridge2::InteractiveWaitInterpolation
(negomo::WaitInterpolationRequest::Request _req)
{
  // assumes taskinaction_ flag false when all actions are done
  // all actions include additional actions within InteractiveWI

  std::string target;
  if (_req.target == -1)
    target = "";
  else
    target = std::to_string(_req.target);

  // re-direct action callback
  preinteractionid_mutex_.lock();
  preinteractionid_ = _req.preinteractionid;
  preinteractionid_mutex_.unlock();

  // if exec_time_ms flag is positive,
  // make sure task action finishes before interaction
  bool postpone_interaction = false;
  int postpone_time_ms = 0;
  if (_req.exec_time_ms >= 5000) {
    postpone_interaction = true;
    postpone_time_ms = _req.exec_time_ms - 5000; // start interaction 5 sec before end
  }

  usleep(100 * 1000);

  results_mutex_.lock();
  results_.next = -1;
  results_mutex_.unlock();

  auto time_start = std::chrono::high_resolution_clock::now() - std::chrono::seconds(5);

  bool check_result = false;
  bool already_interacted = false;
  while (true) {
    usleep(100 * 1000); // high hz will crush negomo viewer

    taskinaction_mutex_.lock();
    bool taskinaction = taskinaction_;
    taskinaction_mutex_.unlock();

    if (!taskinaction)
      break;

    if (_req.interact_only_once && already_interacted) {
      vpWI("-> done");
      continue;
    }

    // interaction is sometimes postponed when task action is long
    if (postpone_interaction &&
        std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - time_start).count()
        > postpone_time_ms) // time elapsed
      postpone_interaction = false;

    if (_req.enable_preinteraction && // robot action partially free
        !postpone_interaction) { // task action is not too long
      vpWI("start:free");
      vpWI("break");
      auto break_result = breakfrom(target);
      if (break_result.proceed && break_result.status_change) {
        vpWI("break -> reset");
        ResetInInteractiveWI(_req.postfalseinteractionid);
      } else if (break_result.proceed == 0) { // begin interaction
        vpWI("break -> yes");
        bool newtask = InteractInInteractiveWI(_req);
        already_interacted = true;
        vpWI("break -> yes -> reset");
        if (newtask)
          ResetInInteractiveWI(_req.postinteractionid);
        else
          ResetInInteractiveWI(_req.postfalseinteractionid);
      }
      // else, no one wants to interact
      vpWI("break -> no");
    } else { // robot action occupied with task
      vpWI("start:busy");
      vpWI("check");
      check_result = check(_req.target, _req.check_threshold, _req.check_options);
      if (_req.warn_avoid // task queue filled, current task high priority
          || postpone_interaction) // warn when in postpone interaction
        if (check_result &&
            std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::high_resolution_clock::now() - time_start).count()
            > 5000) { // time elapsed
          vpWI("check -> warn");
          // trigger occupied action
          negomo::PlannerInteractionCall intsrv;
          intsrv.request.proceed = false;
          if (!shortInteraction(intsrv.request, intsrv.response)) {
            ROS_ERROR("[InteractiveWI] Failed to call interaction callback!");
          } else {
            vpWI("check -> warn -> finish");
            break; // finish waiting for response
          }
          // time_start = std::chrono::high_resolution_clock::now(); //deprecated
        }
      if (!check_result)
        vpWI("check -> no");
    }
  }

  // if check was true
  if (check_result && !_req.warn_avoid) { // begin interaction
    vpWI("check -> yes");
    bool newtask = InteractInInteractiveWI(_req);
    vpWI("check -> yes -> reset");
    // TODO: keep track of global newtask and take AND
    if (newtask)
      ResetInInteractiveWI(_req.postinteractionid);
    else
      ResetInInteractiveWI(_req.postfalseinteractionid);
    // InteractiveWaitInterpolation(_req); // recursive for during return
  }

  vpWI("finish");

  // reset action callback
  preinteractionid_mutex_.lock();
  preinteractionid_ = 0;
  preinteractionid_mutex_.unlock();

  results_mutex_.lock();
  wi_finished_ = true;
  results_mutex_.unlock();
}

//////////////////////////////////////////////////
void NegomoBridge2::ResetInInteractiveWI(int _resetid)
{
  // reset action
  negomo::RobotAction srv;
  if (_resetid == 0)
    srv.request.action = "/negomo/reset";
  else
    srv.request.action = "/negomo/reset/task" + _resetid;
  if (!do_action_.call(srv))
    ROS_ERROR("[ResetInInteractiveWI] Failed to call pre-interaction action.");
}

//////////////////////////////////////////////////
bool NegomoBridge2::InteractInInteractiveWI
(negomo::WaitInterpolationRequest::Request &_req)
{
  vpWI("during");

  // set peek flag
  peekin();

  // the interaction
  negomo::PlannerInteractionCall intsrv;
  intsrv.request.proceed = true;
  if (!shortInteraction(intsrv.request, intsrv.response)) {
    ROS_WARN("[InteractiveWI] Failed to call interaction callback!");
    intsrv.response.nonext = true; // no results
  }

  away();
  peekout();
  vpWI("during -> finish");

  if (intsrv.response.nonext) // task was duplicate or no task from interaction
    return false;

  // get task
  if (intsrv.response.next >= 0) { // -1 = don't change from current task
    if (intsrv.response.priority == 0) // if priority 0, do not allow additional commands
      _req.warn_avoid = true;
    if (_req.task_priority <= intsrv.response.priority) { // current task higher priority
      results_mutex_.lock();
      results_.next = -1;
      if (std::find(_req.queue.begin(), _req.queue.end(), intsrv.response.next)
          == _req.queue.end() &&
          std::find(results_.queue.begin(), results_.queue.end(),
                    intsrv.response.next)
          == results_.queue.end()) {
        results_.queue.push_back(intsrv.response.next);
        --_req.remaining_queue;
      }
      results_mutex_.unlock();
    } else { // current task lower priority
      results_mutex_.lock();
      results_.next = intsrv.response.next;
      if (std::find(_req.queue.begin(), _req.queue.end(), -1)
          == _req.queue.end() &&
          std::find(results_.queue.begin(), results_.queue.end(), -1)
          == results_.queue.end()) {
        results_.queue.push_back(-1);
        --_req.remaining_queue;
      }
      results_mutex_.unlock();
    }
    if (_req.remaining_queue == 0)
      // if queue is now full, do not allow additional commands from next on
      _req.warn_avoid = true;
  }

  return true;
}
