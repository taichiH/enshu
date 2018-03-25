#include "negomo.hh"

using namespace negomo;

//////////////////////////////////////////////////
Negomo::Negomo(ros::NodeHandle _nh, int _rate, parameters _params)
{
  // init ros node
  this->nh = _nh;
  this->param = _params;
  std::string filename = ros::package::getPath("negomo");
  filename += "/data/";

#if defined(__DEBUG__)
  this->vpvar_publisher =
    nh.advertise<std_msgs::String>("vp/negomo/var", 5);
  this->vpflow_publisher =
    nh.advertise<std_msgs::String>("vp/negomo/flow", 5);
#endif

  // load models
  this->models.resize(model_data::filenames.size());
  for (unsigned int i = 0; i < model_data::filenames.size(); ++i) {
    std::string model_filename = filename + model_data::filenames[i];
    this->models[i].import(model_filename);
  }

  // create and initiate target buffers and thread locks
  this->target_buffers.resize(this->param.max_targets);
  this->thread_lock.resize(this->param.max_targets, false);
  for (auto it = target_buffers.begin();
       it != target_buffers.end(); ++it) {
    it->observations =
      {"", "", it->observations.now(), it->observations.now()};
    it->action = label::cat("nperceptual0");
    it->time_count = 0;
    it->image = "noperson";
    // set debug color
    int color = 31 + (it - target_buffers.begin());
    color = std::min(color, 37);
    it->debug_output_color = "\033[" + std::to_string(color) + "m";
  }

  // init member variables
  this->setNegotiationTarget(this->target_buffers.end(), "constructor");
  this->setNegomoStatus(false, "constructor");
  this->setNegomoState("perceptual", "constructor");
  this->rate = _rate;
  this->level =
    {
      {"nperceptual", label::method.at("identical")},
      {"proactive", label::method.at("identical")},
      {"reactive", label::method.at("identical")}
    };
  this->run_lock = false;
  this->in_preparing = false;

  // init ros related members
  this->state_publisher =
    nh.advertise<negomo_enshu::NegomoStatus>("flag", 10);
  this->state_server =
    nh.advertiseService("call", &Negomo::TaskPlannerBridge, this);
  this->robot_client =
    nh.serviceClient<negomo_enshu::RobotAction>("do_action");
  this->state_intermediate_server =
    nh.advertiseService("intermediate/call",
                        &Negomo::TaskPlannerIntermediateBridge, this);
  this->force_during_server =
    nh.advertiseService("force/during",
                        &Negomo::ForceDuring, this);

  // parameters must be initialzed explicitly with SetParams

  this->image_subscriber =
    nh.subscribe("sensor/face", 1, &Negomo::ImageCallback, this);

  this->done_action_subscriber =
    nh.subscribe("done_action", 10, &Negomo::DoneActionCallback, this);

  this->plot_publisher =
    nh.advertise<negomo_enshu::NegomoPlot>("plot", 100);

  this->print_publisher =
    nh.advertise<std_msgs::String>("observation_sequence", 100);

  // for logs
  this->appstart_time = std::chrono::high_resolution_clock::now();
  auto appstart_time_t =
    std::chrono::system_clock::to_time_t(this->appstart_time);

#if defined(__DEBUG__)
  usleep(1000 * 1000); // wait for publisher to be ready
  vpFlow("reset-all");
#endif
}

//////////////////////////////////////////////////
Negomo::Negomo(ros::NodeHandle _nh, std::string _model_file, parameters _params)
{
  this->nh = _nh;
  this->param = _params;
  std::string filename = ros::package::getPath("negomo");
  filename += "/data/";
  filename += _model_file;
  this->hmm.import(filename);
}

//////////////////////////////////////////////////
Negomo::~Negomo()
{
  this->setNegotiationTarget(this->target_buffers.end(), "destructor");
  this->target_buffers.clear();
  this->models.clear();
  this->thread_lock.clear();
}

//////////////////////////////////////////////////
void Negomo::ImageCallback(const negomo_enshu::NegomoSensors::ConstPtr& _str)
{
  // ImageCallback may be called in Run or PlannerBridge
  this->image_buffer_lock.lock();
  this->image_buffer.data.resize(_str->data.size());

  // check empty buffer while copying msg to image_buffer
  int empty_buffers = 0;
  for (unsigned int i = 0; i < _str->data.size(); ++i) {
    auto it = _str->data.begin() + i;
    if (*it == "noperson") ++empty_buffers;
    this->image_buffer.data[i] = *it;
  }

  this->image_buffer_lock.unlock();

  // if no person in all target observations
  if (empty_buffers == this->param.max_targets) {
    // this->msg is only set by target
    // when no person exists, target should not exist
    // but for thread safety, check target existence just in case
    int target = this->getNegotiationTarget();
    if (!OnAwait(target)) {
      ROS_WARN("Detected empty buffers, cleaning msg!");
      this->setNegomoStatus(false, "ImageCallback");
      this->setNegomoState("perceptual", "ImageCallback");
    }
  }
}

//////////////////////////////////////////////////
void Negomo::DoneActionCallback(const std_msgs::Empty::ConstPtr& _str)
{
  this->action_finished_trigger = true;
}

//////////////////////////////////////////////////
void Negomo::AsyncOnce()
{
  // AsyncOnce called in Run
  // during so, ImageCallback may be called from PlannerBridge
  this->image_buffer_lock.lock();
  for (unsigned int i = 0; i < this->image_buffer.data.size(); ++i)
    if (i < this->param.max_targets && !this->OnAwait(i)) {
      auto it = this->target_buffers.begin() + i;
      it->image = this->image_buffer.data.at(i);
    }
  this->image_buffer_lock.unlock();
}

//////////////////////////////////////////////////
void Negomo::SyncOnce(seqitr _it)
{
  // SyncOnce called in PlannerBridge under Await
  // during so, ImageCallback may be called from Run
  int pos = _it - this->target_buffers.begin();
  this->image_buffer_lock.lock();
  if (pos < this->image_buffer.data.size())
    _it->image = this->image_buffer.data.at(pos);
  this->image_buffer_lock.unlock();
}

//////////////////////////////////////////////////
void Negomo::Run()
{
  vpFlow("run");
  // Run function may be stopped from TaskPlanner Bridge
  bool run_locked;
  this->run_lock_mutex.lock();
  run_locked = this->run_lock;
  this->run_lock_mutex.unlock();
  if (run_locked) {
    vpFlow("run -> abort");
    return;
  }

  vpFlow("run -> threads");
  // don't run new threads if thread is already running
  for (unsigned int i = 0; i < this->param.max_targets; ++i)
    if (!this->OnAwait(i)) {
      this->Await(i, true); // must happen before AsyncOnce
      std::thread(&Negomo::RunThread, this,
	  this->target_buffers.begin() + i, i).detach();
    }

  vpFlow("run -> threads -> update");
  // update observations
  ros::spinOnce();
  this->AsyncOnce();

  // publish current state to task planner
  this->state_publisher.publish(this->msg);
  vpFlow("run -> threads -> update -> finished");
}

//////////////////////////////////////////////////
void Negomo::RunThread(seqitr _it, int _i)
{
  vpFlow(threadStr(_i));
  // get latest observation
  std::string observation = this->GetObservation(_it);

  if (observation == "noperson")
    ROS_ERROR("An unexpected 'noperson' was found in RunThread!!!!!!!!!!!!!");

  // if no person in observation
  if (observation == "") {
    vpFlow(threadStr(_i) + " -> noperson");
    // everything is re-initiated
    Reset(_it);
    this->Await(_i, false);

    // if this is target and state is negotiating, this is an unusual phase
    if (this->msg.negotiating && (_it == this->negotiation_target)) {
      ROS_WARN("detected bad task planner bridge escape");
      this->setNegomoState("perceptual", "RunThread");
      this->UpdateStatSendTaskPlanner(false, label::cat("nperceptual0"));
      this->setNegotiationTarget(this->target_buffers.end(), "RunThread"); // free target
    }
    vpFlow(threadStr(_i) + " -> noperson -> finish");
    return;
  }

  // if something new in observation
  if (observation != _it->observations.last) {
    vpFlow(threadStr(_i) + " -> newobservation");

    // scale observation if needed
    this->ScaleObservation(_it);

    // process from registered action
    this->Process(observation, _it->action, _it);
    _it->time_count = 0;
  }

  // if no new observation but constant time has passed
  if (_it->time_count > this->param.percept_unit_sec * this->rate) {
    vpFlow(threadStr(_i) + " -> timeupdate");
    this->ObserveFrom(_it);
    _it->time_count = 0;
  }

  // update time count
  ++_it->time_count;

  this->Await(_i, false);
  vpFlow(threadStr(_i) + " -> finish");
}

//////////////////////////////////////////////////
int Negomo::Process(
    std::string _observation, labelled_action _action, seqitr _it)
{
  vpFlow(threadStr(getProcess(_it)) + " -> RunThread/ObserveFrom -> process");

  // add new observation to sequence
  if (_it->observations.sequence == "") {
    _it->observations.sequence = _observation + "_" + _action.name;
    _it->observations.start = _it->observations.now();
  } else {
    _it->observations.sequence += "," + _observation + "_" + _action.name;
    _it->observations.latest = _it->observations.now();
  }
  _it->observations.last = _observation;

  this->PrintObservations(_it);

  // get human state and its confidence
  weighted_state state =
    this->GetState(_it->observations.sequence,
                   _it->in_during,
                   _it->debug_output_color);
  this->PlotScores(_it, state);

  // when robot is at task state or this is not target in negotiation
  if (this->msg.negotiating == false || _it != this->negotiation_target) {
    vpFlow(threadStr(getProcess(_it)) + " -> process -> nottarget");
    return _it->action.label;
  }

  // when robot is at negotiating state

  // when conflict is proceed-able
  if (this->in_preparing && state.name == "n") {
    vpFlow(threadStr(getProcess(_it)) + " -> process -> npreparing");
    this->UpdateStatSendTaskPlanner(false, _it->action);
    _it->time_count = 0;
    return _it->action.label;
  }

  // when conflict is temporary resolved
  if (!this->Conflict(_it->action.label, state) ||
      this->Timeout(_it)) {
    vpFlow(threadStr(getProcess(_it)) + " -> process -> conflict");

    // below human wants to interact
    if (state.name != "n") {
      _it->observations.sequence = "";
      _it->observations.last = "";
      _it->in_during = true; // enter during
    }

    // else,
    // when state is perceptual, keep observing if not certain
    // this is to reduce the chance of 'false avoidance'

    // publish state to task planner
    if (state.name == "p") {
      this->setNegomoState("proactive", "Process" + getProcess(_it));
      _it->action = label::cat("reactive0", false); // enter during interaction
    } else if (state.name == "r") {
      this->setNegomoState("reactive", "Process" + getProcess(_it));
      _it->action = label::cat("proactive0", false); // enter during interaction
    } else {
      this->setNegomoState("perceptual", "Process" + getProcess(_it));
      _it->action = label::cat("nperceptual0"); // robot should return to task
    }

    this->UpdateStatSendTaskPlanner(false, _it->action); // keep action for next during
    _it->time_count = 0;
    return _it->action.label;
  }

  // when negotiation is on-going

  // decide from registered(want to do) action, not conducted action
  // the want-to-do action may change according to estimated state
  labelled_action next_action = this->DecideAction(_it->action, state);

  // no extra process when no change in registered(want to do) action
  if (next_action.name == _it->action.name) {
    vpFlow(threadStr(getProcess(_it)) + " -> process -> break");
    return _it->action.label;
  }

  // conduct registered action and update observation
  _it->action = next_action;
  return this->ObserveFrom(_it);
}

//////////////////////////////////////////////////
weighted_state Negomo::GetState(
    std::string _sequence, bool _in_during, std::string _dbg_output_color)
{
  int model_num = _in_during ? 1 : 0;

  double es_score;
  std::vector<std::string> states =
    this->Estimate(this->models[model_num], _sequence, es_score, 1,
                   true, _dbg_output_color);

  double esf_score;
  std::vector<std::string> predicted_states =
    this->EstimateFromFuture(this->models[model_num], _sequence,
                             esf_score, true, _dbg_output_color);

  weighted_state score =
    this->Score(states, predicted_states, es_score, esf_score, 1.0);

  this->PrintScores({score}, _dbg_output_color);

  return score;
}

//////////////////////////////////////////////////
bool Negomo::Conflict(int _action_label, weighted_state &_state)
{
  if (label::eq(_action_label, _state.name) && (_state.confidence >= this->param.confidence_thre)) {
    return false;
  } else {
    // future estimate is used for processing
    _state.name = _state.name_from_future;
    return true;
  }
}

//////////////////////////////////////////////////
std::vector<std::string> Negomo::Estimate(
    StochHMM::model _model, std::string _seq,
    double &_score, int _idx_from_back,
    bool _print, std::string _dbg_output_color)
{
  // setups to use StochHMM
  StochHMM::track* tr = _model.getTrack(0);
  StochHMM::sequence* seq = new StochHMM::sequence(_seq, tr);
  StochHMM::sequences sequences;
  sequences.addSeq(seq, tr);

  StochHMM::trellis trell(&_model, &sequences);
  trell.viterbi();
  StochHMM::traceback_path* path =
    new(std::nothrow) StochHMM::traceback_path(trell.get_model());
  trell.traceback(*path);

  if (_print) {
    std::cout << _dbg_output_color; // set debug color
    path->print_label();
    // std::cout << path->getScore() << std::endl;
  }

  // return label sequence
  std::vector<std::string> guessed_sequence(path->size());
  for (unsigned int i = 0; i < path->size(); ++i)
    guessed_sequence[path->size() - i - 1] =
        std::string(_model.getStateLabel(path->val(i)));

  trell.posterior();
  auto table = trell.getPosteriorTable();
  auto scores = table->end() - _idx_from_back;
  _score = *std::max_element(scores->begin(), scores->end());

  delete(path);

  return guessed_sequence;
}

//////////////////////////////////////////////////
std::vector<std::string> Negomo::EstimateFromFuture(
    StochHMM::model _model, std::string _sequence, double &_score,
    bool _print, std::string _dbg_output_color)
{
  // naive state estimate at t=i is sometimes a bad estimate
  // see if estimate at t=i may change in the future
  std::string future_sequence = _sequence;
  std::string last_observation;
  auto pos = future_sequence.find_last_of(",");
  if (pos != std::string::npos)
    last_observation = future_sequence.substr(pos + 1);
  else
    last_observation = future_sequence;
  for (unsigned int i = 0; i < this->param.future_ref; ++i)
    future_sequence += ("," + last_observation);

  return this->Estimate(_model, future_sequence, _score,
                        this->param.future_ref + 1, _print, _dbg_output_color);
}

//////////////////////////////////////////////////
std::string Negomo::GetObservation(seqitr _it)
{
  if (_it->image == "noperson") return "";
  else return _it->image;
}

//////////////////////////////////////////////////
labelled_action Negomo::DecideAction(
    labelled_action _action, weighted_state _state)
{
  // when robot wants to interact

  if (_action.label == label::proactive) // action quits w/ timeout
    return label::grep(label::proactive, this->level, _action.send_to_robot);

  // else simple action decisions from states

  if (_state.name == "n")
    return label::grep(label::perceptual, this->level);
  else if (_state.name == "p")
    return label::grep(label::reactive, this->level);
  else if (_action.label == label::proactive && _state.name == "r")
    return label::grep(label::proactive, this->level);

  return label::grep(label::perceptual, this->level);
}

//////////////////////////////////////////////////
bool Negomo::TaskPlannerIntermediateBridge(
    negomo_enshu::NegomoService::Request &_req, negomo_enshu::NegomoService::Response &_res)
{
  ROS_WARN("Called TaskPlannerIntermediateBridge");

  if (!this->negotiation_target->in_during) {
    ROS_WARN("Bad intermediate call! Interaction already finished?");
    _res.result = "bad";
    return true;
  }

  // disable asynchronous Run observations (for thread safety)
  this->Synchronous(true);
  this->AwaitAll(); // for thread safety
  // error
  if (this->negotiation_target == this->target_buffers.end()) {
    this->Synchronous(false); // negomo can still continue
    return false;
  }

  // flushes current interaction with target (required when ending during)
  if (_req.flush_interaction) {
    Reset(this->negotiation_target);
    this->setNegotiationTarget(this->target_buffers.end(), "TaskPlannerIntermediateBridge"); // release target
    this->Synchronous(false);
    return true;
  }

  // if for some reason observation sequence is empty (e.g. person was lost)
  if (this->negotiation_target->observations.sequence == "") {
    _res.result = "n:1.0000";
    this->Synchronous(false);
    return true;
  }

  // note, this->msg.negotiating is false, robot action is never perceptual

  // peek current state estimate
  weighted_state state =
    this->GetState(this->negotiation_target->observations.sequence,
                   this->negotiation_target->in_during,
                   this->negotiation_target->debug_output_color);
  this->PlotScores(this->negotiation_target, state);
  _res.result = state.name + ":" + std::to_string(state.confidence);

  // set next robot action
  this->negotiation_target->action = label::cat(_req.function, false);
  this->Synchronous(false); // re-enable Run observations
  return true;
}

//////////////////////////////////////////////////
bool Negomo::TaskPlannerBridge(
    negomo_enshu::NegomoService::Request &_req, negomo_enshu::NegomoService::Response &_res)
{
  vpFlow("call");
  ROS_WARN("Called TaskPlannerBridge %s", _req.context.c_str());

  // disable asynchronous Run observations
  // target_buffers and SearchTarget is thread
  this->Synchronous(true);

  // wait till all running threads has finished
  this->AwaitAll();
  vpFlow("call -> context");

  // set action level if any
  std::istringstream iss_function(_req.function);
  std::vector<std::string> parse_function =
    {std::istream_iterator<std::string>{iss_function},
     std::istream_iterator<std::string>{}};

  std::istringstream iss_method(_req.method);
  std::vector<std::string> parse_method =
    {std::istream_iterator<std::string>{iss_method},
     std::istream_iterator<std::string>{}};

  if (parse_function.size() > 0 &&
      parse_function.size() == parse_method.size())
    for (unsigned int i = 0; i < parse_function.size(); ++i) {
      auto itf = this->level.find(parse_function[i]);
      if (itf != this->level.end()) {
	auto itm = label::method.find(parse_method[i]);
	if (itm != label::method.end())
	  itf->second = itm->second;
      }
    }

  // abort observations during interaction for target
  if (_req.flush_interaction &&
      this->negotiation_target != this->target_buffers.end())
    Reset(this->negotiation_target);
  // make sure engine is in suitable situation
  // note, TaskPlannerBridge should never be called while during
  else if (this->negotiation_target != this->target_buffers.end()
           && this->negotiation_target->in_during)
    Reset(this->negotiation_target);

  if (_req.context == "continue task?") {
    vpFlow("call -> context -> reactive");
    // negomo searches conflict between all targets
    // if targets with conflict are found,
    // sets 1 target according to specified algorithm
    // to limit targets, do so in the sensor layer
    // i.e. don't publish observations for excluded targets
    std::vector<int> candidates;
    std::vector<weighted_state> states;
    weighted_state state_of_target; // used during a continue proactive task

    // search targets with conflict
    for (auto it = this->target_buffers.begin();
         it != this->target_buffers.end(); ++it) {
      // // all actions should be perceptual but just in case
      // it->action = {"nperceptual0", label::perceptual};

      // usually sequences are not empty
      // note: previous negotiation_target should be empty, therefore
      // previous negotiation_target will not be an immediate candidate
      if (it->observations.sequence != "") {
        // check state of current sequence
        weighted_state state =
          this->GetState(it->observations.sequence,
                         it->in_during,
                         it->debug_output_color);
        this->PlotScores(it, state);

        if (it == this->negotiation_target) // target of robot proactive task
          state_of_target = state;

        // when conflict is detected, add as candidate
        // candidates only look for those who are "proactive"
        if (this->Conflict(it->action.label, state) && state.name == "p") {
          // note if uncertain from future guess, is considered Conflict
          states.push_back(state);
          it->slot = states.data() + (states.size() - 1);
          this->setCandidates(candidates, it);
        }
      }
    }

    // set 1 target
    if (candidates.size() > 0) {
      vpFlow("call -> context -> reactive -> personp");
      // robot is trying to check whether to continue proactive task
      // however, human may be trying to interact w/ robot
      if (this->in_preparing
          && (this->negotiation_target != this->target_buffers.end())
          && (std::find(candidates.begin(), candidates.end(),
                        this->getNegotiationTarget())
              != candidates.end())) {
        // in this situation, keep target
        // note, when a different human is trying to interrupt robot,
        // robot will no longer keep target and,
        // later on, robot will have to re-initiate proactive task w/ target
      }

      // when only 1 candidate, that is the target
      // if target is anonymous, get head candidate
      // if target method is not valid, get head candidate
      else if (candidates.size() == 1 || _req.target == "anonymous" ||
               this->SearchTarget(_req.target, candidates) == status::fatal)
        this->setNegotiationTarget(this->target_buffers.begin() + candidates[0],
                                   "TaskPlannerBridgeReactiveP");
      // else, negotiation_target set in SearchTarget

      // conflict was detected or uncertain of human state
      // start negotiating w/ target and postpone task
      // choose action from estimated state and provided method
      // robot actions on target will be overwritten in UpdateStat
      // non-target actions will not be overwritten and will remain perceptual

      int i = this->getNegotiationTarget();
      auto targ = // bad code: this was an unexpected step when the code was written
        std::find(candidates.begin(), candidates.end(), i);
      int targidx = static_cast<int>(targ - candidates.begin());
      // above code should be revised in future and should be more neat

      this->UpdateStatSendTaskPlanner
        (true, this->DecideAction(this->negotiation_target->action, states.at(targidx)));
      _res.result = "postpone";
      // set target to awaitable (lock target thread)
      ROS_INFO("human->robot negotiation target is %d", i);
      this->Await(i, true);
      // enable asynchronous Run observations for non targets
      this->Synchronous(false);
      // scale observations (just in case, usually scaled during process)
      // this->ScaleObservation(this->negotiation_target);
      // reset timer (or else interaction may escape the moment it starts)
      this->negotiation_target->observations.start =
        this->negotiation_target->observations.now();
      // target is awaited, therefore target flow is controlled by this thread
      vpFlow("call -> context -> observe");
      this->ObserveFrom(this->negotiation_target);
      this->negotiation_target->time_count = 0;
      // free target thread
      this->Await(i, false);
      return true;
    }

    else {
      vpFlow("call -> context -> reactive -> personelse");
      if (this->in_preparing) {
        vpFlow("call -> context -> reactive -> personelse -> preparing");
        // no human->robot interaction detected
        // check whether robot->human is valid
        if (state_of_target.name == "r") {
          vpFlow("call -> context -> reactive -> preparing -> personr");
          // this means robot may proceed to proactive action
          _res.result = "push";
          this->negotiation_target->observations.sequence = "";
          this->negotiation_target->observations.last = "";
          this->negotiation_target->in_during = true;
        } else {
          vpFlow("call -> context -> reactive -> personn");
          // this means human is not ready to react to robot or target was not found
          // please handle lost target at planner level
          _res.result = "proceed";
        }
        this->Synchronous(false); // in case asynchronous is still locked
        return true;
      } else {
        vpFlow("call -> context -> reactive -> personn");
        // if no conflicts, release target
        this->setNegotiationTarget(this->target_buffers.end(), "TaskPlannerBridgeReactiveN");
      }
    }

  }


  // start negotiating if robot requires proactive action
  else if (_req.context == "interaction ok?" || _req.context == "prepare interaction?") {
    vpFlow("call -> context -> proactive");
    // search for human, if fail abort
    if (this->SearchTarget(_req.target) == status::fatal) {
      vpFlow("call -> context -> proactive -> abort");
      _res.result = "abort";
      this->Synchronous(false);
      return true;
    }

    // negotiation target set in SearchTarget

    if (_req.context == "interaction ok?") {
      this->UpdateStatSendTaskPlanner(
          true, label::grep(label::proactive, this->level));
      this->in_preparing = false;
    }

    // set target to awaitable (lock target thread)
    int i = this->getNegotiationTarget();
    ROS_INFO("robot->human negotiation target is %d", i);
    this->Await(i, true);
    // enable asynchronous Run observations for non targets
    this->Synchronous(false);

    // for external camera, observations may require clean up
    if (_req.context == "prepare interaction?") {
      vpFlow("call -> context -> proactive -> prepare");
      this->ScaleObservation(this->negotiation_target);
      if (parse_function.size() == 1 && parse_function.at(0) == "proactive") {
        ROS_INFO("preparing proactive interaction!!!!");
        // in some situations, preparation itself may be proactive
        this->negotiation_target->action = label::grep(label::proactive, this->level, false);
        this->negotiation_target->time_count = 0;
        this->in_preparing = true;
      }
      this->Await(i, false);
      _res.result = std::to_string(i);
      return true;
    }

    vpFlow("call -> context -> proactive -> usual");
    // reset timer (or else interaction may escape the moment it starts)
    this->negotiation_target->observations.start =
      this->negotiation_target->observations.now();
    this->negotiation_target->time_count = 0;
    vpFlow("call -> context -> observe");
    this->ObserveFrom(this->negotiation_target);

    // free target thread
    this->Await(i, false);
  }

  // negotiation status okay, task planner may proceed
  this->setNegomoState("perceptual", "TaskPlannerBridge");
  _res.result = "proceed";
  this->Synchronous(false); // in case asynchronous is still locked
  vpFlow("call -> context -> finish");
  return true;
}

//////////////////////////////////////////////////
void Negomo::SpinInObserveFrom(seqitr _it, std::string _action, timed_sequence &_timer)
{
  // update observation
  ros::spinOnce();
  this->SyncOnce(_it);

  std::string observation = this->GetObservation(_it);

  if (observation == "noperson")
    ROS_ERROR("An unexpected 'noperson' was found in ObserveFrom mid!!!!!!!!!!!!!");

  // skip if no change in observation was found
  if (observation == _it->observations.last || observation == "") {
    _timer.latest = _timer.now();
    return;
  }

  // if change in observation, add to sequence
  // the action during transition before the switch is previous action
  if (_it->observations.sequence == "") {
    _it->observations.sequence = observation + "_" + _action;
    _it->observations.start = _it->observations.now();
  } else {
    _it->observations.sequence += "," + observation + "_" + _action;
    _it->observations.latest = _it->observations.now();
  }
  _it->observations.last = observation;

  this->PrintObservations(_it);
}

//////////////////////////////////////////////////
int Negomo::ObserveFrom(seqitr _it)
{
  vpFlow(threadStr(getProcess(_it)) + " -> RunThread/Process/TaskPlannerBridge -> observe");

  labelled_action _action = _it->action; // due to fix from old code

  // send action to robot
  negomo_enshu::RobotAction act;
  act.request.action = _action.name;
  act.request.target_id = this->getNegotiationTarget();

  ROS_INFO("%snegotiating is %d, target is %d, called action %s, send to robot %d",
	   _it->debug_output_color.c_str(),
	   this->msg.negotiating, act.request.target_id, _action.name.c_str(),
           static_cast<int>(_action.send_to_robot));

  std::string prev_act;
  auto const pos = _it->observations.sequence.find_last_of("_");
  if (pos != std::string::npos)
    prev_act = _it->observations.sequence.substr(pos + 1);
  else
    prev_act = "nperceptual0";

  // check whether previous action is steady level action
  // AND next action is equal to previous action
  // (occurs when cyclic update is called in observation)
  // has isdigit check in case action level is larger than 10
  bool is_steady_action =
    _action.name.back() == '0' && _action.label == label::cat(prev_act).label
    && !isdigit(_action.name.at(_action.name.length() - 2));

  timed_sequence timer;
  timer.start = timer.now();
  timer.latest = timer.now();

  this->action_finished_trigger = false;
  if (!is_steady_action && _action.send_to_robot) {
    if (!this->robot_client.call(act)) {
      ROS_WARN("error occured while conducting robot action %s target %d",
               act.request.action.c_str(), act.request.target_id);
      vpFlow(threadStr(getProcess(_it)) + " -> observe -> actionfail");
      return status::fatal;
    }
    // real robot action takes time to complete actions
    // add observations when change in scene is detected

    // liftoff parameter is the switch point of before and after action
    while (timer.ms() < this->param.action_liftoff_ratio) {
      SpinInObserveFrom(_it, prev_act, timer);
      // exit when action is finished
      if (this->action_finished_trigger)
        break;
    }
  }

  // if continuous steady action, continue without sleep

  // update observation
  ros::spinOnce();
  this->SyncOnce(_it);

  // get latest observation result
  std::string observation = this->GetObservation(_it);

  if (observation == "noperson")
    ROS_ERROR("An unexpected 'noperson' was found in ObserveFrom end!!!!!!!!!!!!!");

  if (observation == "") {
    ROS_WARN("error occured while receiving observation");
    vpFlow(threadStr(getProcess(_it)) + " -> observe -> none");
    return status::fatal;
  }

  // wait to finish action if action is longer than liftoff
  if (!is_steady_action && _action.send_to_robot)
    while (!this->action_finished_trigger)
      SpinInObserveFrom(_it, _action.name, timer);

  vpFlow(threadStr(getProcess(_it)) + " -> observe -> finish");
  return this->Process(observation, _action, _it);
}

//////////////////////////////////////////////////
int Negomo::SearchTarget(
    std::string _target, const std::vector<int>& _list)
{
  // get first found existing human
  if (_target == "anonymous") {
    for (auto it = this->target_buffers.begin();
	 it != this->target_buffers.end(); ++it)
      if (this->GetObservation(it) != "") {
        this->setNegotiationTarget(it, "SearchTargetAnonymous");
	return status::success;
      }
  }

  // get anonymous with highest state score
  else if (_target == "anonymous_score") {
    float max_score = -1.0;
    for (unsigned int i = 0; i < _list.size(); ++i) {
      if (_list.at(i) >= this->target_buffers.size()) continue;
      auto it = this->target_buffers.begin() + _list.at(i);
      if (!it->slot) continue;
      weighted_state* state = static_cast<weighted_state*>(it->slot);
      if (state->confidence > max_score) {
        this->setNegotiationTarget(it, "SearchTargetAScore");
	max_score = state->confidence;
      }
      it->slot = nullptr; // free slot
    }

    if (max_score < 0) return status::fatal;
    return status::success;
  }

  // get target from least certain score
  else if (_target == "least_certain") {
    float min_score = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < _list.size(); ++i) {
      if (_list.at(i) >= this->target_buffers.size()) continue;
      auto it = this->target_buffers.begin() + _list.at(i);
      if (!it->slot) continue;
      weighted_state* state = static_cast<weighted_state*>(it->slot);
      if (state->confidence < min_score) {
        this->setNegotiationTarget(it, "SearchTargetLCertain");
	min_score = state->confidence;
      }
      it->slot = nullptr; // free slot
    }

    if (min_score == std::numeric_limits<float>::max()) return status::fatal;
    return status::success;
  }

  // get target from number
  else if (!_target.empty() &&
	   std::find_if(_target.begin(), _target.end(),
			[](char c) { return !std::isdigit(c); })
	   == _target.end() &&
	   std::stoi(_target) < this->target_buffers.size()) {
    auto it = this->target_buffers.begin() + std::stoi(_target);
    if (this->GetObservation(it) == "") // target number is valid but empty
      return status::fatal;
    this->setNegotiationTarget(it, "SearchTargetNumber");
    return status::success;
  }

  return status::fatal;
}

//////////////////////////////////////////////////
void Negomo::UpdateStatSendTaskPlanner(bool _status, labelled_action _action)
{
  this->setNegomoStatus(_status, "UpdateStatSendTaskPlanner");
  this->state_publisher.publish(this->msg);
  this->negotiation_target->action = _action;
}

//////////////////////////////////////////////////
bool Negomo::ForceDuring(
    negomo_enshu::NegomoService::Request &_req, negomo_enshu::NegomoService::Response &_res)
{
  ROS_WARN("Called ForceDuring");

  this->Synchronous(true);
  this->AwaitAll(); // for thread safety

  // find target by number id
  if (_req.target != "")
    if (SearchTarget(_req.target) != status::success) {
      ROS_WARN("Bad target passed to ForceDuring.");
      this->Synchronous(false);
      _res.result = "not found";
      return true;
    }

  // find target from possible candidates
  if (this->negotiation_target == this->target_buffers.end()) {
    std::vector<int> priority_candidates;
    std::vector<int> candidates;
    std::vector<weighted_state> states;

    for (auto it = this->target_buffers.begin();
         it != this->target_buffers.end(); ++it) {
      if (it->observations.sequence == "")
        continue;

      weighted_state state =
        this->GetState(it->observations.sequence,
                       it->in_during,
                       it->debug_output_color);
      this->PlotScores(it, state);

      if (state.name == "p") {
        states.push_back(state);
        it->slot = states.data() + (states.size() - 1);
        priority_candidates.push_back
          (static_cast<int>(it - this->target_buffers.begin()));
      } else if (state.name == "n") {
        states.push_back(state);
        it->slot = states.data() + (states.size() - 1);
        candidates.push_back(static_cast<int>(it - this->target_buffers.begin()));
      }
    }

    if (priority_candidates.size() > 0) {
      if (SearchTarget("anonymous_score", priority_candidates) != status::success)
        ROS_WARN("Unexpected error in priority candidates, ForceDuring.");
    } else {
      if (SearchTarget("least_certain", candidates) != status::success)
        ROS_WARN("Unexpected error in candidates, ForceDuring.");
    }
  }

  // no candidates found, error
  if (this->negotiation_target == this->target_buffers.end()) {
    this->Synchronous(false);
    _res.result = "no target";
    return true;
  }

  this->negotiation_target->observations.sequence = "";
  this->negotiation_target->observations.last = "";
  this->negotiation_target->in_during = true;
  this->negotiation_target->action = label::cat("reactive0", false);
  this->UpdateStatSendTaskPlanner(false, this->negotiation_target->action);
  this->negotiation_target->time_count = 0;

  this->Synchronous(false); // re-enable Run observations
  return true;
}

//////////////////////////////////////////////////
bool Negomo::Timeout(seqitr _it)
{
  ROS_INFO("%stimes passed %f",
	   _it->debug_output_color.c_str(), _it->observations.ms());
  if (_it->observations.ms() > this->param.timeout_ms) return true;
  else return false;
}

//////////////////////////////////////////////////
weighted_state Negomo::Score(
    std::vector<std::string> _seq, std::vector<std::string> _fut,
    double _seq_score, double _fut_score, double _weight)
{
  std::string state_estimate = _seq[_seq.size() - 1];

  int continuous_matches = 0;
  for (auto state = _seq.begin(); state != _seq.end(); ++state) {
    if (*state == state_estimate) {
      ++continuous_matches;
    } else {
      continuous_matches = 0;
    }
  }

  float seq_score = exp(_seq_score);
  float fut_score = exp(_fut_score);

  float confidence = seq_score;
  std::string estimate_from_future = _fut[_seq.size() - 1];
  // if (state_estimate != estimate_from_future) {
  //   float normalized_score = seq_score / (seq_score + fut_score);
  //   float normalized_future_score = fut_score / (seq_score + fut_score);
  //   // confidence is Expectation of current score as 1, and else -1
  //   confidence = normalized_score
  //     - std::pow(_weight, this->param.future_ref) * normalized_future_score;
  //   confidence = std::min(seq_score, confidence);
  // }

  return {state_estimate, confidence, estimate_from_future,
      seq_score, fut_score, continuous_matches};
}

//////////////////////////////////////////////////
void Negomo::ScaleObservation(seqitr _it)
{
  // scale observation (remove long time lookaway_nperceptuals)
  size_t pos = 0;
  size_t last = _it->observations.sequence.length();
  int count = 0;
  std::string obs;
  while ((pos = _it->observations.sequence.find_last_of(",", last))
         && pos != std::string::npos) {
    obs = _it->observations.sequence.substr(pos + 1, last - pos);
    if (obs.find("lookaway_nperceptual") != std::string::npos) // hard coded
      ++count;
    else
      break;
    last = pos - 1;
  }

  // flush observations if necessary
  if (count >= this->param.past_flush) {
    // keep only the last lookaway_nperceptual observation
    last = _it->observations.sequence.length();
    pos = _it->observations.sequence.find_last_of(",", last);
    obs = _it->observations.sequence.substr(pos + 1, last - pos);
    _it->observations.sequence = obs;
  }
}

//////////////////////////////////////////////////
void Negomo::Reset(seqitr _it)
{
  _it->observations.sequence = "";
  _it->observations.last = "";
  _it->action = label::cat("nperceptual0");
  _it->time_count = 0;
  _it->in_during = false;
  if (_it == this->negotiation_target)
    this->in_preparing = false;
}

//////////////////////////////////////////////////
void Negomo::PrintObservations(seqitr _it)
{
// #if defined(__DEBUG__)
//   ROS_INFO("----------");
//   if (_it->observations.sequence == "") {
//     ROS_WARN("no observations");
//     return;
//   }
//   std::cout << _it->debug_output_color
//             << this->LogTime() << ": "
//             << _it->observations.sequence << std::endl;
// #endif
  std_msgs::String msg;
  msg.data = std::to_string(static_cast<int>(_it - this->target_buffers.begin()));
  msg.data += ";" + std::to_string(this->LogTime()) + ":" + _it->observations.sequence;
  this->print_publisher.publish(msg);
}

//////////////////////////////////////////////////
void Negomo::PlotScores(seqitr _it, weighted_state _state)
{
  auto time_now = this->LogTime();

  // remove old logs
  auto log = _it->timeline.begin();
  while ((log != _it->timeline.end())
         && (time_now - log->first > 40.0))
    log = _it->timeline.erase(log);

  // add new log
  _it->timeline.push_back({time_now, _state});

  // create msg
  negomo_enshu::NegomoPlot msg;
  msg.target = static_cast<int>(_it - this->target_buffers.begin());
  for (auto log = _it->timeline.begin(); log != _it->timeline.end(); ++log) {
    if (log->second.name == "p") {
      msg.timelinep.push_back(log->first - time_now);
      msg.intentp.push_back(log->second.confidence);
    } else if (log->second.name == "r") {
      msg.timeliner.push_back(log->first - time_now);
      msg.intentr.push_back(-log->second.confidence);
    } else {
      msg.timelinep.push_back(log->first - time_now);
      msg.intentp.push_back(-log->second.confidence);
      msg.timeliner.push_back(log->first - time_now);
      msg.intentr.push_back(log->second.confidence);
    }
  }
  if (_it == this->negotiation_target)
    msg.color =  (_it->in_during ? "g" : "y");
  else
    msg.color =  "b";

  this->plot_publisher.publish(msg);
}

//////////////////////////////////////////////////
void Negomo::PrintScores(std::vector<weighted_state > _scores, std::string _dbg_output_color)
{
  ROS_INFO("%s%f:", _dbg_output_color.c_str(), this->LogTime()); // log
  for (auto s = _scores.begin(); s != _scores.end(); ++s)
    ROS_INFO("%sstate:%s, confidence:%f",
             _dbg_output_color.c_str(),
             s->name.c_str(), s->confidence);
}

//////////////////////////////////////////////////
float Negomo::LogTime()
{
  // return seconds
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - this->appstart_time)
    .count() * 0.001;
}
