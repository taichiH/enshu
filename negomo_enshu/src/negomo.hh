#ifndef _NEGOTIATION_MODEL_H_
#define _NEGOTIATION_MODEL_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <negomo_enshu/NegomoStatus.h>
#include <negomo_enshu/NegomoService.h>
#include <negomo_enshu/RobotAction.h>
#include <negomo_enshu/NegomoSensors.h>
#include <negomo_enshu/NegomoPlot.h>
#include <chrono>
#include <thread>
#include <mutex>
#include "StochHMMlib.h"

#include "data.hh"

#include <algorithm>
#include <iterator>
#include <map>

#define __DEBUG__

namespace negomo
{
  namespace status
  {
    const static int success = 1;

    const static int warning = -1;

    const static int aborted = -2;

    const static int fatal = -4;
  };


  struct labelled_action
  {
  public: std::string name;

  public: int label;

  public: bool send_to_robot;
  };


  namespace label
  {
    const static int perceptual = 0;

    const static int proactive = 1;

    const static int reactive = 2;

    // @brief Return labelled action given action name.
    inline labelled_action cat(std::string _action, bool _send_to_robot=true) {
      if (_action.find("proa") != std::string::npos)
        return {_action, proactive, _send_to_robot};
      else if (_action.find("reac") != std::string::npos)
        return {_action, reactive, _send_to_robot};
      else return {_action, perceptual, _send_to_robot};
    };

    typedef
    std::map<std::string, std::function<std::pair<int, std::string>()> >
    map;
    
    inline std::pair<int, std::string> identical() {
      return {0, "identical"};
    };

    inline std::pair<int, std::string> random() {
      std::random_device rd;
      std::mt19937 mt(rd());
      std::uniform_int_distribution<int> rand(0, 1);
      return {rand(mt), "random"};
    };

    inline std::pair<int, std::string> purge() {
      return {1, "identical"};
    };

    inline std::pair<int, std::string> demand() {
      return {1, "demand"};
    };

    const static map method = {
      {"identical", [](){ return label::identical(); }},
      {"random", [](){ return label::random(); }},
      {"purge", [](){ return label::purge(); }},
      {"demand", [](){ return label::demand(); }}
    };

    // @brief Randomly return action given action label.
    // send_to_robot parameter is overwritten outside of grep
    inline labelled_action grep(int _label, map& _level, bool _send_to_robot=true) {
      std::string name;
      if (_label == proactive) name = "proactive";
      else if (_label == reactive) name = "reactive";
      else name = "nperceptual";
      auto it = _level.find(name);
      if (it != _level.end()) {
        auto level = it->second();
        it->second = method.at(level.second);
        return {name + std::to_string(level.first), _label, _send_to_robot};
      }
      return {name + std::to_string(0), _label, _send_to_robot};
    };

    // @brief Evaluate conflict of robot action and human state.
    inline bool eq(int _label, std::string _state) {
      if (_state == "p") {
        if (_label == reactive) return true;
        return false;
      } else if (_state == "r") {
        if (_label == proactive) return true;
        return false;
      } else {
        if (_label == perceptual) return true;
        return false;
      }
    };
  };


  struct timed_sequence
  {
  public: std::string sequence;

  public: std::string last;

  public: std::chrono::high_resolution_clock::time_point start;

  public: std::chrono::high_resolution_clock::time_point latest;

  public:
    inline std::chrono::high_resolution_clock::time_point now() {
      return std::chrono::high_resolution_clock::now();
    };

  public:
    inline float ms() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
                 latest - start).count();
    };
  };


  struct weighted_state
  {
  public: std::string name;

    // @brief confidence scores from posterior probability.
  public: float confidence;

    // @brief estimate using future observations.
  public: std::string name_from_future;

    // @brief posterior probability of state given current observations.
  public: float posterior;

    // @brief posterior probability of state given future observations.
  public: float posterior_from_future;

    // @brief number of state matches using current observations.
  public: int matches;
  };


  struct sequence_description
  {
  public: bool in_during;

  public: timed_sequence observations;

  public: labelled_action action;

  public: int time_count;

    // @brief Values: noperson, lookaway, looktoward
  public: std::string image;

  public: void* slot;

  public: std::vector<std::pair<float, weighted_state> > timeline;

  public: std::string debug_output_color;
  };

  typedef std::vector<sequence_description>::iterator seqitr;

  
  struct parameters
  {
  public: float confidence_thre;

  public: int future_ref;

  public: int past_flush;

  // public: double hard_thre;

  // public: double soft_thre;

  public: double percept_unit_sec;

  public: int timeout_ms;

  // public: double continue_task_thre;

  // public: double quit_proactive_thre;

  // public: double quit_reactive_thre;

  public: double action_liftoff_ratio;

  public: std::vector<double> k_model;

  public: int max_targets;
  };


  class Negomo
  {
  public: explicit Negomo(ros::NodeHandle _nh, int _rate, parameters _params);

  public: explicit Negomo(ros::NodeHandle _nh, std::string _model_file, parameters _params);

  public: ~Negomo();

  public: void Run();

    // @brief Asynchronous processing of observation sequence for multi user.
  private: void RunThread(seqitr _it, int _i);

    // @brief Return labelled action given action name.
    // \param[in] _observation: Newest observation.
    // \param[in] _action: Conducted(conducting) action.
    // \param[in] _it: Target sequence(human).
    // \return Reserved(want to conduct) action type.
  private: int Process(
      std::string _observation, labelled_action _action, seqitr _it);

  private: weighted_state GetState(
      std::string _sequence, bool _in_during, std::string _dbg_output_color="\033[39m");

  public: bool Conflict(int _action_label, weighted_state &_state);

  public: std::vector<std::string> Estimate(
      StochHMM::model _model, std::string _seq,
      double &_score, int _idx_from_back=1,
      bool _print=true, std::string _dbg_output_color="\033[39m");

  public: std::vector<std::string> EstimateFromFuture(
      StochHMM::model _model, std::string _seq, double &_score,
      bool _print=true, std::string _dbg_output_color="\033[39m");

  private: std::string GetObservation(seqitr _it);

  public: labelled_action DecideAction(
      labelled_action _action, weighted_state _state);

    // @brief Used to find an end to an interaction.
  private: bool TaskPlannerIntermediateBridge(
      negomo_enshu::NegomoService::Request &_req, negomo_enshu::NegomoService::Response &_res);

  private: bool TaskPlannerBridge(
      negomo_enshu::NegomoService::Request &_req, negomo_enshu::NegomoService::Response &_res);

  private: void SpinInObserveFrom(
      seqitr _it, std::string _action, timed_sequence &_timer);

  private: int ObserveFrom(seqitr _it);

  private: int SearchTarget(
      std::string _target,
      const std::vector<int>& _list = std::vector<int>());

    // @brief Called at end or beginning of negotiation sequence.
  private: void UpdateStatSendTaskPlanner(
      bool _status, labelled_action _action);

    // @brief Forces in_during phase on target. e.g. Voice trigger.
  private: bool ForceDuring(
      negomo_enshu::NegomoService::Request &_req, negomo_enshu::NegomoService::Response &_res);

  private: bool Timeout(seqitr _it);

  public: weighted_state Score(
      std::vector<std::string> _seq, std::vector<std::string> _fut,
      double _seq_score, double _fut_score, double _weight);

  private: void ScaleObservation(seqitr _itr);

  private: void Reset(seqitr _itr);

  private: void PrintObservations(seqitr _it);

  private: void PlotScores(seqitr _it, weighted_state _state);

  private: void PrintScores(
      std::vector<weighted_state> _scores,
      std::string _dbg_output_color);

    // @brief Starts synchronous run in PlannerBridge.
    //   During synchronous run, Run() is stopped.
  private:
    inline void Synchronous(bool _flag) {
      this->run_lock_mutex.lock();
      this->run_lock = _flag;
      this->run_lock_mutex.unlock();
    };

    // @brief Checks whether thread is already running.
  public:
    inline bool OnAwait(int _i) {
      bool is_running;
      this->thread_lock_mutex.lock();
      if (_i < this->thread_lock.size() && _i >=0)
        is_running = this->thread_lock.at(_i);
      else
        is_running = false;
      this->thread_lock_mutex.unlock();
      return is_running;
    };

    // @brief Blocks additional thread runs.
  public:
    inline void Await(int _i, bool _flag) {
      this->thread_lock_mutex.lock();
      if (_i < this->thread_lock.size() && _i >= 0)
        this->thread_lock.at(_i) = _flag;
      this->thread_lock_mutex.unlock();
    };

    // @brief Wait till all threads have finished.
  public:
    inline void AwaitAll() {
      while (1) {
        int count = 0;
        this->thread_lock_mutex.lock();
        for (auto it = this->thread_lock.begin();
             it != this->thread_lock.end(); ++it)
          if (*it == false) ++count;
        this->thread_lock_mutex.unlock();
        if (count == this->thread_lock.size())
          break;
        usleep(1000);
      }
    };

  private: ros::NodeHandle nh;

  private: std::vector<StochHMM::model> models;

  private: std::vector<sequence_description> target_buffers;

  private: seqitr negotiation_target;

  private: negomo_enshu::NegomoSensors image_buffer;
    
  private: bool run_lock;

  private: std::mutex run_lock_mutex;

  private: std::vector<bool> thread_lock;

  private: std::mutex thread_lock_mutex;

  private: std::mutex image_buffer_lock;

  private: label::map level;
    
  private: negomo_enshu::NegomoStatus msg;

  private: bool in_preparing;

  private: int rate;

  private: ros::Publisher state_publisher;

  private: ros::ServiceServer state_server;

  private: ros::ServiceServer state_intermediate_server;

  private: ros::ServiceServer force_during_server;

  private: ros::ServiceClient robot_client;

  private: parameters param;

    // sensors >

  private: void ImageCallback(
      const negomo_enshu::NegomoSensors::ConstPtr& _str);

    // @brief Syncs callback data in asynchronous runs.
  private: void AsyncOnce();

    // @brief Syncs callback data in synchronous runs.
  private: void SyncOnce(seqitr _it);

    // @brief Subscribes detection results.
  private: ros::Subscriber image_subscriber;

    // < sensors

  private: void DoneActionCallback(const std_msgs::Empty::ConstPtr& _msg);

  private: ros::Subscriber done_action_subscriber;

  private: bool action_finished_trigger;
    
  // for test

  public: StochHMM::model hmm;

  // for log

  private: float LogTime();

  private: std::chrono::high_resolution_clock::time_point appstart_time;

  private: ros::Publisher plot_publisher;

  private: ros::Publisher print_publisher;

    // variable tracking

  private:
    inline void vpVar(std::string _str) {
      std_msgs::String msg;
      msg.data = _str;
      this->vpvar_publisher.publish(msg);
    };

    inline void vpFlow(std::string _str) {
#if defined(__DEBUG__)
      std_msgs::String msg;
      msg.data = _str;
      this->vpflow_publisher.publish(msg);
#endif
    };

  private: ros::Publisher vpvar_publisher;

  private: ros::Publisher vpflow_publisher;

  private:
    inline int getNegotiationTarget() {
      return static_cast<int>
        (this->negotiation_target - this->target_buffers.begin());
    };

    inline void setNegotiationTarget(seqitr _negotiation_target,
                                     std::string _where) {
      this->negotiation_target = _negotiation_target;
      // target information is sometimes used other than debug
      vpVar("negotiation_target:" + std::to_string(getNegotiationTarget())
            + " in " + _where);
    };

    inline void setNegomoStatus(bool _negotiating, std::string _where) {
      this->msg.negotiating =_negotiating;
#if defined(__DEBUG__)
      vpVar("status.negotiating:"
            + std::to_string(static_cast<int>(_negotiating)) + " in " + _where);
#endif
    };
      inline void setNegomoState(std::string _state, std::string _where) {
      this->msg.state = _state;
#if defined(__DEBUG__)
      vpVar("status.state:" + _state + " in " + _where);
#endif
    };

    inline void setCandidates(std::vector<int>& _v, seqitr _it) {
      _v.push_back(static_cast<int>(_it - this->target_buffers.begin()));
#if defined(__DEBUG__)
      if (_v.size() == 1)
        vpVar("candidates:" + std::to_string(_v.back()));
      else
        vpVar("candidates+:" + std::to_string(_v.back()));
#endif
    };

    inline int returnSearchTarget(std::string _target, int _res) {
#if defined(__DEBUG__)
      vpVar("search:" + _target + " -> " + std::to_string(_res));
#endif
      return _res;
    };

    inline std::string getProcess(seqitr _it) {
      return std::to_string(static_cast<int>(_it - this->target_buffers.begin()));
    };

    inline std::string threadStr(int _i) {
      return "thread" + std::to_string(_i);
    };
    inline std::string threadStr(std::string _s) {
      return "thread" + _s;
    };
  };

  typedef std::shared_ptr<Negomo> NegomoPtr;

  void Test(ros::NodeHandle _nh, negomo::parameters _param);
}

#endif
