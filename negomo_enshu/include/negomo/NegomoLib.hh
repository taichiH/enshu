#ifndef __NEGOMO_LIB_INCLUDE__
#define __NEGOMO_LIB_INCLUDE__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <negomo_enshu/NegomoStatus.h>
#include <negomo_enshu/NegomoPlot.h>
#include <negomo_enshu/NegomoService.h>
#include <negomo_enshu/RobotAction.h>
#include <negomo_enshu/BridgeRequest.h>
#include <negomo_enshu/PartialResultRequest.h>
#include <negomo_enshu/RegisterActions.h>
#include <negomo_enshu/WaitInterpolationRequest.h>
#include <negomo_enshu/PlannerInteractionCall.h>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <tuple>

namespace negomo_lib
{
  class NegomoBridge
  {
  public: explicit NegomoBridge
  (ros::NodeHandle _nh, std::string _ns, size_t _p, size_t _r, bool _peekauto=false);

  public: ~NegomoBridge();

  // @brief Breakpoint call on whether to proceed or postpone current task.
  // @param[in] _target: Name of target person.
  // @param[in] _method: For now, always "identical".
  // @return See Below
  //    0: Negotiation result was reactive task.
  //    1: Task may be proceeded.
  //   -4: Fatal error.
  //   Call to state-return required if status_change = 1.
  public: negomo_enshu::BridgeRequest::Response breakfrom
  (std::string _target="", std::string _method="identical");

  // @brief Breakpoint call on whether to finish current interaction.
  // @param[in] _action: Name of current action e.g. proactive1.
  // @return See Below
  //    0: Finish interaction.
  //    1: Continue interaction.
  //   -4: Fatal error.
  public: int peek(std::string _action);

  // @brief Same as above peek, used when action type is set on background.
  // @return true if continue interaction.
  public: bool peek();

  // @brief Set during interaction flag.
  // @param[in] _flag: true if peekin.
  private: void PeekInOut(bool _flag);

  // @brief Begin during interaction. Used with peek() mode.
  public: inline void peekin() {PeekInOut(true);};

  // @brief End during interaction. Used with peek() mode. 
  public: inline void peekout() {PeekInOut(false);};

  // @brief Force a during interaction to begin.
  // @return See below
  //    0: Failed force() due to no person.
  //    1: Succeed force().
  //   -1: Failed force() target person was not found.
  //   -4: Fatal error.
  public: int force();

  // @brief Breakpoint call to forcefully finish current interaction.
  // @return See below
  //    0: Failed away() interaction was already finished.
  //    1: Succeed away().
  //   -4: Fatal error.
  public: int away();

  // @brief Start proactive action without occupying task thread.
  //     Used to set action as proactive when e.g. robot is approaching person.
  // @param[in] _target: Name of target person.
  // @param[in] _start_negotiation: Used for speaking while approaching?
  // @param[in] _method: "identical"(no speak) or "purge"(speak).
  // @return Name of target person (if _in was auto, returns id value).
  public: std::string prepare(std::string _target="", bool _start_negotiation=false, std::string _method="identical");

  // @brief Breakpoint call confirming whether a proactive task is possible.
  // @param[in] _target: Name of target person.
  // @param[in] _method: For now, always "purge".
  // @return See below
  //   -2: Proactive task cannot be conducted as target not found.
  //   -1: Proactive task cannot be conducted. (Postpone task)
  //	0: Negotiation result was reactive task.
  //    1: Proceed proactive task.
  //   -4: Fatal error.
  public: int tryto(std::string _target="", std::string _method="purge");

  // @brief Backchannel call on current state of user.
  // @param[in] _threshold: Probability bounds to distinguish interest.
  // @param[in] _target: Number id of target to check.
  // @param[in] _options: 0 to check degree of interest, 1 to check non-interest.
  // @return See below
  //    1: Human interested in robot.
  //    0: Human not interested in robot.
  //   -1: Human reacted to robot.
  public: bool check(float _threshold=0.3, int _target=-1, int _options=0);

  // @brief Backchannel call on current state of user. Used for node mode.
  // @param[in] _res: Variable to save results.
  // @param[in] _threshold: Probability bounds to distinguish interest.
  // @param[in] _target: Number id of target to check.
  // @param[in] _options: 0 to check degree of interest, 1 to check non-interest.
  // @return See below
  //    1: Human interested in robot.
  //    0: Human not interested in robot.
  //   -1: Human reacted to robot.
  public: bool check(negomo_enshu::PartialResultRequest::Response &_res,
                     float _threshold=0.3, int _target=-1, int _options=0);

  // @brief Status subcriber callback.
  private: void StatusCallback(const negomo_enshu::NegomoStatus::ConstPtr status);

  // @brief Partial result subscriber callback.
  private: void PartialResultCallback(const negomo_enshu::NegomoPlot::ConstPtr _msg);

  // @brief Status subcriber callback.
  private: void SetDuringActionCallback(const std_msgs::String::ConstPtr _msg);

  // @brief Pre-interaction action subscriber callback.
  private: bool ActionCallback(negomo_enshu::RobotAction::Request &req,
                               negomo_enshu::RobotAction::Response &res);

  public: ros::ServiceClient do_action_;

  private: ros::ServiceClient negomo_bridge_;

  private: ros::ServiceClient negomo_intermediate_bridge_;

  private: ros::ServiceClient force_client_;

  private: ros::Publisher negomo_action_finished_;

  private: std::mutex status_mutex_;

  private: std::mutex partial_result_mutex_;

  private: std::mutex peekauto_mutex_;

  private: ros::Subscriber negomo_status_;

  private: ros::Subscriber negomo_partial_result_;

  private: ros::Subscriber set_during_action_;

  private: ros::ServiceServer negomo_actions_;

  private: ros::CallbackQueue action_queue_;

  private: ros::AsyncSpinner action_spinner_;

  private: ros::CallbackQueue status_queue_;

  private: ros::AsyncSpinner status_spinner_;

  private: negomo_enshu::NegomoStatus status_;

  private: ros::CallbackQueue partial_result_queue_;

  private: ros::AsyncSpinner partial_result_spinner_;

  private: std::map<int, std::pair<int, float> > partial_result_;

  private: std::map<int, bool> partial_is_target_;

  private: bool peekauto_setup_;

  private: std::chrono::high_resolution_clock::time_point peekauto_timer_;

  private: int peekauto_result_;

  private: ros::CallbackQueue peekauto_queue_;

  private: ros::AsyncSpinner peekauto_spinner_;

  protected: bool peekauto_;

  protected: int preinteractionid_;

  protected: std::mutex preinteractionid_mutex_;

  protected: ros::NodeHandle nh_;

  protected: std::string ns_;

  // optional functions

  // @brief Enable getting target head position from callback.
  public: void startHeadposListener();

  // @brief Get head position of target. Must call startHeadposListener().
  // @param[in] _i: Number id of target.
  // @return Head position x, y, z.
  public: std::tuple<double,double,double> getHeadPos(int _i); // callback safe

  // @brief Head position callback.
  private: void OptionalHeadPositionCallback
  (const geometry_msgs::PoseArray::ConstPtr &_msg);

  private: ros::Subscriber optional_headpos_sub_;

  private: ros::CallbackQueue optional_headpos_queue_;

  private: ros::AsyncSpinner optional_headpos_spinner_;

  private: std::vector<std::tuple<double,double,double> > optional_headpos_;

  private: std::mutex optional_headpos_mutex_;

  // lib functions

  // @brief Add action callback (only used with lib and not node mode).
  // @param[in] _f: Robot pre-interaction action callback function.
  public: void initlib(const boost::function<bool(negomo_enshu::RobotAction::Request &, negomo_enshu::RobotAction::Response &)> & _f);

  // lib optionals

  // @brief Register robot speech listening callback. Used for action level.
  public: template<typename T> void setSpeechListner
  (std::string _s, const boost::function<void(const boost::shared_ptr<T const > &)> & _f) {
    ros::SubscribeOptions ops =
      ros::SubscribeOptions::create<T>(_s, 10, _f, ros::VoidPtr(), &optional_speech_queue_);
    ros::Subscriber sub = nh_.subscribe(ops);
    optional_speech_sub_.push_back(sub);
  };

  // @brief Register robot speech finished callback. Used for action level.
  public: template<typename T> void setSpeechfinishedListner
  (std::string _s, const boost::function<void(const boost::shared_ptr<T const > &)> & _f) {
    ros::SubscribeOptions ops =
      ros::SubscribeOptions::create<T>(_s, 10, _f, ros::VoidPtr(), &optional_speech_queue_);
    ros::Subscriber sub = nh_.subscribe(ops);
    optional_speech_sub_.push_back(sub);
  };

  // @brief Enable robot speech listening.
  public: void startSpeechListener();

  private: ros::ServiceServer lib_action_;

  private: ros::CallbackQueue lib_queue_;

  private: ros::AsyncSpinner lib_spinner_;

  private: std::vector<ros::Subscriber> optional_speech_sub_;

  private: ros::CallbackQueue optional_speech_queue_;

  private: ros::AsyncSpinner optional_speech_spinner_;
  };

  typedef std::shared_ptr<NegomoBridge> NegomoBridgePtr;
}

#endif
