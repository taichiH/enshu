#ifndef __NEGOMO_LIB2_INCLUDE__
#define __NEGOMO_LIB2_INCLUDE__

#include "negomo/NegomoLib.hh"
#include "negomo/NegomoPlannerLib.hh"

namespace negomo_lib
{
  // NegomoBridge2 is an extended NegomoBridge for handling interactivewi
  // NegomoBridge2 is an extended NegomoPlanner using negomo for interactions.
  class NegomoBridge2 : public NegomoBridge, public NegomoPlanner
  {
  public: explicit NegomoBridge2
  (ros::NodeHandle _nh, std::string _ns, ActionFunc _wsf, size_t _p=2, size_t _r=1, bool _peekauto=true, bool _use_base=false);

  public: ~NegomoBridge2();

  // @brief Starts interaction.
  public: virtual void iStart
  (jumpSettings _js=jumpSettings(), waitSettings _ws=waitSettings());

  // @brief Get results from interaction.
  public: virtual int getWaitInterpolation();

  // @brief The threaded interaction.
  private: void InteractiveWaitInterpolation
  (negomo::WaitInterpolationRequest::Request _req);

  // @brief Reset interaction actions in interactiveWaitInterpolation.
  // @param[in] _resetid: ID of reset funtion, calls "reset/task{_resetid}".
  private: void ResetInInteractiveWI(int _resetid);

  // @brief During interaction in interactiveWaitInterpolation.
  private: bool InteractInInteractiveWI
  (negomo::WaitInterpolationRequest::Request &_req);

  private: bool taskinaction_;

  private: std::mutex taskinaction_mutex_;

  private: std::mutex results_mutex_;

  private: bool wi_finished_;

  private: negomo::WaitInterpolationRequest::Response results_;

  // private: ros::ServiceClient wi_interaction_client_;

  // viewer variables

  // @brief Publishes current planner state during an interaction.
  private: inline void vpWI(std::string _str) {
    std_msgs::String msg;
    msg.data = _str;
    vppub_wi_.publish(msg);
    usleep(10 * 1000); // ROS msg may enter in wrong order without sleep
  }

  private: ros::Publisher vppub_wi_;
  };

  typedef std::shared_ptr<NegomoBridge2> NegomoBridge2Ptr;
}

#endif
