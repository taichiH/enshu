#include "negomo/NegomoLib.hh"

negomo_lib::NegomoBridgePtr engine_;

// ------------ callbacks ------------

bool Break(negomo_enshu::BridgeRequest::Request &_req,
           negomo_enshu::BridgeRequest::Response &_res) {
  if (_req.method == "")
    _res = engine_->breakfrom(_req.target);
  else
    _res = engine_->breakfrom(_req.target, _req.method);
  return true;
};

bool Try(negomo_enshu::BridgeRequest::Request &_req,
         negomo_enshu::BridgeRequest::Response &_res) {
  _res.status_change = 1;
  if (_req.method == "")
    _res.proceed = engine_->tryto(_req.target);
  else
    _res.proceed = engine_->tryto(_req.target, _req.method);
  return true;
};

bool Prepare(negomo_enshu::NegomoService::Request &_req,
             negomo_enshu::NegomoService::Response &_res) {
  bool start_negotiation = (_req.function == "" ? false : true);
  std::string method = (_req.method == "" ? "identical" : _req.method);
  // result is interacting person id
  _res.result = engine_->prepare(_req.target, start_negotiation, method);
  return true;
};

bool Peek(negomo_enshu::BridgeRequest::Request &_req,
          negomo_enshu::BridgeRequest::Response &_res) {
  if (_req.function == "") {
    ROS_ERROR("Failed Peek()! Please set 'proactive' or 'reactive' in function!");
    return true;
  }
  _res.proceed = engine_->peek(_req.function);
  return true;
};

bool PeekAuto(negomo_enshu::BridgeRequest::Request &_req,
              negomo_enshu::BridgeRequest::Response &_res) {
  _res.proceed = static_cast<int>(engine_->peek());
  return true;
};

bool PeekInOut(std_srvs::SetBool::Request &_req,
               std_srvs::SetBool::Response &_res) {
  if (_req.data)
    engine_->peekin();
  else
    engine_->peekout();
  return true;
};

bool Check(negomo_enshu::PartialResultRequest::Request &_req,
           negomo_enshu::PartialResultRequest::Response &_res) {
  engine_->check(_res, _req.threshold, _req.target, _req.options);
  return true;
};

bool Force(negomo_enshu::BridgeRequest::Request &_req,
          negomo_enshu::BridgeRequest::Response &_res) {
  _res.proceed = engine_->force();
  return true;
};

bool Away(negomo_enshu::BridgeRequest::Request &_req,
          negomo_enshu::BridgeRequest::Response &_res) {
  _res.proceed = engine_->away();
  return true;
};

// ------------ the node ------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "negomo_bridge");
  ros::NodeHandle nh("~");

  std::string ns = "/negomo/";
  nh.getParam("namespace", ns);

  engine_.reset(new negomo_lib::NegomoBridge(nh, ns, 2, 1, true));

  ros::ServiceServer negomo_break_request_ =
    nh.advertiseService(ns + "break", &Break);
  ros::ServiceServer negomo_prepare_request_ =
    nh.advertiseService(ns + "prepare", &Try);
  ros::ServiceServer negomo_try_request_ =
    nh.advertiseService(ns + "try", &Try);
  ros::ServiceServer negomo_peek_request_ =
    nh.advertiseService(ns + "peek", &Peek);
  ros::ServiceServer negomo_peekauto_request_ =
    nh.advertiseService(ns + "peekauto", &PeekAuto);
  ros::ServiceServer negomo_peekinout_request_ =
    nh.advertiseService(ns + "peekinout", &PeekInOut);
  ros::ServiceServer negomo_check_request_ =
    nh.advertiseService(ns + "check", &Check);
  ros::ServiceServer negomo_force_request_ =
    nh.advertiseService(ns + "force/during", &Force);
  ros::ServiceServer negomo_away_request_ =
    nh.advertiseService(ns + "away/during", &Away);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
