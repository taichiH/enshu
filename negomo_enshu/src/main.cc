#include "negomo.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "negomo");
  ros::NodeHandle nh("~");

  bool test;
  nh.param<bool>("test", test, false);

  negomo::parameters param;
  param.confidence_thre = 0.6;
  param.future_ref = 3;
  param.past_flush = 3;
  param.percept_unit_sec = 1.0;
  param.timeout_ms = 2000;
  param.action_liftoff_ratio = 2000;
  param.max_targets = 3;

  nh.getParam("confidence_thre", param.confidence_thre);
  nh.getParam("future_ref", param.future_ref);
  nh.getParam("past_flush", param.past_flush);
  nh.getParam("percept_unit_sec",
              param.percept_unit_sec);
  nh.getParam("timeout_ms", param.timeout_ms);
  nh.getParam("action_liftoff_ratio",
              param.action_liftoff_ratio);
  nh.getParam("max_targets",
              param.max_targets);

  // below not used
  for (int i = 0; i < negomo::model_data::param_name.size(); ++i) {
    param.k_model.push_back(
        1.0 / negomo::model_data::param_name.size());
    nh.getParam(negomo::model_data::param_name[i], param.k_model[i]);
  }

  if (test){
    negomo::Test(nh, param);
  } else {
    int rate = 10;

    negomo::NegomoPtr nm(new negomo::Negomo(nh, rate, param));

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Rate loop(rate);
    while (ros::ok()) {
      nm->Run();
      // ros::spinOnce in Run
      loop.sleep();
    }
  }

  return 0;
}
