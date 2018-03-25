#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <aero_recognition_msgs/Scored2DBoxArray.h>
#include <geometry_msgs/Point.h>
#include <negomo_enshu/NegomoSensors.h>

int max_hands_;
negomo_enshu::NegomoSensors msg_;
ros::Time last_found_;

void callback(const aero_recognition_msgs::Scored2DBoxArray::ConstPtr _msg) {
  msg_.timestamp = _msg->header.stamp;
  msg_.data.clear();
  // if at least one of the hands is in conflict, return all as conflict
  std::string in_conflict = "lookaway";
  for (auto obj : _msg->boxes) {
    last_found_ = ros::Time::now();
    in_conflict = "looktoward";
    break;
  }
  // if hand is lost for more than 5 seconds, clear
  if ((ros::Time::now() - last_found_).toSec() > 5.0)
    in_conflict = "noperson";

  msg_.data.resize(max_hands_, in_conflict); // size does not have a meaning
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "negomo_sensor_hand");
  ros::NodeHandle nh;

  max_hands_ = 1;
  nh.getParam("/negotiation_model/max_targets", max_hands_);

  ros::Publisher hand_status_publisher =
    nh.advertise<negomo_enshu::NegomoSensors>("/negomo/sensor/face", 10);
  ros::Subscriber fcn_sub =
    nh.subscribe("/hand_detector/boxes", 1, &callback);

  std_srvs::SetBool::Request req;
  req.data = true;
  std_srvs::SetBool::Response res;
  ros::service::call("/hand_detector/set_mode", req, res);
  usleep(1000 * 1000);

  ROS_INFO("detect hand node start!!!!!");

  last_found_ = ros::Time::now();
  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    hand_status_publisher.publish(msg_);
    r.sleep();
  };
};
