// usage: remap /negomo/sensor/face -> /negomo/sensor/face/dummy
//        use with roboenvcv/launch2/person_detection2_onbot.launch
// original source1: roboenvcv/nodes2/openpose_faceinfo_extractor.cc
// original source2: linux_kinect/nodes/extract_rgb_from_depth_filtered.cc

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <negomo/NegomoSensors.h>
#include <roboenvcv/BoolStamped.h>
#include <visualization_msgs/Marker.h>

#include <mutex>
#include <vector>

std::vector<geometry_msgs::PoseStamped> v_depth_;
std::vector<geometry_msgs::PoseArray> v_people_;
std::vector<roboenvcv::BoolStamped> v_sensor_;
ros::Publisher filtered_publisher_;
ros::Publisher marker_publisher_;
std::mutex depth_mutex_;
std::mutex people_mutex_;
std::mutex sensor_mutex_;
int max_faces_;

std::function<bool(float, float)> f_;

float localradius_param_r_;

bool LocalRadius(float _x, float _z) {
  // condition: z < r * cos(theta)
  return (_z < (localradius_param_r_ * _z / sqrt(_x*_x + _z*_z)));
}

void SensorPositionCallback
(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
  depth_mutex_.lock();
  v_depth_.push_back(*_msg);
  depth_mutex_.unlock();
}

void SensorFilterCallback
(const roboenvcv::BoolStamped::ConstPtr &_msg) {
  sensor_mutex_.lock();
  v_sensor_.push_back(*_msg);
  sensor_mutex_.unlock();
}

void PeoplePositionCallback
(const geometry_msgs::PoseArray::ConstPtr &_msg) {
  people_mutex_.lock();
  v_people_.push_back(*_msg);
  people_mutex_.unlock();
}

void PeopleFaceCallback
(const negomo::NegomoSensors::ConstPtr &_msg) {
  // because current NegomoSensors output does not have timestamp
  // use most latest sensor results

  bool use_data;
  sensor_mutex_.lock();
  if (v_sensor_.size() == 0) {
    sensor_mutex_.unlock();
    return;
  }
  v_sensor_.erase(v_sensor_.begin(), v_sensor_.end() - 1);
  use_data = v_sensor_.begin()->data;
  sensor_mutex_.unlock();

  if (!use_data) // sensor data out of region, should be rejected
    return;

  float sensor_x, sensor_y;
  depth_mutex_.lock();
  if (v_depth_.size() == 0) {
    depth_mutex_.unlock();
    return;
  }
  v_depth_.erase(v_depth_.begin(), v_depth_.end() - 1);
  sensor_x = v_depth_.begin()->pose.position.x;
  sensor_y = v_depth_.begin()->pose.position.y;
  depth_mutex_.unlock();

  people_mutex_.lock();
  if (v_people_.size() == 0) {
    people_mutex_.unlock();
    return;
  }
  v_people_.erase(v_people_.begin(), v_people_.end() - 1);
  negomo::NegomoSensors msg;
  msg.timestamp = _msg->timestamp;
  msg.data.resize(max_faces_);
  for (size_t i = 0; i < max_faces_; ++i)
    if (!f_(v_people_.begin()->poses.at(i).position.x - sensor_x,
            fabs(v_people_.begin()->poses.at(i).position.y - sensor_y)))
      msg.data.at(i) = "noperson";
    else
      msg.data.at(i) = _msg->data.at(i);
  people_mutex_.unlock();

  filtered_publisher_.publish(msg);

  // publish range marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "dummy_sensor_filter";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.02;
  geometry_msgs::Point p0;
  p0.x = sensor_x + localradius_param_r_;
  p0.y = sensor_y;
  p0.z = 1.0;
  for (float theta = 0.174; theta < 6.28; theta += 0.174) {
    geometry_msgs::Point p;
    p.x = sensor_x + localradius_param_r_ * cos(theta);
    p.y = sensor_y + localradius_param_r_ * sin(theta);
    p.z = 1.0;
    marker.points.push_back(p0);
    marker.points.push_back(p);
    p0 = p;
  }
  marker_publisher_.publish(marker);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_negomo_sensor_results");
  ros::NodeHandle nh("~");

  max_faces_ = 3;
  nh.getParam("/tracknfaces", max_faces_);

  localradius_param_r_ = 3.0; // ~1.2 personal space ~3.7 social space
  nh.getParam("localradius_r", localradius_param_r_);

  f_ = LocalRadius;

  filtered_publisher_ =
    nh.advertise<negomo::NegomoSensors>("/negomo/sensor/face", 10);
  marker_publisher_ =
    nh.advertise<visualization_msgs::Marker>("sensor_filter", 1);

  ros::CallbackQueue sensorpos_queue;
  ros::SubscribeOptions sensorpos_ops =
    ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
        "/tf_msg/dynamic_sensor",
        1,
        boost::bind(&SensorPositionCallback, _1),
        ros::VoidPtr(),
        &sensorpos_queue);
  ros::Subscriber sensorpos_sub = nh.subscribe(sensorpos_ops);
  ros::AsyncSpinner sensorpos_spinner(1, &sensorpos_queue);
  sensorpos_spinner.start();

  ros::CallbackQueue sensorresult_queue;
  ros::SubscribeOptions sensorresult_ops =
    ros::SubscribeOptions::create<negomo::NegomoSensors>(
        "/negomo/sensor/face/dummy",
        1,
        boost::bind(&PeopleFaceCallback, _1),
        ros::VoidPtr(),
        &sensorresult_queue);
  ros::Subscriber sensorresult_sub = nh.subscribe(sensorresult_ops);
  ros::SubscribeOptions peoplepos_ops =
    ros::SubscribeOptions::create<geometry_msgs::PoseArray>(
        "/negomo/sensor/face/position/global",
        1,
        boost::bind(&PeoplePositionCallback, _1),
        ros::VoidPtr(),
        &sensorresult_queue);
  ros::Subscriber peoplepos_sub = nh.subscribe(peoplepos_ops);
  ros::AsyncSpinner sensorresult_spinner(1, &sensorresult_queue);
  sensorresult_spinner.start();

  ros::CallbackQueue sensorfilter_queue;
  ros::SubscribeOptions sensorfilter_ops =
    ros::SubscribeOptions::create<roboenvcv::BoolStamped>(
        "/kinect/global/sensordirection/filter",
        1,
        boost::bind(&SensorFilterCallback, _1),
        ros::VoidPtr(),
        &sensorfilter_queue);
  ros::Subscriber sensorfilter_queue_sub = nh.subscribe(sensorfilter_ops);
  ros::AsyncSpinner sensorfilter_spinner(1, &sensorfilter_queue);
  sensorfilter_spinner.start();

  ros::spin();
}
