#include <ros/ros.h>
#include "roboenvcv/interaction.h"
#include <negomo/NegomoSensors.h>
#include <roboenvcv/PersonCoordinate.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <mutex>
#include <std_msgs/Bool.h>

Eigen::Vector3f target_global_;

ros::Publisher face_status_publisher_;
ros::Publisher face_global_pos_publisher_;
ros::Publisher marker_publisher_;

// for streaming individual positions
std::vector<ros::Publisher> face_pos_v_publisher_;

int max_faces_;
bool project_points_; // does not distinguish face looking up or down

bool now_logging_;
ros::Publisher logger_;

struct FaceLog {
  std::string name;
  std::string status;
  Eigen::Vector3f global_pos;
  bool empty;
  std::chrono::high_resolution_clock::time_point last_track_point;
};

std::vector<FaceLog> logs_;
std::mutex logs_mutex_;

unsigned int callback_count_;
std::mutex count_mutex_;

void setMarker(Eigen::Vector3f _pos1, Eigen::Vector3f _pos2, int _id=0) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "markers";
  marker.id = _id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.02;
  geometry_msgs::Point p1;
  p1.x = _pos1.x(); p1.y = _pos1.y(); p1.z = _pos1.z();
  geometry_msgs::Point p2;
  p2.x = _pos2.x(); p2.y = _pos2.y(); p2.z = _pos2.z();
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker_publisher_.publish(marker);
};

std::string getStatus(const roboenvcv::PersonCoordinate::ConstPtr& _msg,
                      int _id, Eigen::Vector3f& _global_pos) {
  roboenvcv::PersonCameraCoords person;

  person.position3d =
    Eigen::Vector3f(_msg->position3d_camera.x,
                    _msg->position3d_camera.y,
                    _msg->position3d_camera.z);
  person.p_base_to_camera =
    Eigen::Vector3f(_msg->map_to_camera.position.x,
                    _msg->map_to_camera.position.y,
                    _msg->map_to_camera.position.z);
  person.mat_base_to_camera =
    Eigen::Quaternionf(_msg->map_to_camera.orientation.x,
                       _msg->map_to_camera.orientation.y,
                       _msg->map_to_camera.orientation.z,
                       _msg->map_to_camera.orientation.w);
  person.pose3d =
    Eigen::Quaternionf(_msg->pose3d_camera.x, _msg->pose3d_camera.y,
                       _msg->pose3d_camera.z, _msg->pose3d_camera.w);

  // return face position in global coordinate
  _global_pos =
    Eigen::Vector3f(_msg->position3d_map.x,
                    _msg->position3d_map.y,
                    _msg->position3d_map.z);

  // exclude target in case found as human face
  Eigen::Vector3f diff = _global_pos - target_global_;
  diff.z() = 0.0; // don't use z value for distance calculation
  if (diff.norm() < 0.2) // set big number for time delays
    return "rejected";

  Eigen::Vector3f
    sight_direction_camera = person.pose3d * Eigen::Vector3f(1.0, 0, 0);
  Eigen::Vector3f
    sight_direction_base = person.mat_base_to_camera * sight_direction_camera;
  Eigen::Vector3f
    sight_base = _global_pos + 10.0 * sight_direction_base;
  setMarker(_global_pos, sight_base, _id);

  bool is_looking =
    roboenvcv::SharedAttention(person, target_global_, 0.78, project_points_);

  if (is_looking)
    return "looktoward";
  else
    return "lookaway";
};

void LogTimer(const ros::TimerEvent& _event) {
  logs_mutex_.lock();
  int timeout_counter = 0;
  int logging_check = 0; // for logging

  negomo::NegomoSensors msg;
  msg.timestamp = ros::Time::now();
  msg.data.resize(max_faces_);
  auto m_i = msg.data.begin();
  geometry_msgs::PoseArray msg_p;
  msg_p.poses.resize(max_faces_);
  auto mp_i = msg_p.poses.begin();
  // check timeouts
  for (auto log = logs_.begin(); log != logs_.end(); ++log) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - log->last_track_point).count()
        > 5000
        && log->status != "noperson") {
      log->empty = true;
      log->status = "noperson";
      log->global_pos = Eigen::Vector3f(0, 0, 0);
      setMarker(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0),
                static_cast<int>(log - logs_.begin()));
      ++timeout_counter;
    }
    if (log->status == "rejected")
      *m_i = "noperson";
    else
      *m_i = log->status;

    // for logging
    if (*m_i == "noperson")
      ++logging_check;

    ++m_i;

    // update positions
    mp_i->position.x = log->global_pos.x();
    mp_i->position.y = log->global_pos.y();
    mp_i->position.z = log->global_pos.z();
    ++mp_i;
  }

  if (timeout_counter > 0) {
    face_status_publisher_.publish(msg);
    face_global_pos_publisher_.publish(msg_p);
  }
  logs_mutex_.unlock();

  // start/end logging trigger (mainly for auto-rosbag)
  if (now_logging_ && (logging_check == max_faces_)) {
    std_msgs::Bool msg;
    msg.data = false;
    logger_.publish(msg);
    now_logging_ = false;
  } else if (!now_logging_ && (logging_check < max_faces_)) {
    std_msgs::Bool msg;
    msg.data = true;
    logger_.publish(msg);
    now_logging_ = true;
  }
};

void PersonCoordinateWithIDCallback
(const roboenvcv::PersonCoordinate::ConstPtr& _msg) {
  // always check that TargetPoseCallback has been called at least once!
  count_mutex_.lock();
  unsigned int count = callback_count_;
  count_mutex_.unlock();
  if (count == 0) {
    ROS_ERROR("pose callback was never called!!!!!! are you sure msgs are correct?");
    return;
  }

  logs_mutex_.lock();

  // ROS_INFO("stat: %s", _msg->id.c_str());

  negomo::NegomoSensors msg;
  msg.timestamp = ros::Time::now();
  msg.data.resize(max_faces_);
  geometry_msgs::PoseArray msg_p;
  msg_p.poses.resize(max_faces_);

  // person all removed signal
  if (_msg->id == "---remove-all") {
    auto m_i = msg.data.begin();
    auto mp_i = msg_p.poses.begin();
    for (auto log = logs_.begin(); log != logs_.end(); ++log) {
      log->empty = true;
      log->status = "noperson";
      log->global_pos = Eigen::Vector3f(0, 0, 0);
      setMarker(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0),
                static_cast<int>(log - logs_.begin()));
      *m_i = log->status;
      ++m_i;

      mp_i->position.x = log->global_pos.x();
      mp_i->position.y = log->global_pos.y();
      mp_i->position.z = log->global_pos.z();
      ++mp_i;
    }
    face_status_publisher_.publish(msg);
    face_global_pos_publisher_.publish(msg_p);
    logs_mutex_.unlock();
    return;
  }

  // find buffer id from name in log
  bool buffer_id_found = false;
  for (auto log = logs_.begin(); log != logs_.end(); ++log)
    if (_msg->id == log->name) {
      log->status =
        getStatus(_msg, static_cast<int>(log - logs_.begin()), log->global_pos);
      log->last_track_point = std::chrono::high_resolution_clock::now();
      buffer_id_found = true;
      break;
    }
  // new name, try setting a buffer id from an empty log
  if (!buffer_id_found)
    for (auto log = logs_.begin(); log != logs_.end(); ++log)
      if (log->empty) {
        log->name = _msg->id;
        log->empty = false;
        log->status =
          getStatus(_msg, static_cast<int>(log - logs_.begin()), log->global_pos);
        log->last_track_point = std::chrono::high_resolution_clock::now();
        break;
      }
      // else, log is occupied, cannot register face information

  auto m_i = msg.data.begin();
  auto mp_i = msg_p.poses.begin();
  auto pub_i = face_pos_v_publisher_.begin();
  for (auto log = logs_.begin(); log != logs_.end(); ++log) {
    if (log->status == "rejected")
      *m_i = "noperson";
    else
      *m_i = log->status;
    mp_i->position.x = log->global_pos.x();
    mp_i->position.y = log->global_pos.y();
    mp_i->position.z = log->global_pos.z();
    ++m_i;
    ++mp_i;
    // for streaming individual positions
    geometry_msgs::Point p;
    p.x = log->global_pos.x();
    p.y = log->global_pos.y();
    p.z = log->global_pos.z();
    pub_i->publish(p);
    ++pub_i;
  }
  face_status_publisher_.publish(msg);
  face_global_pos_publisher_.publish(msg_p);

  logs_mutex_.unlock();
};

void TargetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
  count_mutex_.lock();
  ++callback_count_;
  count_mutex_.unlock();
  target_global_ =
    Eigen::Vector3f(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "negomo_sensor");
  ros::NodeHandle nh;

  callback_count_ = 0;
  max_faces_ = 3;
  nh.getParam("/negomo/max_targets", max_faces_);

  project_points_ = false;
  nh.getParam("/project_points", project_points_);

  logs_.resize(max_faces_);
  for (auto log = logs_.begin(); log != logs_.end(); ++log) {
    log->name = "anonymous";
    log->status = "noperson";
    log->global_pos = Eigen::Vector3f(0, 0, 0);
    log->empty = true;
    log->last_track_point = std::chrono::high_resolution_clock::now();
  }

  marker_publisher_ =
    nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  face_status_publisher_ =
    nh.advertise<negomo::NegomoSensors>("/negomo/sensor/face", 10);
  // when user is lost, values will stream (0, 0, 0) for notification
  face_global_pos_publisher_ =
    nh.advertise<geometry_msgs::PoseArray>("/negomo/sensor/face/position/global", 10);

  // for streaming individual positions
  // note, these positions will not update when user is lost
  for (size_t i = 0; i < max_faces_; ++i) {
    ros::Publisher pub =
      nh.advertise<geometry_msgs::Point>("/look_at/person/target" + std::to_string(i) + "/map/", 10);
    face_pos_v_publisher_.push_back(pub);
  }

  ros::Subscriber person_coordinate_with_id_subscriber =
    nh.subscribe("/roboenvcv/personcoordinate/global/withid", max_faces_,
                 PersonCoordinateWithIDCallback);
  ros::Subscriber target_pose_subscriber =
    nh.subscribe("/tf_msg/robot", 1, TargetPoseCallback);

  ros::Timer timer =
    nh.createTimer(ros::Duration(0.01), LogTimer);

  target_global_ = Eigen::Vector3f(0, 0, 0);

  now_logging_ = false;
  logger_ = nh.advertise<std_msgs::Bool>("/negomo/recordlog", 10);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
