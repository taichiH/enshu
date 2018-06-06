#ifndef AERO_DEVEL_LIB_HH_
#define AERO_DEVEL_LIB_HH_

#include <aero_std/AeroMoveitInterface.hh>
#include <aero_std/ObjectFeatures.hh>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Empty.h>
#include <aero_std/TopGrasp-inl.hh>
#include <aero_recognition_msgs/LabeledPoseArray.h>
#include <aero_recognition_msgs/Scored2DBoxArray.h>
#include <aero_recognition_msgs/Scored2DBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace aero {

  struct item {
    std::string label;
    Eigen::Vector3d position;
  };

  struct box {
    aero::Vector3 pose;
    aero::Quaternion qua;
    aero::Vector3 dimension;
    std::string label;
    double value;
  };

  class DevelLib {
  public: explicit DevelLib(ros::NodeHandle _nh, aero::interface::AeroMoveitInterface::Ptr _controller, Eigen::Vector3d _cam_pos=Eigen::Vector3d(0.06, 0.04, 0.08), Eigen::Quaterniond _cam_qua=Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5));

  public: ~DevelLib();

  public: void speak(const std::string &_speech, const float &_wait_sec);

  public: void speakAsync(const std::string &_speech);

  public: bool makeTopGrasp(const aero::arm _arm, const Eigen::Vector3d _pos, aero::trajectory& _tra);

    // grasp coffee

    //with offset
  // public: bool pickCoffeeFront(Eigen::Vector3d _pos, float _container_height=0.80, aero::arm _arm=aero::arm::rarm, Eigen::Vector3d _offset=Eigen::Vector3d(0.0, -0.015, 0.0));
  public: bool pickCoffeeFront(Eigen::Vector3d _pos, float _container_height=0.80, aero::arm _arm=aero::arm::rarm, Eigen::Vector3d _offset=Eigen::Vector3d(0.0, -0.0, 0.0));

  public: bool graspCoffee(aero::arm _arm=aero::arm::rarm);

  public: bool rpyToQuaternion(const aero::Vector3 &_rpy, aero::Quaternion &_rot);

  public: bool setBothArm(aero::Transform &_r_arm, aero::Transform &_l_arm);

  public: bool visualServo(aero::Vector3 &_pos, double &_offset, aero::arm _arm=aero::arm::both_arms);

  public: bool createResultsBuf(const Eigen::Vector3d &_result, std::vector<Eigen::Vector3d> &_results_buf, Eigen::Vector3d &_diff, const int &_max);

  public: bool createResultsBuf(std::vector<Eigen::Vector3d> &_results, std::vector<Eigen::Vector3d> &_results_buf, const int &_max);

  public: bool interactionResultsBuf(const Eigen::Vector3d &_result, std::vector<Eigen::Vector3d> &_results_buf, std::vector<Eigen::Vector3d> &_last_buf);

  public: bool visualizeMarker(const std::vector<Eigen::Vector3d> &_results_buf);

  public: bool getNewPutPos(std::vector<Eigen::Vector3d> &_results, std::vector<Eigen::Vector3d> &_pre_results);

  public: bool watchFlag(const double &_norm, const double &_min, double _max, const int &_index);

  public: bool watchFlag(const double &_norm, const double &_min, const int &_index);

  public: bool placeCoffee(Eigen::Vector3d _pos=Eigen::Vector3d(0.75, -0.25, 1.05), double _offset_y=0.0, aero::arm _arm=aero::arm::rarm);

  public: bool holdSupportItem(Eigen::Vector3d _pos=Eigen::Vector3d(0.75, -0.25, 1.05),
                               Eigen::Vector3d _offset=Eigen::Vector3d(0.0, 0.03, 0.03),
                               aero::arm _arm=aero::arm::rarm);

  public: bool placeCoffeeReach(Eigen::Vector3d _pos=Eigen::Vector3d(0.75, -0.25, 1.05), double _offset_y=0.0, aero::arm _arm=aero::arm::rarm);
  public: void sendResetPose();

  public: bool placeCoffeeReturn();

    // look motions

  public: bool poseAndRecognize(const std::string _location, const std::string _item, Eigen::Vector3d& _pos, float _lifter_z=-0.25);

  public: void lookContainerFront(float _lifter_z=-0.25);

  public: void lookShelfFront(float _lifter_z=0.0);

  public: void lookShelf();

    // recognition

  public: void startFCN();

  public: void startHandDetection();

  public: void endHandDetection();

  public: void setFCNModel(std::string _name, int _projector_id=0);

  public: void setObject3DProjectorMode(int _id);

  public: std::vector<aero::item> recognizeItems();

  public: std::vector<aero_recognition_msgs::Scored2DBox> recognizeHand();

  public: std::vector<aero::box> recognizeBoxes();

  public: bool findItem(std::string _label, std::vector<Eigen::Vector3d> &_positions);

  public: bool findNearestItem(std::string _label,Eigen::Vector3d &_position);

  public: bool findBoxes(std::vector<aero::Vector3> &_positions);

  public: void stopFCN();

  private: void fcnCallback_(const aero_recognition_msgs::LabeledPoseArray::ConstPtr _msg);

  private: void boxCallback_(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr _clustered_boxes);

  private: void handCallback_(const aero_recognition_msgs::Scored2DBoxArray::ConstPtr _hand_boxes);


    // moving

  public: bool goTo(std::string _location);

  private: void triggerArMarker(bool _trigger);

  public: bool itemShelfTf(std::vector<Eigen::Vector3d> _results);

  public: bool adjustShelfArMarker(std::vector<std::string> _markernames={"/shelf_1_marker_link_3", "/shelf_1_marker_link_4", "/shelf_1_marker_link_5"});

  private: void arMarkerCallback_(const ar_track_alvar_msgs::AlvarMarkersPtr _msg);

    // calculation utils

  public: Eigen::Quaterniond getRotationQuaternion(std::string _axis, double _radian);

  public: bool fastestTrajectory3(const aero::arm _arm, const std::vector<aero::Transform> _poses, const aero::eef _eef, aero::trajectory& _tra, bool _lock_lifter=true);

  public: int calcPathTime(std::map<aero::joint, double> _av_from, std::map<aero::joint, double> _av_to, double _factor=0.5);

  public: int calcPathTime(std::map<aero::joint, double> _av, double _factor=0.5);

  public: std::vector<int> calcTrajectoryTimes(aero::trajectory _trajectory, double _factor=0.5);

  public: std::vector<int> calcTrajectoryTimes(aero::trajectory _trajectory, std::vector<double> _factors);

    // adjust utils

  public: bool distinctTwoBox(std::vector<aero::box> &boxes);

  public: bool getContactPoints(const std::vector<aero::box> &boxes,
                                std::vector<aero::Vector3> &_r_contact_point,
                                std::vector<aero::Vector3> &_l_contact_point);

  public: bool calcAdjustmentError(std::vector<aero::Vector3> &_r_contact_point, std::vector<aero::Vector3> &_l_contact_point);

  public: bool makeAdjustableTrajectory(std::vector<aero::trajectory> &_adjust_tra, const std::vector<aero::Vector3> &_r_contact_point,const std::vector<aero::Vector3> &_l_contact_point, aero::arm _arm, aero::trajectory &_tra);

  public: bool findLabeledBox(std::string _label, std::vector<aero::Vector3> _pos);

  public: bool handEyeManipulation(std::vector<aero::Vector3> &_pos);

  public: bool checkDisplayState();

  public: bool adjust(std::vector<aero::trajectory> _tra);

    // hand usage

  public: int getUsingHandsNum();

  public: bool isUsingArm(aero::arm _arm);

  public: void setUsingArm(aero::arm _arm);

  public: void unsetUsingArm(aero::arm _arm);

  public: void openHand(aero::arm _arm);// with hand count

    // variables

  private: ros::NodeHandle nh_;

  private: aero::interface::AeroMoveitInterface::Ptr controller_;

  public: aero::vision::ObjectFeaturesPtr features_;

  public: aero_recognition_msgs::LabeledPoseArray fcn_msg_;

  private: aero_recognition_msgs::Scored2DBoxArray hand_msg_;

  private: jsk_recognition_msgs::BoundingBoxArray bounding_box_msg_;

  private: bool using_rarm_ = false;

  private: bool using_larm_ = false;

  private: ros::ServiceClient fcn_starter_;

  private: ros::Subscriber fcn_sub_;

  private: ros::Subscriber hand_sub_;

  private: ros::Subscriber box_sub_;

  private: ros::Publisher speak_pub_;

  private: aero::trajectory tra_;

  private: ros::Subscriber ar_sub_;

  private: ros::Publisher ar_start_pub_;

  private: ros::Publisher initialpose_pub_;

  private: ar_track_alvar_msgs::AlvarMarkers ar_msg_;

  private: tf::TransformListener tf_listener_;

  public: bool interaction_flag;

  public: std::mutex flag_mutex;

  };

  typedef std::shared_ptr<DevelLib> DevelLibPtr;
}

#endif
