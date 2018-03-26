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
#include <tf/transform_listener.h>

namespace aero {

  struct item {
    std::string label;
    Eigen::Vector3d position;
  };

  class DevelLib {
  public: explicit DevelLib(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _controller, Eigen::Vector3d _cam_pos=Eigen::Vector3d(0.06, 0.04, 0.08), Eigen::Quaterniond _cam_qua=Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5));

  public: ~DevelLib();

  public: bool makeTopGrasp(const aero::arm _arm, const Eigen::Vector3d _pos, aero::trajectory& _tra);

    // grasp coffee

  public: bool pickCoffeeFront(Eigen::Vector3d _pos, float _container_height=0.80, aero::arm _arm=aero::arm::rarm, Eigen::Vector3d _offset=Eigen::Vector3d(-0.01, -0.015, 0.0));

  public: bool graspCoffee();

  public: bool placeCoffee(Eigen::Vector3d _pos=Eigen::Vector3d(0.75, -0.25, 1.05), double _offset_y=0.0, aero::arm _arm=aero::arm::rarm);

    // look motions

  public: bool poseAndRecognize(const std::string _location, const std::string _item, Eigen::Vector3d& _pos, float _lifter_z=-0.25);

  public: void lookContainerFront(float _lifter_z=-0.25);

  public: void lookShelf();

    // recognition

  public: void startFCN();

  public: void setFCNModel(std::string _name, int _projector_id=0);

  public: void setObject3DProjectorMode(int _id);

  public: std::vector<aero::item> recognizeItems();

  public: bool findItem(std::string _label, std::vector<Eigen::Vector3d> &_positions);

  public: bool findNearestItem(std::string _label,Eigen::Vector3d &_position);

  public: void stopFCN();

  private: void fcnCallback_(const aero_recognition_msgs::LabeledPoseArray::ConstPtr _msg);

    // moving

  public: bool goTo(std::string _location);

    // calculation utils

  public: Eigen::Quaterniond getRotationQuaternion(std::string _axis, double _radian);

  public: bool fastestTrajectory3(const aero::arm _arm, const std::vector<geometry_msgs::Pose> _poses, const aero::eef _eef, const Eigen::Vector3d _des_pos, aero::trajectory& _tra);

  public: int calcPathTime(std::map<aero::joint, double> _av_from, std::map<aero::joint, double> _av_to, double _factor=0.5);

  public: int calcPathTime(std::map<aero::joint, double> _av, double _factor=0.5, bool _update_model=true);

  public: std::vector<int> calcTrajectoryTimes(aero::trajectory _trajectory, double _factor=0.5, bool _update_model=true);

  public: std::vector<int> calcTrajectoryTimes(aero::trajectory _trajectory, std::vector<double> _factors, bool _update_model=true);

    // hand usage

  public: int getUsingHandsNum();

  public: bool isUsingArm(aero::arm _arm);

  public: void setUsingArm(aero::arm _arm);

  public: void unsetUsingArm(aero::arm _arm);

  public: void openHand(aero::arm _arm);// with hand count

    // variables

  private: ros::NodeHandle nh_;

  private: aero::interface::AeroMoveitInterfacePtr controller_;

  private: aero::vision::ObjectFeaturesPtr features_;

  private: aero_recognition_msgs::LabeledPoseArray fcn_msg_;

  private: bool using_rarm_ = false;

  private: bool using_larm_ = false;

  private: ros::ServiceClient fcn_starter_;

  private: ros::Subscriber fcn_sub_;
  };

  typedef std::shared_ptr<DevelLib> DevelLibPtr;
}

#endif
