#ifndef SOFT_BODY_PHYSICS_CORE_H
#define SOFT_BODY_PHYSICS_CORE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dual_links_robot_gazebo/shape_publisher.h>


namespace soft_body_demo
{

// io of physics model

struct State
{
  struct Twist
  {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero(), angular = Eigen::Vector3d::Zero();
  };

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Twist twist;
  State() {}
  State(const Eigen::Isometry3d &pose, const Eigen::Vector3d &linear_vel, const Eigen::Vector3d &angular_vel)
    : pose(pose), twist{linear_vel, angular_vel} {}
  void updateTranslation(double x, double y, double z)
  {
    pose.translation() << x, y, z;
  }
  void updateRotation(double w, double x, double y, double z)
  {    
    pose.linear() = Eigen::Matrix3d(Eigen::Quaterniond(w, x, y, z));
  }
};

struct Wrench
{
  Eigen::Vector3d force;
  Eigen::Vector3d torque;
  Wrench() : force(Eigen::Vector3d::Zero()), torque(Eigen::Vector3d::Zero()) {}
  Wrench(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    : force(force), torque(torque) {}
};

class RodIVPPhysics
{
public:

  RodIVPPhysics() {
    shape_rod1.defineSock("ipc://@rod1");
    shape_rod2.defineSock("ipc://@rod2");
  }

  void initParams(double dt, double fxy, double fz, double mz, double damping, double length, const State &link1_state, const State &link2_state);

  Wrench update(const State &state);





private:
  Eigen::Vector3d attach;
  //ShapePublisher shape_publisher;
  ShapePublisher shape_rod1;
  ShapePublisher shape_rod2;
  ShapeMsg shape1;
  ShapeMsg shape2;

  // sampling time
  double dt;

  // physics params
  double fxy ,fz, mz, damping, length;


};

}


#endif
