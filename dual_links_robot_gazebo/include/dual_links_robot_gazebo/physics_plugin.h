#ifndef DUAL_LINKS_ROBOT_PHYSICS_PLUGIN_H
#define DUAL_LINKS_ROBOT_PHYSICS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <dual_links_robot_gazebo/physics_core.h>

using ignition::math::Vector3d;

namespace gazebo
{

class SpringModelPlugin : public ModelPlugin
{
public:
    SpringModelPlugin() {}
    ~SpringModelPlugin()
    {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void Update();

protected:
    soft_body_demo::RodIVPPhysics robot_model;
    soft_body_demo::State state;
    physics::LinkPtr distal_plate, joint1_, joint2_;
    physics::JointPtr rev_joint1;
    double length;
    double mxy, mz, fxy, fz, damping;
    Vector3d attach;
    event::ConnectionPtr update_event;
};

GZ_REGISTER_MODEL_PLUGIN(SpringModelPlugin)
}


#endif // DUAL_LINKS_ROBOT_PHYSICS_PLUGIN_H
