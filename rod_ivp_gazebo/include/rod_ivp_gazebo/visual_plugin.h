#ifndef SOFT_BODY_VISUAL_PLUGIN_H
#define SOFT_BODY_VISUAL_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <rod_ivp_gazebo/shape_listener.h>

using ignition::math::Vector3d;

namespace gazebo
{

class RodVisualPlugin : public VisualPlugin
{
public:
    RodVisualPlugin() {}
    ~RodVisualPlugin()
    {}

    void Load(rendering::VisualPtr parent, sdf::ElementPtr _sdf) override;
    void Update();

protected:
    rendering::VisualPtr visual;
    soft_body_demo::ShapeListener shape_listener;
    double radius;
    event::ConnectionPtr update_event;
    rendering::DynamicLines* lines;
    int points = 100;
    double scale = 1;
};
GZ_REGISTER_VISUAL_PLUGIN(RodVisualPlugin)
}


#endif // SOFT_BODY_VISUAL_PLUGIN_
