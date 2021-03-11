#include <rod_bvp_gazebo/visual_plugin.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/DynamicLines.hh>

template <typename T>
inline T read(sdf::ElementPtr sdf, std::string key, T fallback)
{
  if(sdf->HasElement(key))
    return sdf->Get<T>(key);
  return fallback;
}

// evaluate a polynomial
inline double eval(const soft_body_demo::Coefs &coefs, double x)
{
  double out(0);
  double exp(1);
  for(auto &coef: coefs)
  {
    out += coef * exp;
    exp *= x;
  }
  return out;
}

Vector3d getPoint(const soft_body_demo::ShapeMsg &msg, double t)
{
  return {eval(msg.xc, t), eval(msg.yc, t), eval(msg.zc, t)};
}

namespace gazebo
{

void RodVisualPlugin::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf)
{
  std::cout << "Loading the visualization of the rod ivp" << std::endl;
  radius = read(sdf, "radius", 0.1);
  scale = read(sdf, "scale", 1.0);

  visual = parent;
  lines = visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  lines->setMaterial("Gazebo/Red");
  lines->setVisibilityFlags(GZ_VISIBILITY_GUI);
  // Add empty points to the line (initialization)
  for(int i = 0; i <= points; ++i)
    lines->AddPoint({}, ignition::math::Color::Blue);

  std::cout << "There are : " << lines->GetPointCount() <<"points in the line" <<std::endl;
  std::cout << "We are using : " << points << "elements." << std::endl;
  // connect update function
  update_event = event::Events::ConnectPreRender(std::bind(&RodVisualPlugin::Update, this));
}

void RodVisualPlugin::Update()
{
  auto msg = shape_listener.lastVisual();


  for(int i= 0; i< points; i++)
    lines->SetPoint(i, scale*Vector3d(msg.x[i],msg.y[i],msg.z[i]));
  lines->SetPoint(points, scale*Vector3d(msg.x[points-1],msg.y[points-1],msg.z[points-1]));
  lines->Update();

}

}
