#include <dual_links_robot_gazebo/visual_plugin.h>
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
//inline double eval(const soft_body_demo::Coefs &coefs, double x)
//{
//  double out(0);
//  double exp(1);
//  for(auto &coef: coefs)
//  {
//    out += coef * exp;
//    exp *= x;
//  }
//  return out;
//}

Vector3d getPoint(const soft_body_demo::ShapeMsg &msg, double t)
{
  //return {eval(msg.xc, t), eval(msg.yc, t), eval(msg.zc, t)};
}

namespace gazebo
{

void RodVisualPlugin::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf)
{
//  std::cout << "Loading the visualization of the rod ivp" << std::endl;
  radius = read(sdf, "radius", 0.1);
  joint_number = read(sdf, "joint_number", -1);
  inv_scale.X(1./parent->Scale().X());
  inv_scale.Y(1./parent->Scale().Y());
  inv_scale.Z(1./parent->Scale().Z());
  std::cout << "This is the joint number : " << joint_number << std::endl;

  std::string ip = "ipc://@rod";
  ip = ip + std::to_string(joint_number);
//  std::cout << "Reading from : " << ip << std::endl;
  shape_listener.defineSock(ip);

  visual = parent;
  lines = visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  lines->setMaterial("Gazebo/Red");
  lines->setVisibilityFlags(GZ_VISIBILITY_GUI);
  // Add empty points to the line (initialization)
  for(int i = 0; i <= points; ++i)
    lines->AddPoint(Vector3d(0.0, 0.0, 0.0), ignition::math::Color::Blue);


  std::cout << "We are using : " << lines->GetPointCount() << " to represent the rod." << std::endl;
  // connect update function
  update_event = event::Events::ConnectPreRender(std::bind(&RodVisualPlugin::Update, this));
}

void RodVisualPlugin::Update()
{
  auto msg = shape_listener.lastVisual();
//  soft_body_demo::UniqueShape shape = msg.shapes[joint_number-1];

//  std::cout << "End " << joint_number << " is in : " << shape.x[99] << ", " << shape.y[99] << std::endl;

  for(int i= 0; i< points; i++){
    if(!std::isnan(msg.x[i]) && !std::isnan(msg.y[i]) && !std::isnan(msg.z[i]))
      lines->SetPoint(i, inv_scale*Vector3d(msg.x[i],msg.y[i],msg.z[i]));
  }
  lines->SetPoint(points, inv_scale*Vector3d(msg.x[points-1],msg.y[points-1],msg.z[points-1]));
  lines->Update();

  lines->Update();


}

}
