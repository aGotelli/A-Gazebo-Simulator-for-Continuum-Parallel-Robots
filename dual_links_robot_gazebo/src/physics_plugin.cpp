#include <dual_links_robot_gazebo/physics_plugin.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <Eigen/Geometry>

template <typename T>
inline T read(sdf::ElementPtr sdf, std::string key, T fallback)
{
  if(sdf->HasElement(key))
    return sdf->Get<T>(key);
  return fallback;
}
using namespace std;
namespace gazebo
{

void SpringModelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{

  distal_plate = model->GetLinks()[0];


  std::cout << "Loading soft body physics" << std::endl;
  Eigen::Vector3d joint1_pos, joint2_pos;
  std::cout << "links[i]->GetName()" << std::endl;
  auto links = model->GetLinks();
  for(unsigned int i=0; i< links.size(); i++) {
    cout << links[i]->GetName();
    auto pose = links[i]->WorldPose();
    cout << " position : " << pose.X() << ", " << pose.Y() << endl;
  }
  auto world = model->GetWorld();
  auto joint1 = world->ModelByName("joint1")->GetLinks();
  while(joint1[0] == nullptr)
    joint1 = world->ModelByName("joint1")->GetLinks();
  cout << "Getting info about joint1" << endl;
  for (unsigned int i=0; i<joint1.size();i++) {
    cout << joint1[i]->GetName();
    auto pose = joint1[i]->WorldPose();
    cout << " position : " << pose.X() << ", " << pose.Y() << endl;
    joint1_pos << pose.X(), pose.Y(), pose.Z();
  }
  auto joint2 = world->ModelByName("joint2")->GetLinks();
  while(joint2[0] == nullptr)
    joint2 = world->ModelByName("joint2")->GetLinks();
  cout << "Getting info about joint2"  << endl;
  for (unsigned int i=0; i<joint2.size();i++) {
    cout << joint2[i]->GetName();
    auto pose = joint2[i]->WorldPose();
    cout << " position : " << pose.X() << ", " << pose.Y() << endl;
    joint2_pos << pose.X(), pose.Y(), pose.Z();
  }
//  ignition::math::Pose3d pose;
//  ignition::math::Quaterniond q;
//  q.Euler(Vector3d(0,0,M_PI/3));
//  pose.Set(Vector3d(0,0,0), q);
//  joint2[0]->SetWorldPose(pose);
  joint1_ = joint1[0];
  joint2_ = joint2[0];




//  ee = model->GetLinks()[0];
//  auto base1 = model->GetWorld()->ModelByName("base1")->GetLinks()[0];
//  auto base2 = model->GetWorld()->ModelByName("base2")->GetLinks()[0];





  // model params are passed through plugin definition in SDF
  robot_model.initParams(distal_plate->GetWorld()->Physics()->GetMaxStepSize(),  // dt from Gazebo engine
                          read(sdf, "fxy", 10.),
                          read(sdf, "fz", 10.),
                          read(sdf, "mz", 10.),
                          read(sdf, "damping", 0.01),
                          read(sdf, "length", 1.),
                          joint1_pos, joint2_pos);

//  attach = read(sdf, "attach_point", Vector3d());

//  // connect update function
  update_event = event::Events::ConnectWorldUpdateBegin(std::bind(&SpringModelPlugin::Update, this));
  cout << "End of Load " << endl;
}

void SpringModelPlugin::Update()
{
  auto pose = distal_plate->WorldPose();
  Eigen::Vector3d distal_plate_pos;
  distal_plate_pos << pose.X(), pose.Y(), pose.Z();

  const auto joint_positions = robot_model.update(distal_plate_pos);

  joint1_->SetWorldPose(ignition::math::Pose3d(joint1_->WorldPose().Pos(),
                          ignition::math::Quaterniond(0,0,joint_positions[0])));
  joint2_->SetWorldPose(ignition::math::Pose3d(joint2_->WorldPose().Pos(),
                          ignition::math::Quaterniond(0,0,joint_positions[1])));
//  rev_joint1->SetPosition(3, M_PI/3);
//  cout << rev_joint1->Position() << endl;
}


}
