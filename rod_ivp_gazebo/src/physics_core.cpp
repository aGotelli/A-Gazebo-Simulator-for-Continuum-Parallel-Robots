#include <rod_ivp_gazebo/physics_core.h>

#include <rod_ivp_gazebo/numericalintegration.h>
const double pi = 3.1415926535897932384626433832795028841971L;

using namespace Eigen;

//Independent Parameters
const double E = 200e9;
const double G = 80e9;
const double rad = 0.001;
const double rho = 8000;
const Vector3d g = Vector3d::UnitZ();
const double L = 0.5;

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 3> Kse = DiagonalMatrix<double, 3>(G*A,G*A,E*A);
const DiagonalMatrix<double, 3> Kbt = DiagonalMatrix<double, 3>(E*I,E*I,G*J);

inline Eigen::Matrix3d hat(Eigen::Vector3d y){
    Eigen::Matrix3d y_hat;
    y_hat <<    0, -y(2),  y(1),
             y(2),     0, -y(0),
            -y(1),  y(0),     0;

    return y_hat;
}



namespace soft_body_demo
{

//Ordinary differential equation describing elastic rod
VectorXd cosseratRodOde(VectorXd y){
    //Unpack state vector
    Matrix3d R = Map<Matrix3d>(y.segment<9>(3).data());
    Vector3d n = y.segment<3>(12);
    Vector3d m = y.segment<3>(15);

    //Hard-coded material constitutive equation w/ no precurvature
    Vector3d v = Kse.inverse()*R.transpose()*n + Vector3d::UnitZ();
    Vector3d u = Kbt.inverse()*R.transpose()*m;

    //ODEs
    Vector3d p_s = R*v;
    Matrix3d R_s = R*hat(u);
    Vector3d n_s = -rho*A*g;
    Vector3d m_s = -p_s.cross(n);

    //Pack state vector derivative
    VectorXd y_s(18);
    y_s << p_s, Map<VectorXd>(R_s.data(), 9), n_s, m_s;

    return y_s;
}


void RodIVPPhysics::initParams(double dt, double fxy, double fz, double mz, double damping, double length)
{
std::cout << "SpringPhysics::initParams" << std::endl;
  this->fxy = fxy;
  this->dt = dt;
  this->fz = fz;
  this->mz = mz;
  this->damping = damping;
  this->length = length;

}





Wrench RodIVPPhysics::update(const State &state)
{
  //Set initial conditions

  auto pose = state.pose.inverse(); //Vector3d::Zero();
  Vector3d p0 = pose.translation();
  Matrix3d R0 = Matrix3d::Identity();

  Vector3d n0;// = Vector3d::UnitY();
  n0 << 1, 0, 0;
  Vector3d m0; // = Vector3d::Zero();
  m0 << 0, 0, 0;

  VectorXd y0(18);
  y0 << p0, Map<VectorXd>(R0.data(), 9), n0, m0;

  //Numerically integrate the Cosserat rod equations
  MatrixXd Y = ContinuumRobotLibrary::ode4<cosseratRodOde>(y0, L);

  for (unsigned int i=0;i<100;i++) {
    shape.x[i] = Y(0,i);
    shape.y[i] = Y(1,i);
    shape.z[i] = Y(2,i);
  }

  shape_publisher.publish(shape);

  return Wrench();

}


}
