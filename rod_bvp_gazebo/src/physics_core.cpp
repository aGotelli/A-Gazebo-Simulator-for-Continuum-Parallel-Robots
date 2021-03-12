#include <rod_bvp_gazebo/physics_core.h>

#include <rod_bvp_gazebo/commonmath.h>
#include <rod_bvp_gazebo/convexoptimization.h>
#include <rod_bvp_gazebo/numericalintegration.h>



using namespace ContinuumRobotLibrary;

using namespace Eigen;

//const double pi = 3.1415926535897932384626433832795028841971L;

//Independent Parameters
const double E = 200e9;
const double G = 80e9;
const double rad = 0.001;
const double rho = 8000;
const Vector3d g = Vector3d::Zero();
const double L = 0.5;

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 3> Kse = DiagonalMatrix<double, 3>(G*A,G*A,E*A);
const DiagonalMatrix<double, 3> Kbt = DiagonalMatrix<double, 3>(E*I,E*I,G*J);



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

//Boundary conditions
static Vector3d p0(0.0, 0.0, 0.0);// = Vector3d::Zero();
const Matrix3d R0 = Matrix3d::Identity();
static Vector3d pL(0.0, 0.0, 0.0); // = Vector3d(0, -0.1*L, 0.8*L);
const Matrix3d RL = Matrix3d::Identity();
static MatrixXd Y; //Declare Y global for shooting and visualization
static VectorXd guess_;

VectorXd shootingFunction(VectorXd guess){
    guess_ = guess;
    VectorXd y0(18);

    double roll = -M_PI/2;
    Matrix3d Rx;
    Rx  <<  1,     0,         0,
            0,   cos(roll), -sin(roll),
            0,   sin(roll),  cos(roll);
    double th = guess(0);
    Matrix3d Rz;
    Rz  <<  cos(th), -sin(th),  0,
            sin(th),  cos(th),  0,
              0,         0,     1;
    Matrix3d R = Rz*Rx;

    Vector3d n;
    n << guess(1), guess(2), guess(3);
    Vector3d m;
    m << guess(4), guess(5), guess(6);
    y0 << p0, Map<VectorXd>(Matrix3d(R).data(), 9), n, m;

    Y = ode4<cosseratRodOde>(y0, L);

    //Calculate the pose (position and orientation) error at the far end
    Vector3d pL_shooting = Y.block<3,1>(0, Y.cols()-1);

    VectorXd distal_error(4);
    distal_error << pL - pL_shooting, Y(17);

    return distal_error;
}




const Vector3d n0(0.0, 0.0, 0.0);
const Vector3d m0(0.0, 0.0, 0.0);
void RodIVPPhysics::initParams(double dt, double fxy, double fz, double mz, double damping, double length, const State &link_state)
{
  std::cout << "SpringPhysics::initParams" << std::endl;
  this->fxy = fxy;
  this->dt = dt;
  this->fz = fz;
  this->mz = mz;
  this->damping = damping;
  this->length = length;

  p0 = link_state.pose.translation();
  p0.z() = 0;

}

double RodIVPPhysics::update(const State& distal_plate_state)
{

  //Set initial conditions
  pL = distal_plate_state.pose.translation();
  pL.z() = 0.0;

  double th = 0.0;
  VectorXd init_guess(7);

  init_guess << th, n0, m0;

  //Solve with shooting method
  VectorXd wrench_soln = solveLevenbergMarquardt<shootingFunction>(init_guess);

  //  Copy the values in the shape message
  //  The visual plugin works in local coordinates so we need to
  //  use the inverse of the transformation (simply subtract the joint
  //  position in the plane)
  for (unsigned int i=0;i<100;i++) {
    shape.x[i] = Y(0,i) - p0.x();
    shape.y[i] = Y(1,i) - p0.y();
    shape.z[i] = Y(2,i) - p0.z();
  }

  shape_publisher.publish(shape);

  //  Take the last theta guessed
  return guess_(0);

}

}
