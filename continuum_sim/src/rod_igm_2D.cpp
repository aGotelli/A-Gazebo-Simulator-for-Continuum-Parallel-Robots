#include <ros/ros.h>


#include <iostream>
#include <fstream>
#include <cmath>

#include <boost/array.hpp>

#include <boost/numeric/odeint/config.hpp>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer_dense_out.hpp>

#include <continuum_sim/simpleplotting.h>
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace boost::numeric::odeint;

typedef Eigen::Matrix<double, 6, 1 > state_type;


const double pi = 3.1415926535897932384626433832795028841971L;


//Independent Parameters
const double E = 200e9;
const double G = 80e9;
const double rad = 0.001;
const double rho = 8000;
const Vector3d g = 9.81*Vector3d::UnitX();
const double L = 0.5;

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 3> Kse = DiagonalMatrix<double, 3>(G*A,G*A,E*A);
const DiagonalMatrix<double, 3> Kbt = DiagonalMatrix<double, 3>(E*I,E*I,G*J);


const Vector2d f(0.0, 0.0);
const double l = 0.0;


inline Matrix3d hat(Vector3d y){
    Matrix3d y_hat;
    y_hat <<    0, -y(2),  y(1),
             y(2),     0, -y(0),
            -y(1),  y(0),     0;

    return y_hat;
}

//Ordinary differential equation describing elastic rod

void cosseratRodOde(const state_type& y , state_type &dxdt , const double  t )
{
  double th = y(2);

  //Unpack state vector
  Matrix3d R;
  R   <<  cos(th), -sin(th),  0,
          sin(th),  cos(th),  0,
            0,        0,      1;
  Vector3d n = y.segment<3>(12);
  Vector3d m = y.segment<3>(15);

  //Hard-coded material constitutive equation w/ no precurvature
  Vector3d v = Kse.inverse()*R.transpose()*n + Vector3d::UnitY();
  Vector3d u = Kbt.inverse()*R.transpose()*m;

  //ODEs
  Vector3d p_s = R*v;
  Matrix3d R_s = R*hat(u);
  Vector3d n_s = -rho*A*g;
  Vector3d m_s = -p_s.cross(n);

  Vector3d ea = R_s.eulerAngles(0, 1, 2);

  double th_s = ea(2);

  //Pack state vector derivative
  VectorXd y_s(18);
  y_s << p_s(0), p_s(1), th_s, n_s(0), n_s(1), m_s(3);

  //ROS_INFO_STREAM(y(0), Y(1))

  dxdt = y_s;

}



static std::vector<double> x_history;
static std::vector<double> y_history;
static std::vector<double> z_history;
void write_out( const state_type &x , const double t )
{
    x_history.push_back(x[0]);
    y_history.push_back(x[1]);
    z_history.push_back(x[2]);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "elliptic_odeint");
  ros::NodeHandle nh;

  typedef runge_kutta_dopri5<state_type, double, state_type, double, vector_space_algebra> stepper;
//  typedef runge_kutta4<state_type, double, state_type, double, vector_space_algebra> stepper2;
//  typedef runge_kutta4_classic<state_type, double, state_type, double, vector_space_algebra> stepper3;

  //Set initial conditions
  Vector2d p0 = Vector2d::Zero();
  double th0 = 0.0;
  Vector2d n0;
  n0 << 0, 0;
  double m0 = 0.0;
  double ds = 0.005;

  VectorXd y0(6);
  y0 << p0, th0, n0, m0;

  integrate_adaptive(stepper(), cosseratRodOde , y0 , 0.0 , L , ds , write_out );

  ROS_INFO_STREAM("end rod pos: " << x_history.back() << ", " << y_history.back() << ", " << z_history.back());

  VectorXd x_(x_history.size());
  VectorXd y_(y_history.size());
  VectorXd z_(z_history.size());

  for (int i = 0;i<x_history.size();i++) {
    x_[i] = x_history[i];
    y_[i] = y_history[i];
    z_[i] = z_history[i];
  }

  ContinuumRobotLibrary::plot(x_, y_, "Using Boost and Eigen", "y (m)", "z (m)");

  ROS_INFO("Hello world!");
}
