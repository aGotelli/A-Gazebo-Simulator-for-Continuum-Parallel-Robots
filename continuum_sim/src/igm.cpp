#include <ros/ros.h>

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <continuum_sim/simpleplotting.h>
using namespace Eigen;

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

/*! Maps a vector in R3 to a 3x3 skew-symettric matrix in so3. */
inline Matrix3d hat(Vector3d y){
    Matrix3d y_hat;
    y_hat <<    0, -y(2),  y(1),
             y(2),     0, -y(0),
            -y(1),  y(0),     0;

    return y_hat;
}

const double gam = 0.15;


//Ordinary differential equation describing elastic rod
void cosseratRodOde(const std::vector<double>& y, std::vector<double> dydt, const double& t){
    //Unpack state vector
//    Matrix3d R;
//    R << y[3], y[4], y[5],
//         y[6], y[7], y[8],
//         y[9], y[10], y[11];
//    //Matrix3d R = Map<Matrix3d>(y.segment<9>(3).data());
//    Vector3d n; // = y.segment<3>(12);
//    n << y[12], y[13], y[14];
//    Vector3d m;// = y.segment<3>(15);
//    m << y[15], y[16], y[17];
//    //Hard-coded material constitutive equation w/ no precurvature
//    Vector3d v = Kse.inverse()*R.transpose()*n + Vector3d::UnitZ();
//    Vector3d u = Kbt.inverse()*R.transpose()*m;

//    //ODEs
//    Vector3d p_s = R*v;
//    Matrix3d R_s = R*hat(u);
//    Vector3d n_s = -rho*A*g;
//    Vector3d m_s = -p_s.cross(n);

//    //Pack state vector derivative
//    VectorXd y_s(18);
//    y_s << p_s, Map<VectorXd>(R_s.data(), 9), n_s, m_s;

//    dydt = std::vector<double>(y_s.data(), y_s.data()+y_s.size());

  dydt[0] = y[1];
  dydt[1] = -y[0] - gam*y[1];

}

static std::vector<double> x_history;
static std::vector<double> y_history;
void cosseratObserver(const std::vector<double>& y,  const double t )
{
    x_history.push_back(y[0]);
    y_history.push_back(y[1]);
}







int main(int argc, char** argv)
{
  ros::init(argc, argv, "igm");
  ros::NodeHandle nh;

  //Set initial conditions
  Vector3d p0 = Vector3d::Zero();
  Matrix3d R0 = Matrix3d::Identity();
  Vector3d n0 = Vector3d::UnitY();
  Vector3d m0 = Vector3d::Zero();

  VectorXd y0_(18);
  y0_ << p0, Map<VectorXd>(R0.data(), 9), n0, m0;

  std::vector<double> y0(y0_.data(), y0_.data()+y0_.size());

  //  boost::numeric::odeint::integrate(cosseratRodOde_, x_init , 0.0 , 10.0 , 0.01 );
  int N = 100;
  double ds = L/(N-1);
  auto steps = boost::numeric::odeint::integrate(cosseratRodOde, y0, 0.0, L, ds, cosseratObserver);

  VectorXd x_(x_history.size());
  VectorXd y_(y_history.size());

  for (int i = 0;i<x_history.size();i++) {
    x_[i] = x_history[i];
    y_[i] = y_history[i];
  }

  ContinuumRobotLibrary::plot( x_, y_, "Using Boost", "y (m)", "z (m)");

  ROS_INFO_STREAM("Hello world!" << steps );

}
