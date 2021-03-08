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

typedef Eigen::Matrix<double, 3, 1 > state_type;



/*
 * x1' = x2*x3
 * x2' = -x1*x3
 * x3' = -m*x1*x2
 */

void rhs( const state_type &x , state_type &dxdt , const double t )
{
    static const double m = 0.51;

    dxdt[0] = x[1]*x[2];
    dxdt[1] = -x[0]*x[2];
    dxdt[2] = -m*x[0]*x[1];
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

  double t = 0.0;
  double dt = 0.01;


  typedef runge_kutta_dopri5<state_type, double, state_type, double, vector_space_algebra> stepper;

//  state_type x = {{ 0.0 , 1.0 , 1.0 }};
  Eigen::VectorXd x(3, 1);
  x << 0.0 , 1.0 , 1.0 ;


  integrate_adaptive(stepper(), rhs , x , t , 100.0 , dt , write_out );

  VectorXd x_(x_history.size());
  VectorXd y_(y_history.size());

  for (int i = 0;i<x_history.size();i++) {
    x_[i] = x_history[i];
    y_[i] = y_history[i];
  }

  ContinuumRobotLibrary::plot( x_, y_, "Using Boost", "y (m)", "z (m)");

  ROS_INFO("Hello world!");
}
