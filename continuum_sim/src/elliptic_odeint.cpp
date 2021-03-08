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

using namespace std;
using namespace boost::numeric::odeint;

//typedef boost::array< double , 3 > state_type;
typedef std::vector<double> state_type;

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

ofstream out;


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

//  bulirsch_stoer_dense_out< state_type > stepper( 1E-9 , 1E-9 , 1.0 , 0.0 );

  state_type x1 = {{ 0.0 , 1.0 , 1.0 }};

  double t = 0.0;
  double dt = 0.01;

//    out.open( "elliptic1.dat" );
//    out.precision(16);
//    integrate_const( stepper , rhs , x1 , t , 100.0 , dt , write_out );
//    out.close();

//    state_type x2 = {{ 0.0 , 1.0 , 1.0 }};

//    out.open( "elliptic2.dat" );
//    out.precision(16);
//    integrate_adaptive( stepper , rhs , x2 , t , 100.0 , dt , write_out );
//    out.close();

//    typedef runge_kutta_dopri5< state_type > dopri5_type;
//    typedef controlled_runge_kutta< dopri5_type > controlled_dopri5_type;
//    typedef dense_output_runge_kutta< controlled_dopri5_type > dense_output_dopri5_type;
//    dense_output_dopri5_type dopri5( controlled_dopri5_type( default_error_checker< double >( 1E-9 , 0.0 , 0.0 , 0.0 )  ) );




  state_type x3 = {{ 0.0 , 1.0 , 1.0 }};

  integrate( rhs , x3 , t , 100.0 , dt , write_out );

  VectorXd x_(x_history.size());
  VectorXd y_(y_history.size());

  for (int i = 0;i<x_history.size();i++) {
    x_[i] = x_history[i];
    y_[i] = y_history[i];
  }

  ContinuumRobotLibrary::plot( x_, y_, "Using Boost", "y (m)", "z (m)");

  ROS_INFO("Hello world!");
}
