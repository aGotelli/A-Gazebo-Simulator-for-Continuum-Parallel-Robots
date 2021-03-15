#include <ros/ros.h>

#include <iostream>
#include <continuum_sim/commonmath.h>
#include <continuum_sim/numericalintegration.h>
#include <continuum_sim/convexoptimization.h>


#include <boost/array.hpp>

#include <boost/numeric/odeint/config.hpp>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer_dense_out.hpp>

using namespace ContinuumRobotLibrary;

#ifdef QT_CORE_LIB
#include <continuum_sim/simpleplotting.h>
#endif


typedef Eigen::Matrix<double, 15, 1 > state_type;



//Independent Parameters
const double E = 200e9;
const double G = 80e9;
const double rad = 0.001;
const double L1 = 1.0;
const double L2 = 1.0;

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 2> Kse = DiagonalMatrix<double, 2>(G*A, E*A);
const double Kbt = E*I;

const Vector2d F(0.0, 0.0);// = Vector3d::Zero();
const Vector2d pE(0.3, 0.6);


const Vector2d p10(-0.2, 0.0);
const Vector2d p20(+0.2, 0.0);


//Ordinary differential equation describing elastic rod
VectorXd cosseratRodOde(VectorXd y){
//       0  1   2   3   4  5
//  y << x0 y0 th0 nx0 ny0 m0
    //Unpack state vector
    const Rotation2D<double> R(y(2));
    const Vector2d n = y.segment<2>(3);
    const double m = y(5);

    //Hard-coded material constitutive equation w/ no precurvature
    const Vector2d v = Kse.inverse()*R.toRotationMatrix().transpose()*n + Vector2d::UnitY();

    //ODEs
    const Vector2d p_s = R*v;
    const double theta_s = m/Kbt;
    const Vector2d n_s = Vector2d::Zero();
    const double m_s = - p_s(0)*n(1) + p_s(1)*n(0);

    //Pack state vector derivative
    VectorXd y_s(6);
    y_s << p_s, theta_s, n_s, m_s;

    return y_s;
}


static MatrixXd Y1;
static MatrixXd Y2;
static double th1;
static double th2;
VectorXd shootingFunction(VectorXd guess)
{
    VectorXd y1(6);
    y1 << p10, guess.segment<4>(0);

    VectorXd y2(6);
    y2 << p20, guess.segment<4>(4);

    Y1 = ode4<cosseratRodOde>(y1, L1);
    Y2 = ode4<cosseratRodOde>(y2, L2);


    VectorXd residual(8);

    //  Geometrical constraints
    Vector2d pL_shooting1 = Y1.block<2,1>(0, Y1.cols()-1);
    Vector2d pL_shooting2 = Y2.block<2,1>(0, Y2.cols()-1);

    Vector2d distal_error = pL_shooting1 - pE;
    Vector2d check_closure= pL_shooting1 - pL_shooting2;

    //  Equilibrium considerations
    Vector2d nL1(Y1.block<2,1>(3, Y1.cols()-1));
    Vector2d nL2(Y2.block<2,1>(3, Y2.cols()-1));
    double mL1 = Y1(5, Y1.cols()-1);
    double mL2 = Y2(5, Y2.cols()-1);

    Vector2d res_force = nL1 + nL2 + F;

    //  Compose the residual vector;
    residual << distal_error, check_closure, res_force, mL1, mL2;


    return residual;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "dual_arm_planar");
    ros::NodeHandle nh;

    Vector2d n10 = Vector2d::Zero();
    double m10 = 0.0;
    Vector2d n20 = Vector2d::Zero();
    double m20 = 0.0;

    double th10 = 0.0;
    double th20 = 0.0;


    VectorXd init_guess(8);
    init_guess << th10, n10, m10, th20, n20, m20;

    //Solve with shooting method
    solveLevenbergMarquardt<shootingFunction>(init_guess);


    std::cout << "Angles of the joints : " << th1 << ", " << th2 << std::endl;
    ContinuumRobotLibrary::plot(Y1.row(0), Y1.row(1),Y2.row(0), Y2.row(1), "Two Flexible Links Robot", "x (m)", "y (m)");

    return 0;
}
