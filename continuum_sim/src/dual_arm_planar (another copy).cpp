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
const double rho = 8000;
const double g = 0.0;
const double alpha1 = 100*pi/180; //major angle of the hole pattern
const double L1 = 1.0;
const double L2 = 1.0;

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 2> Kse = DiagonalMatrix<double, 2>(G*A,E*A);
const double Kbt = E*I;
const Vector2d F(0.0, 0.0);// = Vector3d::Zero();
const double M = 0.0;
const Vector2d pE(0.3, 0.7);
const Matrix2d RE = DiagonalMatrix<double, 2>(1, 1);


const Vector3d p10(-0.2, 0.0);
const Vector3d p20(+0.2, 0.0);

//Ordinary differential equation describing elastic rod
VectorXd cosseratRodOde(VectorXd y){
    //Unpack state vector
    Matrix2d R = Map<Matrix2d>(y.segment<4>(2).data());
    Vector2d n = y.segment<2>(6);
    double m = y(7);

    //Hard-coded material constitutive equation w/ no precurvature
    Vector2d v = Kse.inverse()*R.transpose()*n + Vector2d::UnitY();
    double u = m/Kbt;

    //ODEs
    Vector2d p_s = R*v;
    Matrix2d R_s = R*hat(u);
    Vector2d n_s = Vector2d::Zero();
    Vector2d m_s = -p_s.cross(n);

    //Pack state vector derivative
    VectorXd y_s(18);
    y_s << p_s, Map<VectorXd>(R_s.data(), 9), n_s, m_s;

    return y_s;
}


static MatrixXd Y1;
static MatrixXd Y2;
static double th1;
static double th2;
VectorXd shootingFunction(VectorXd guess)
{
    double roll = -M_PI/2;
    Matrix3d Rx;
    Rx  <<  1,     0,         0,
            0,   cos(roll), -sin(roll),
            0,   sin(roll),  cos(roll);

    //  Unpack the guess vector
    th1 = guess(0);
    Matrix2d R1;
    R1  <<  cos(th1), -sin(th1),
            sin(th1),  cos(th1);
    Vector2d n1;
    n1 << guess(1), guess(2);
    double m1 = guess(3);

    //  Compose state for the cosserat rod model
    VectorXd y1(9);
    y1 << p10, Map<VectorXd>(Matrix2d(R1).data(), 4), n1, m1;

    //  Unpack the guess vector
    th2 = guess(4);
    Matrix2d R2;
    R2 <<   cos(th2), -sin(th2),
            sin(th2),  cos(th2);
    Vector2d n2;
    n2 << guess(5), guess(6);
    double m2 = guess(7);

    //  Compose state for the cosserat rod model
    VectorXd y2(9);
    y2 << p20, Map<VectorXd>(Matrix2d(R2).data(), 4), n2, m2;

    Y1 = ode4<cosseratRodOde>(y1, L1);
    Y2 = ode4<cosseratRodOde>(y2, L2);


    VectorXd residual(8);

    //  Geometrical constraints
    Vector2d pL_shooting1 = Y1.block<2,1>(0, Y1.cols()-1);
    Vector2d pL_shooting2 = Y2.block<2,1>(0, Y2.cols()-1);

    Vector2d distal_error = pL_shooting1 - pE;
    Vector2d check_closure= pL_shooting1 - pL_shooting2;

    //  Equilibrium considerations
    Vector2d nL1(Y1.block<2,1>(6, Y1.cols()-1));
    Vector2d nL2(Y2.block<2,1>(6, Y2.cols()-1));
    double mL1(Y1.block<2,1>(8, Y1.cols()-1));
    double mL2(Y2.block<2,1>(8, Y2.cols()-1));

    Vector2d res_force = nL1 + nL2 + F;

    //  Compose the residual vector;
    residual << distal_error, check_closure, res_force, mL1.z(), mL2.z();


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
//                  0   1 2   3     4   5 6   7
    init_guess << th10, n10, m10, th20, n20, m20;
//                  1   2    1     1    2    1
    //Solve with shooting method
    solveLevenbergMarquardt<shootingFunction>(init_guess);


    std::cout << "Angles of the joints : " << th1 << ", " << th2 << std::endl;
    ContinuumRobotLibrary::plot(Y1.row(0), Y1.row(1),Y2.row(0), Y2.row(1), "Two Flexible Links Robot", "x (m)", "y (m)");
    ContinuumRobotLibrary::plot(Y1.row(1), Y1.row(2),Y2.row(1), Y2.row(2), "Two Flexible Links Robot", "y (m)", "z (m)");
    //ContinuumRobotLibrary::plot(Y2.row(1), Y2.row(2), "Cosserat Rod BVP Solution", "y (m)", "z (m)");
    std::cout << "Solved rod1 centerline:\n" << Y1.block(0,0,3,Y1.cols()) << std::endl;

    //std::cout << "Solved rod2 centerline:\n" << Y2.block(0,0,3,Y2.cols()) << std::endl;

    return 0;
}
