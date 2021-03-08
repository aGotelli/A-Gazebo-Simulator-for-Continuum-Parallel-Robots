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
const Vector3d g = -9.81*Vector3d::Zero();
const double ee_mass = 0.1;
const double scrib_R = 0.087; //radius of the hole pattern
const double alpha1 = 100*pi/180; //major angle of the hole pattern
const double L1 = 1.0;
const double L2 = 1.0;

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 3> Kse = DiagonalMatrix<double, 3>(G*A,G*A,E*A);
const DiagonalMatrix<double, 3> Kbt = DiagonalMatrix<double, 3>(E*I,E*I,G*J);
const Vector3d F = Vector3d::Zero();
const Vector3d M = Vector3d::Zero();
const Vector3d pE(0.0, 0.3, 0.6);
const Matrix3d RE = DiagonalMatrix<double, 3>(1, 1, 1);
const double alpha2 = 120*pi/180 - alpha1; //minor angle of the hole pattern

const Vector3d p10(0.0, -0.2, 0.0);
const Vector3d p20(0.0, +0.2, 0.0);

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



static MatrixXd Y1;
static MatrixXd Y2;
static double th1;
static double th2;
VectorXd shootingFunction(VectorXd guess)
{
    //  Unpack the guess vector
    th1 = guess(0);
    Matrix3d R1;
    R1 <<  1,     0    ,     0     ,
           0,  cos(th1),  -sin(th1),
           0,  sin(th1),  cos(th1) ;
    Vector3d n1;
    n1 << guess(1), guess(2), guess(3);
    Vector3d m1;
    m1 << guess(4), guess(5), guess(6);
    //  Compose state for the cosserat rod model
    VectorXd y1(18);
    y1 << p10, Map<VectorXd>(Matrix3d(R1).data(), 9), n1, m1;

    //  Unpack the guess vector
    th2 = guess(7);
    Matrix3d R2;
    R2 <<  1,     0    ,     0     ,
           0,  cos(th2),  -sin(th2),
           0,  sin(th2),  cos(th2) ;
    Vector3d n2;
    n2 << guess(8), guess(9), guess(10);
    Vector3d m2;
    m2 << guess(11), guess(12), guess(13);
    //  Compose state for the cosserat rod model
    VectorXd y2(18);
    y2 << p20, Map<VectorXd>(Matrix3d(R2).data(), 9), n2, m2;

    Y1 = ode4<cosseratRodOde>(y1, L1);
    Y2 = ode4<cosseratRodOde>(y2, L2);


    VectorXd residual(11);

    //  Geometrical constraints
    Vector3d pL_shooting1 = Y1.block<3,1>(0, Y1.cols()-1);
    Vector3d pL_shooting2 = Y2.block<3,1>(0, Y2.cols()-1);

    Vector3d distal_error = pL_shooting1 - pE;
    Vector3d check_closure= pL_shooting1 - pL_shooting2;

    //  Equilibrium considerations
    Vector3d nL1(Y1.block<3,1>(12, Y1.cols()-1));
    Vector3d nL2(Y2.block<3,1>(12, Y2.cols()-1));
    Vector3d mL1(Y1.block<3,1>(15, Y1.cols()-1));
    Vector3d mL2(Y2.block<3,1>(15, Y2.cols()-1));

    Vector3d res_force = nL1 + nL2 + F;

    //  Compose the residual vector;
    //residual << distal_error, check_closure, res_force, Y1(17), Y2(17);
    residual << distal_error, check_closure, res_force, mL1.z(), mL2.z();

    return residual;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "dual_arm_planar");
    ros::NodeHandle nh;

    Vector3d n10 = Vector3d::Zero();
    Vector3d m10 = Vector3d::Zero();
    Vector3d n20 = Vector3d::Zero();
    Vector3d m20 = Vector3d::Zero();

    double th10 = 0.0;
    double th20 = -M_PI/12;

//    m10 << 0.0, 0.0, -1.2;
//    m20 << 0.0, 0.0, 1.2;


    VectorXd init_guess(14);
    init_guess << th10, n10, m10, th20, n20, m20;

    //Solve with shooting method
    solveLevenbergMarquardt<shootingFunction>(init_guess);

    //ROS_INFO_STREAM("OK FINALYYY!!!" << h1[0].size());

    //ROS_INFO_STREAM("end rod pos: " << h2[0].back()<< ", " << h2[1].back() << "mz(0) : " << h2[4][0] << "mz(L) : " << h2[4].back());
//    double size1 = Y1.cols();
//    double size2 = Y2.cols();
//    VectorXd x(size1 + size2);
//    x << Y1.row(0), Y2.row(0);
//    VectorXd y(size1 + size2);
//    y << Y1.row(1), Y2.row(1);
//    VectorXd z(size1 + size2);
//    z << Y1.row(2), Y2.row(2);
    std::cout << "Angles of the joints : " << th1 << ", " << th2 << std::endl;
    ContinuumRobotLibrary::plot(Y1.row(1), Y1.row(2),Y2.row(1), Y2.row(2), "Two Flexible Links Robot", "y (m)", "z (m)");
    //ContinuumRobotLibrary::plot(Y2.row(1), Y2.row(2), "Cosserat Rod BVP Solution", "y (m)", "z (m)");
    std::cout << "Solved rod1 centerline:\n" << Y1.block(0,0,3,Y1.cols()) << std::endl;

    //std::cout << "Solved rod2 centerline:\n" << Y2.block(0,0,3,Y2.cols()) << std::endl;

    return 0;
}
