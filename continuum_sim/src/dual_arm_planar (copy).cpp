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

//Dependent parameter calculations
const double A = pi*pow(rad,2);
const double I = pi*pow(rad,4)/4;
const double J = 2*I;
const DiagonalMatrix<double, 3> Kse = DiagonalMatrix<double, 3>(G*A,G*A,E*A);
const DiagonalMatrix<double, 3> Kbt = DiagonalMatrix<double, 3>(E*I,E*I,G*J);
const Vector3d F = Vector3d::Zero();
const Vector3d M = Vector3d::Zero();
const Vector3d pE(0.0, 0.0, 1);
const Matrix3d RE = DiagonalMatrix<double, 3>(1, 1, 1);
const double alpha2 = 120*pi/180 - alpha1; //minor angle of the hole pattern

const Vector3d p10(0.0, -0.2, 0.0);
const Vector3d p20(0.0, +0.2, 0.0);

void cosseratRodOde(const state_type& y , state_type &dxdt , const double  t )
{
  //init_guess << px, py, px, Map<VectorXd>(R10.data(), 9), nx, ny, mz;
    //Unpack state vector
    Matrix3d R;
    R << y[3], y[6], y[9],
         y[4], y[7], y[10],
         y[5], y[8], y[11];
    Vector3d n;// = y.segment<3>(12);
    n << y(12), y(13), 0;
    Vector3d m;// = y.segment<3>(15);
    m << 0, 0, y(14);

    //Hard-coded material constitutive equation w/ no precurvature
    Vector3d v = Kse.inverse()*R.transpose()*n + Vector3d::UnitZ();
    Vector3d u = Kbt.inverse()*R.transpose()*m;

    //ODEs
    Vector3d p_s = R*v;
    Matrix3d R_s = R*hat(u);
    Vector3d n_s = -rho*A*g;
    Vector3d m_s = -p_s.cross(n);
    //Pack state vector derivative
    VectorXd y_s(15);
    y_s << p_s, Map<VectorXd>(R_s.data(), 9), n_s.x(), n_s.y(), m_s.z();

    dxdt = y_s;
}

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

static std::vector<std::vector<double>> h1(5);
static std::vector<std::vector<double>> h2(5);
void write_out1( const state_type &x , const double t )
{
  std::cout << x(0) << std::endl;
  //      0   1   2   3  4  5  6  7  8  9  10  11   12  13  14
  //y0 << px, py, px, Map<VectorXd>(R10.data(), 9), nx, ny, mz;
    h1[0].push_back(x(0));  //px
    h1[1].push_back(x(1));  //py
    h1[2].push_back(x(12));  //nx
    h1[3].push_back(x(13));  //ny
    h1[4].push_back(x(14));  //mz

}

void write_out2( const state_type &x , const double t )
{
  //      0   1   2   3  4  5  6  7  8  9  10  11   12  13  14
  //y0 << px, py, px, Map<VectorXd>(R10.data(), 9), nx, ny, mz;
    h2[0].push_back(x(0));  //px
    h2[1].push_back(x(1));  //py
    h2[2].push_back(x(12));  //nx
    h2[3].push_back(x(13));  //ny
    h2[4].push_back(x(14));  //mz
}

//Shooting method objective function
const int N = 100;
static std::vector<VectorXd> px(12);
static std::vector<VectorXd> pz(12);
static MatrixXd p(3*6,N);


static MatrixXd Y1;
static MatrixXd Y2;
VectorXd shootingFunction(VectorXd guess){

    for(auto& vec : h1) {
      vec.clear();
    }
    for(auto& vec : h2) {
      vec.clear();
    }

    double L1 = 1.0;
    VectorXd y01(18);
    //y0 << px, py, px, Map<VectorXd>(R10.data(), 9), nx, ny, mz;
    y01 << p10, guess.segment<15>(0);

    double L2 = 1.0;
    VectorXd y02(18);
    y02 << p20, guess.segment<15>(15);

//    double ds = 0.005;
//    typedef boost::numeric::odeint::runge_kutta_dopri5<state_type, double, state_type, double, boost::numeric::odeint::vector_space_algebra> stepper;
//    boost::numeric::odeint::integrate_adaptive(stepper(), cosseratRodOde , y01 , 0.0 , L1 , ds , write_out1 );
//    boost::numeric::odeint::integrate_adaptive(stepper(), cosseratRodOde , y02 , 0.0 , L2 , ds , write_out2 );

    Y1 = ode4<cosseratRodOde>(y01, L1);
    Y2 = ode4<cosseratRodOde>(y02, L2);
/*
    VectorXd residual(8);

    //  Check Closure loop
    Vector3d p1L;
    p1L << h1[0].back(), h1[1].back();
    Vector3d p2L;
    p2L << h2[0].back(), h2[1].back();
    residual.segment<2>(0) = p1L - p2L;

    //  Check Equilibrium
    Vector3d n1L;
    n1L << h1[2].back(), h1[3].back();
    Vector3d n2L;
    n2L << h2[2].back(), h2[3].back();
    residual.segment<2>(2) = n1L + n2L + F;

    //  Check torques = 0
    double m1L =  h1[4].back();
    residual(4) = m1L;
    double m2L =  h2[4].back();
    residual(5) = m2L;

    //  Check e.e. position
    residual.segment<2>(6) = p1L - pE;
*/
    VectorXd residual(11);

    //  Geometrical constraints
    Vector3d pL_shooting1 = Y1.block<3,1>(0, Y1.cols()-1);
    Vector3d pL_shooting2 = Y2.block<3,1>(0, Y2.cols()-1);

    Vector3d distal_error = pL_shooting1 - pE;
    Vector3d check_closure= pL_shooting1 - pL_shooting2;

    //  Equilibrium considerations
    Vector3d nL1(Y1.block<3,1>(12, Y1.cols()-1));
    Vector3d nL2(Y2.block<3,1>(12, Y2.cols()-1));

    Vector3d res_force = nL1 + nL2 + F;

    //  Compose the residual vector;
    residual << distal_error, check_closure, res_force, Y1(17), Y2(17);

    return residual;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "dual_arm_planar");
    ros::NodeHandle nh;

    Vector3d n10 = Vector3d::Zero();
    Vector3d m10 = Vector3d::Zero();
    Vector3d n20 = Vector3d::Zero();
    Vector3d m20 = Vector3d::Zero();


    Matrix3d R10 = Matrix3d::Identity();
//    R10 <<  cos(pi/2), -sin(pi/2),  0,
//            sin(pi/2),  cos(pi/2),  0,
//              0,           0,       1;
    double th = -M_PI/2;
    Matrix3d R20;
    R20 <<  cos(th), -sin(th),  0,
            sin(th),  cos(th),  0,
              0,         0,     1;

    VectorXd init_guess(30);
    init_guess << Map<VectorXd>(R10.data(), 9), n10, m10, Map<VectorXd>(R20.data(), 9), n20, m20;

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
    ContinuumRobotLibrary::plot(Y1.row(1), Y1.row(2),Y2.row(1), Y2.row(2), "Cosserat Rod BVP Solution", "y (m)", "z (m)");
    //ContinuumRobotLibrary::plot(Y2.row(1), Y2.row(2), "Cosserat Rod BVP Solution", "y (m)", "z (m)");
    std::cout << "Solved rod1 centerline:\n" << Y1.block(0,0,3,Y1.cols()) << std::endl;
    std::cout << "Solved rod2 centerline:\n" << Y2.block(0,0,3,Y2.cols()) << std::endl;
    /*
    VectorXd x_(h1[0].size());
    VectorXd y_(h1[1].size());
    VectorXd z_(h1[2].size());

    for (int i = 0;i<h1[0].size();i++) {
      x_[i] = h1[0][i];
      y_[i] = h1[1][i];
      z_[i] = h1[2][i];
    }


    for (int i = h1[0].size();i< h1[0].size() + h2[0].size();i++) {
      x_[i] = h2[0][i];
      y_[i] = h2[1][i];
      z_[i] = h2[2][i];
    }

    ContinuumRobotLibrary::plot(x_, y_,   "Using Boost and Eigen", "x (m)", "y (m)");



    #ifdef QT_CORE_LIB
    //Include lines for the end-effector by connecting the rod ends
    for(auto i = 0; i < 5; i++){
        px[6+i] = Vector2d(px[i](N-1), px[i+1](N-1));
        pz[6+i] = Vector2d(pz[i](N-1), pz[i+1](N-1));
    }
    px[11] = Vector2d(px[5](N-1), px[0](N-1));
    pz[11] = Vector2d(pz[5](N-1), pz[0](N-1));
    //Build the color argument and show the solution
    const Vector3d b = Vector3d(0, 0, 255);
    const Vector3d r = Vector3d(255, 0, 0);
    std::vector<Vector3d> colors = {b,b,b,b,b,b, r,r,r,r,r,r};
    plot(px, pz, colors, "Continuum Stewart Gough BVP Solution", "x (m)", "z (m)");
    #endif

    for(int i = 0; i < 6; i++)
        std::cout << "Rod " << i << " centerline:\n" << p.block<3,N>(3*i,0) << '\n' << std::endl;
*/
    return 0;
}
