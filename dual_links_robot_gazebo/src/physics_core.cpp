#include <dual_links_robot_gazebo/physics_core.h>

#include <dual_links_robot_gazebo/commonmath.h>
#include <dual_links_robot_gazebo/convexoptimization.h>
#include <dual_links_robot_gazebo/numericalintegration.h>



using namespace ContinuumRobotLibrary;

using namespace Eigen;

//const double pi = 3.1415926535897932384626433832795028841971L;

//Independent Parameters
const double E = 200e9;
const double G = 80e9;
const double rad = 0.001;
const double rho = 8000;
const Vector3d g = 9.81*Vector3d::Zero();
const double L1 = 1.0;
const double L2 = 1.0;

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
static Vector3d p10(0.0, 0.0, 0.0);
static Vector3d p20(0.0, 0.0, 0.0);
static Vector3d pE(0.0, 0.0, 0.0); // = Vector3d(0, -0.1*L, 0.8*L);
const Vector3d F(0.0, 0.0, 0.0);// = Vector3d::Zero();
static MatrixXd Y1;
static MatrixXd Y2;
static double th1;
static double th2;
//static VectorXd guess_;
VectorXd shootingFunction(VectorXd guess){
    //guess_ = guess;
    double roll = -M_PI/2;
    Matrix3d Rx;
    Rx  <<  1,     0,         0,
            0,   cos(roll), -sin(roll),
            0,   sin(roll),  cos(roll);

    //  Unpack the guess vector
    th1 = guess(0);
    Matrix3d Rz1;
    Rz1 <<  cos(th1), -sin(th1),  0,
            sin(th1),  cos(th1),  0,
               0,         0,      1;
    Matrix3d R1 =Rz1*Rx;
    Vector3d n1;
    n1 << guess(1), guess(2), guess(3);
    Vector3d m1;
    m1 << guess(4), guess(5), guess(6);
    //  Compose state for the cosserat rod model
    VectorXd y1(18);
    y1 << p10, Map<VectorXd>(Matrix3d(R1).data(), 9), n1, m1;

    //  Unpack the guess vector
    th2 = guess(7);
    Matrix3d Rz2;
    Rz2 <<  cos(th2), -sin(th2),  0,
            sin(th2),  cos(th2),  0,
               0,         0,      1;
    Matrix3d R2 =Rz2*Rx;
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
    residual << distal_error, check_closure, res_force, mL1.z(), mL2.z();


    return residual;
}



const Vector3d n10 = Vector3d::Zero();
const Vector3d m10 = Vector3d::Zero();
const Vector3d n20 = Vector3d::Zero();
const Vector3d m20 = Vector3d::Zero();
void RodIVPPhysics::initParams(double dt, double fxy, double fz, double mz, double damping, double length, const State &link1_state, const State& link2_state)
{
  std::cout << "SpringPhysics::initParams" << std::endl;
  this->fxy = fxy;
  this->dt = dt;
  this->fz = fz;
  this->mz = mz;
  this->damping = damping;
  this->length = length;

  p10 = link1_state.pose.translation();
  p10.z() = 0;
  p20 = link2_state.pose.translation();
  p20.z() = 0;

}

Wrench RodIVPPhysics::update(const State &distal_plate_state)
{

  //Set initial conditions
  pE = distal_plate_state.pose.translation();
  pE.z() = 0.0;

  double th10 = 0.0;
  double th20 = 0.0;


  VectorXd init_guess(14);
  init_guess << th10, n10, m10, th20, n20, m20;

//  std::cout << "p10 : " << p10.transpose() << std::endl;
//  std::cout << "p20 : " << p20.transpose() << std::endl;
//  std::cout << "pE : " << pE.transpose() << std::endl;

  //Solve with shooting method
  solveLevenbergMarquardt<shootingFunction>(init_guess);

  for (unsigned int i=0;i<100;i++) {
    shape1.x[i] = Y1(0,i) - p10.x();
    shape1.y[i] = Y1(1,i) - p10.y();
    shape1.z[i] = Y1(2,i) - p10.z();
  }
  shape_rod1.publish(shape1);

  for (unsigned int i=0;i<100;i++) {
    shape2.x[i] = Y2(0,i) - p20.x();
    shape2.y[i] = Y2(1,i) - p20.y();
    shape2.z[i] = Y2(2,i) - p20.z();
  }
  shape_rod2.publish(shape2);

  return Wrench();

}


}
