#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mm_ik");
  ros::NodeHandle nh("~");
  std::string chain_start = "mmrobot_link0";
  std::string chain_end = "mmrobot_link8";
  double timeout = 0.05;
  std::string urdf_param = "/mmrobot/robot_description";
  double eps = 1e-1;
  unsigned int maxiter = 100;
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  KDL::Chain chain;
  KDL::JntArray ll, ul; // Joint Limits
  bool valid = tracik_solver.getKDLChain(chain);
  valid = tracik_solver.getKDLLimits(ll, ul);
  for (int i = 0; i < ll.rows(); ++i) {
    std::cout << ll(i) << ", " << ul(i) << std::endl;
  }
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, maxiter, eps); // Joint Limit Solver

  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  KDL::JntArray q(chain.getNrOfJoints());

  std::cout << "configuration " << std::endl;
  for (uint j = 0; j < ll.data.size(); j++)
  {
    q(j) = fRand(ll(j), ul(j));
    std::cout << q(j) << ", ";
  }
  std::cout << std::endl;

  KDL::JntArray result(chain.getNrOfJoints());
  KDL::Frame end_effector_pose;
  int rc;
  fk_solver.JntToCart(q, end_effector_pose);
  std::cout << "End-effoctor-Position" << std::endl;
  for (int i = 0; i < 3; ++i) {
    std::cout << end_effector_pose.p.data[i] << ", ";
  }
  std::cout << std::endl;
  std::cout << "End effector Rotation" << std::endl;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      std::cout << end_effector_pose.M.data[i * 3 + j] << ", ";
    }
    std::cout << std::endl;
  }
  rc = kdl_solver.CartToJnt(nominal, end_effector_pose, result);
  std::cout << rc << std::endl;
  for (uint j = 0; j < ll.data.size(); j++)
  {
    std::cout << result(j) << ", ";
  }
  std::cout << std::endl;

  return 0;
}
