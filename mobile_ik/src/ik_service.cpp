#include "ros/ros.h"
#include "mm_msgs/MMIk.h"
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>

#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"

bool computeIK(mm_msgs::MMIk::Request  &req, mm_msgs::MMIk::Response &res)
{
  tf::TransformListener *tfListenerPtr = new tf::TransformListener();
  geometry_msgs::PoseStamped posePanda;
  tfListenerPtr->waitForTransform("mmrobot_link0", req.pose.header.frame_id, ros::Time::now(), ros::Duration(1.0));
  tfListenerPtr->transformPose("mmrobot_link0", req.pose, posePanda); 
  std::string chain_start = "mmrobot_link0";
  std::string chain_end = "mmrobot_link8";
  double timeout = 0.5;
  unsigned int maxiters = 1000;
  std::string urdf_param = "/mmrobot/robot_description";
  double eps = 1e-5;
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  KDL::Chain chain;
  KDL::JntArray ll, ul; // Joint Limits
  bool valid = tracik_solver.getKDLChain(chain);
  valid = tracik_solver.getKDLLimits(ll, ul);
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j = 0; j < ll.data.size(); j++)
  {
    ll(j) *= 0.8;
    ul(j) *= 0.8;
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, maxiters, eps); // Joint Limit Solver


  KDL::JntArray result(chain.getNrOfJoints());
  KDL::Frame target_pose;


  target_pose.M = KDL::Rotation::Quaternion(posePanda.pose.orientation.x,
                                            posePanda.pose.orientation.y,
                                            posePanda.pose.orientation.z,
                                            posePanda.pose.orientation.w);
  target_pose.p(0) = posePanda.pose.position.x;
  target_pose.p(1) = posePanda.pose.position.y;
  target_pose.p(2) = posePanda.pose.position.z;
  int rc = kdl_solver.CartToJnt(nominal, target_pose, result);
  
  res.errorFlag = rc;
  for (int i = 0; i < result.data.size(); ++i) {
    res.config.data.push_back(result(i));
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_computation");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("ik_computation", computeIK);
  ROS_INFO("Ready to computeIK.");
  ros::spin();

  return 0;
}
