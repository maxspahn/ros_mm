#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Path.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();
    start->as<ob::SE3StateSpace::StateType>()->setX(0.0);
    start->as<ob::SE3StateSpace::StateType>()->setY(0.0);


    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE3StateSpace::StateType>()->setX(0.0);
    goal->as<ob::SE3StateSpace::StateType>()->setY(4.0);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

og::PathGeometric planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        return ss.getSolutionPath();
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return ss.getSolutionPath();
    }
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendSimpleGoal(MoveBaseClient *actionClient, double x, double y, double w) {
  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  ROS_INFO("Sending goal");
  actionClient->sendGoal(goal);

  actionClient->waitForResult();

  if(actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The goal was reached");
  else
    ROS_INFO("The goal was not reached");

  
}

void sendSimpleGoal(MoveBaseClient *actionClient, ob::SE3StateSpace::StateType* s0) {
  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  printf("X : %1.4f, Y : %1.4f\n", s0->getX(), s0->getY());
  goal.target_pose.pose.position.x = s0->getX();
  goal.target_pose.pose.position.y = s0->getY();
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  actionClient->sendGoal(goal);

  actionClient->waitForResult();

  if(actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The goal was reached");
  else
    ROS_INFO("The goal was not reached");

  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  plan();

  std::cout << std::endl << std::endl;

  og::PathGeometric p1 = planWithSimpleSetup();
  p1.interpolate(10);
  p1.printAsMatrix(std::cout);
  ob::SE3StateSpace::StateType* s0;
  std::size_t nbStates = p1.getStateCount();
  for (int i = 0; i < nbStates; ++i) {
    s0 = p1.getState(i)->as<ob::SE3StateSpace::StateType>();
    sendSimpleGoal(&ac, s0);
  }

  sendSimpleGoal(&ac, 1.0, 5.0, 1.0);
  sendSimpleGoal(&ac, 2.0, 5.0, 1.0);

  return 0;
}
