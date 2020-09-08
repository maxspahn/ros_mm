#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>


class OdomPub
{
private:
  sensor_msgs::JointState jointStates_;
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber jointState_pub_;
  tf::TransformBroadcaster odom_broadcaster_;
  ros::Rate rate_;

  double x_, y_, theta_, w1_, w2_, wheel_radius_, wheel_seperation_;

public:
  OdomPub(double, double);
  void jointStates_cb(const sensor_msgs::JointState::ConstPtr&);
  void loop();
  void integrateRungeKutta2(double, double, double);
  void publishOdom(ros::Time, ros::Time);
};


OdomPub::OdomPub(double wheel_sep, double wheel_radius) : rate_(10.0), wheel_seperation_(wheel_sep), wheel_radius_(wheel_radius)
{
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
  ROS_INFO("INIT odom pub");
  rate_.sleep();
  jointState_pub_ = nh_.subscribe<sensor_msgs::JointState>("/boxer/joint_states", 10, &OdomPub::jointStates_cb, this);
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  w1_ = 0.0;
  w2_ = 0.0;
  ros::spinOnce();
  ROS_INFO("INIT odom pub");
}

void OdomPub::jointStates_cb(const sensor_msgs::JointState::ConstPtr& data)
{
  w1_ = data->velocity[1];
  w2_ = data->velocity[2];
  // Filtering simple way
  if (std::abs(w1_ - 0.0001) < 0.001) w1_ = 0.0;
  if (std::abs(w2_ - 0.0001) < 0.001) w2_ = 0.0;
  ROS_INFO("Getting joint states %1.2f %1.2f", w1_, w2_);
}

void OdomPub::integrateRungeKutta2(double linear, double angular, double dt)
{
  const double direction = theta_ + angular * 0.5;

  // Runge-Kutta 2nd order integration:
  x_       += linear * cos(theta_) * dt;
  y_       += linear * sin(theta_) * dt;
  theta_   += angular * dt;
              
}

void OdomPub::loop()
{
  ros::Time last_time, current_time;
  last_time = ros::Time::now();
  while(nh_.ok()){
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    /*
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    */
    double linear  = (w1_ + w2_) * 0.5 * wheel_radius_;
    double angular = (-w1_ + w2_) * wheel_radius_ / wheel_seperation_;
    ROS_INFO("Linear Velocity : %1.2f", linear);
    ROS_INFO("Angular Velocity : %1.2f", angular);

    integrateRungeKutta2(linear, angular, dt);
    publishOdom(current_time, last_time);
    last_time = current_time;
    rate_.sleep();
    ros::spinOnce();
  }
}

void OdomPub::publishOdom(ros::Time current_time, ros::Time last_time)
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster_.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  //publish the message
  odom_pub_.publish(odom);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  OdomPub op = OdomPub(0.480, 0.08);
  op.loop();
}
