#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <cmath>

void calPose(const ros::TimerEvent& event);
void calOdom(const ros::TimerEvent& event);

ros::Publisher _pubPose;
ros::Publisher _pubOdom;
ros::Publisher _pubPoseRviz;

nav_msgs::Odometry _odomOld;
bool _odomInit = false;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_test");
  ros::NodeHandle n;

  _pubPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1000);
  _pubPoseRviz = n.advertise<geometry_msgs::PoseStamped>("poseRviz", 1000);
  _pubOdom = n.advertise<nav_msgs::Odometry>("odom", 1000);
  ros::Timer timPose = n.createTimer(ros::Duration(0.5), calPose);
  ros::Timer timOdom = n.createTimer(ros::Duration(0.05), calOdom);

  ros::spin();
}

void calOdom(const ros::TimerEvent& event)
{
  ros::Time now;
  now = ros::Time::now();
  double dNow = (double)now.sec + ((double)now.nsec) * 1e-9;
  nav_msgs::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = std::sin(dNow) + 3.0 * std::sin(0.1 * dNow);
  odom.pose.pose.position.y = std::sin(dNow) + 3.0 * std::sin(0.1 * dNow);
  odom.pose.pose.position.z = 0.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), odom.pose.pose.orientation);
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  odom.twist.twist.linear.x = std::cos(dNow) + 0.1 * std::cos(0.1 * dNow);
  odom.twist.twist.linear.y = std::cos(dNow) + 0.1 * std::cos(0.1 * dNow);
  odom.twist.twist.linear.z = 0;

  double odomVar = 1e-10;

  odom.twist.covariance.elems[0] = odomVar;
  odom.twist.covariance.elems[7] = odomVar;
  odom.twist.covariance.elems[14] = odomVar;
  odom.twist.covariance.elems[21] = odomVar;
  odom.twist.covariance.elems[28] = odomVar;
  odom.twist.covariance.elems[35] = odomVar;

  odomVar = 1e-10;

  odom.pose.covariance.elems[0] = odomVar;
  odom.pose.covariance.elems[7] = odomVar;
  odom.pose.covariance.elems[14] = odomVar;
  odom.pose.covariance.elems[21] = odomVar;
  odom.pose.covariance.elems[28] = odomVar;
  odom.pose.covariance.elems[35] = odomVar;

  _pubOdom.publish(odom);
}

void calPose(const ros::TimerEvent& event)
{
  ros::Time now;
  now = ros::Time::now();
  double dNow = (double)now.sec + ((double)now.nsec) * 1e-9;
  geometry_msgs::PoseWithCovarianceStamped pose;
  geometry_msgs::PoseStamped poseRviz;
  pose.header.stamp = now;
  pose.header.frame_id = "map";
  pose.pose.pose.position.x = std::sin(dNow);
  pose.pose.pose.position.y = std::sin(dNow);
  pose.pose.pose.position.z = 0.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), pose.pose.pose.orientation);

  double poseVar = 1e-100;
  pose.pose.covariance.elems[0] = poseVar;
  pose.pose.covariance.elems[7] = poseVar;
  pose.pose.covariance.elems[14] = poseVar;
  pose.pose.covariance.elems[21] = poseVar;
  pose.pose.covariance.elems[28] = poseVar;
  pose.pose.covariance.elems[35] = poseVar;

  poseRviz.header.frame_id = "map";
  poseRviz.pose.orientation = pose.pose.pose.orientation;
  poseRviz.pose.position = pose.pose.pose.position;

  _pubPoseRviz.publish(poseRviz);
  _pubPose.publish(pose);
}
