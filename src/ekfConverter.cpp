#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

void calOdom(const nav_msgs::Odometry odom);
void calPose(const geometry_msgs::PoseStamped msg);

static int _iPose = 0;

ros::Publisher pubOdom;
ros::Publisher pubPose;
ros::Publisher pubPoseRviz;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekfConverter");
  ros::NodeHandle n;

  ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("wheelodom", 1000, calOdom);
  ros::Subscriber subPose = n.subscribe<geometry_msgs::PoseStamped>("pose", 1000, calPose);

  pubOdom = n.advertise<nav_msgs::Odometry>("ekfOdom", 1);
  pubPoseRviz = n.advertise<geometry_msgs::PoseStamped>("ekfPoseRviz", 1);
  pubPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekfPose", 1);

  ros::spin();

}

void calPose(const geometry_msgs::PoseStamped msgConst)  //message subscriber
{
  if (_iPose < 10)
  {
    _iPose++;
    return;
  }

  _iPose = 0;

  geometry_msgs::PoseStamped msg = msgConst;
  //Transform Message
  tf::Transform F_L;
  tf::Transform T_LR;
  tf::Transform F_B;

  tf::Quaternion tmp;
  F_L.setIdentity();
  F_L.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0));
  tf::quaternionMsgToTF(msg.pose.orientation, tmp);
  F_L.setRotation(tmp);

  T_LR.setIdentity();
  T_LR.setOrigin(tf::Vector3(-0.61, 0.0, 0.0));  // Verschiebung Laser Scanner base_footprint

  F_B = F_L * T_LR;

  msg.pose.orientation.w = F_B.getRotation().w();
  msg.pose.orientation.x = F_B.getRotation().x();
  msg.pose.orientation.y = F_B.getRotation().y();
  msg.pose.orientation.z = F_B.getRotation().z();
  msg.pose.position.x = F_B.getOrigin().getX();
  msg.pose.position.y = F_B.getOrigin().getY();
  msg.pose.position.z = 0.0;
  msg.header.frame_id = "ekfMap";

  //Publish msg for rviz
  pubPoseRviz.publish(msg);

  //Publish msg for ekf
  geometry_msgs::PoseWithCovarianceStamped msg_c;

  msg_c.header = msg.header;
  msg_c.pose.pose = msg.pose;

  double covPose = 50;

  msg_c.pose.covariance.elems[0] = covPose;
  msg_c.pose.covariance.elems[7] = covPose;
  msg_c.pose.covariance.elems[14] = covPose;
  msg_c.pose.covariance.elems[21] = covPose;
  msg_c.pose.covariance.elems[28] = covPose;
  msg_c.pose.covariance.elems[35] = covPose;
  //double yaw = tf::getYaw(msg_c.pose.pose.orientation);
  //tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-yaw), msg_c.pose.pose.orientation);
  pubPose.publish(msg_c);

  return;
}

void calOdom(const nav_msgs::Odometry odomConst)
{
  nav_msgs::Odometry odom = odomConst;

  odom.header.frame_id = "ekfOdom";
  odom.child_frame_id = "ekfBaseFootprint";

  double CovOdomTwist = 1e-10;
  double CovOdomPose = 1e-10;

  odom.pose.covariance.elems[0] = CovOdomPose;
  odom.pose.covariance.elems[7] = CovOdomPose;
  odom.pose.covariance.elems[14] = CovOdomPose;
  odom.pose.covariance.elems[21] = CovOdomPose;
  odom.pose.covariance.elems[28] = CovOdomPose;
  odom.pose.covariance.elems[35] = CovOdomPose;
  odom.twist.covariance.elems[0] = CovOdomTwist;
  odom.twist.covariance.elems[7] = CovOdomTwist;
  odom.twist.covariance.elems[14] = CovOdomTwist;
  odom.twist.covariance.elems[21] = CovOdomTwist;
  odom.twist.covariance.elems[28] = CovOdomTwist;
  odom.twist.covariance.elems[35] = CovOdomTwist;
  pubOdom.publish(odom);

  //old odom tf
  //create tf broadcaster
  static tf::TransformBroadcaster odom_broadcaster;
  //publish transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header = odom.header;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "baseFootprint"; // Set child_frame_id of message to be the "base_link" frame
  odom_trans.transform.translation.x = odom.pose.pose.position.x; //Übergibt die Odomdaten an odom_trans, welches über tf rausgeschickt wird.
  odom_trans.transform.translation.y = odom.pose.pose.position.y;
  odom_trans.transform.translation.z = odom.pose.pose.position.z;
  odom_trans.transform.rotation = odom.pose.pose.orientation;
  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
}
