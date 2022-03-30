#include "ros/ros.h"
#include "forward_kinematics/joint_variables.h"
#include "cmath"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Callback(const forward_kinematics::joint_variables::ConstPtr& msg)
{
  float q1,q2,q3,L1,L2,L3,r11,r12,r13,dx,r21,r22,r23,dy,r31,r32,r33,dz;
  q1 = msg->q1;
  q1 = (q1*M_PI)/180;

  q2 = msg->q2;
  q2 = (q2*M_PI)/180;

  q3 = msg->q3;
  q3 = (q3*M_PI)/180;

  L1 = 1.0;
  L2 = 1.0;
  L3 = 1.0;

  r11 = (cos(q1)*cos(q2)*cos(q3)) - (cos(q1)*sin(q2)*sin(q3));
  r12 = -(cos(q1)*cos(q2)*sin(q3)) - (cos(q1)*sin(q2)*sin(q3));
  r13 = -sin(q1);
  dx = (L3*cos(q1)*cos(q2)*cos(q3)) - (L3*cos(q1)*sin(q2)*sin(q3)) + (L2*cos(q1)*cos(q2));

  r21 = (sin(q1)*cos(q2)*cos(q3)) - (sin(q1)*sin(q2)*sin(q3));
  r22 = -(sin(q1)*cos(q2)*sin(q3)) - (sin(q1)*sin(q2)*cos(q3));
  r23 = cos(q1);
  dy = (L3*sin(q1)*cos(q2)*cos(q3)) - (L3*sin(q1)*sin(q2)*sin(q3)) + (L2*sin(q1)*cos(q2));

  r31 = -(sin(q2)*cos(q3)) - (cos(q2)*sin(q3));
  r32 = -(sin(q2)*sin(q3)) - (cos(q2)*cos(q3));
  r33 = 0;
  dz = -(L3*sin(q2)*cos(q3)) - (L3*cos(q2)*sin(q3)) + (L2*sin(q2)) + L1;
  ROS_INFO("[[%f] [%f] [%f] [%f]]", r11, r12, r13, dx);
  ROS_INFO("[[%f] [%f] [%f] [%f]]", r21, r22, r23, dy);
  ROS_INFO("[[%f] [%f] [%f] [%f]]", r31, r32, r33, dz);
  ROS_INFO("[[%f] [%f] [%f] [%f]]", 0.0, 0.0, 0.0, 1.0);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "forward_kinematics");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("FK_topic", 1000, Callback);

  ros::spin();

  return 0;
}