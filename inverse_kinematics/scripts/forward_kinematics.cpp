#include "ros/ros.h"
#include "forward_kinematics/joint_variables.h"
#include "geometry_msgs/Pose.h"
#include "cmath"

void Callback_fk(const forward_kinematics::joint_variables::ConstPtr& msg)
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
  
  ROS_INFO("---------------------------------------");
  ROS_INFO("----------FORWARD KINEMATICS-----------");
  ROS_INFO("---------------------------------------");
  ROS_INFO("---HOMOGENOUS TRANSFORMATION MATRIX----");
  ROS_INFO("[[%f] [%f] [%f] [%f]]", r11, r12, r13, dx);
  ROS_INFO("[[%f] [%f] [%f] [%f]]", r21, r22, r23, dy);
  ROS_INFO("[[%f] [%f] [%f] [%f]]", r31, r32, r33, dz);
  ROS_INFO("[[%f] [%f] [%f] [%f]]", 0.0, 0.0, 0.0, 1.0);
  ROS_INFO("---------------POSITION----------------");
  ROS_INFO("X:[%f] Y:[%f] Z:[%f]", dx, dy, dz);
}

void Callback_ik(const geometry_msgs::Pose::ConstPtr& msg)
{
  float L1 = 1.0, L2 = 1.0, L3 = 1.0;
  
  float X,Y,Z;
  X = msg -> position.x;
  Y = msg -> position.y;
  Z = msg -> position.z;

  float Q1,Q2,Q3,S,R,D;
  S = Z - L1;
  R = sqrt(pow(X,2) + pow(Y,2));
  D = ((pow(R,2)) + (pow(S,2)) - (pow(L2,2)) - (pow(L3,2)))/(2*L2*L3);

  Q1 = atan2(Y,X);
  Q3 = atan2(sqrt(1-pow(D,2)),D);
  Q2 = atan2(S,R) - atan2((L3*sin(Q3)), (L2 + L3*cos(Q3)));

  
  Q1 = (Q1*180)/M_PI;
  
  Q2 = (Q2*180)/M_PI;

  Q3 = (Q3*180)/M_PI;

  ROS_INFO("---------------------------------------");
  ROS_INFO("----------INVERSE KINEMATICS-----------");
  ROS_INFO("---------------------------------------");
  ROS_INFO("------------JOINT VARIABLES------------");
  ROS_INFO("q1:[%f] q2:[%f] q3:[%f]", Q1, Q2, Q3);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "forward_kinematics");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("FK_topic", 1000, Callback_fk);
  ros::Subscriber sub1 = n.subscribe("IK_topic", 1000, Callback_ik);
  

  ros::spin();

  return 0;
}