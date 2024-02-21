/**
 * 该例程将发布depth话题，消息类型String
 */
 
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "talk.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;
#include <math.h>

struct pose
{
  double position_x;
  double position_y;
  double position_z;

  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
};
pose orb_pose, tag_pose;






int main(int argc, char **argv)
{

  //下面是求orb变换矩阵的程序
  Eigen::Quaterniond q_orb(orb_pose.orientation_w,orb_pose.orientation_x,orb_pose.orientation_y,orb_pose.orientation_z);//orb的姿态，四元数表示
  // Eigen::Quaterniond q_orb(1,0,0,0);
  Eigen::Matrix3d R_orb = q_orb.toRotationMatrix();//将四元数转换为旋转矩阵
  Eigen::Vector3d t_orb (orb_pose.position_x,orb_pose.position_y,orb_pose.position_z);//orb的平移向量
  Eigen::MatrixXd T_orb_p(3,4);//先创建一个3*4的矩阵，里面只包括R和t，
  T_orb_p.block(0,0,3,3) = R_orb;//前3*3是旋转矩阵
  T_orb_p.col(3) = t_orb;//将orb的t赋值给T_orb的第四列
  // cout << T_orb_p << endl;
  Eigen::MatrixXd T_orb(4,4);//orb的矩阵变换
  T_orb.block(0,0,3,4) = T_orb_p;
  Eigen::RowVector4d r(0,0,0,1);//定义一个行向量
  T_orb.row(3) = r;
  cout << "orb变换矩阵 T_orb = " << endl << T_orb  << endl;















  // ROS节点初始化
  ros::init(argc, argv, "my_depth");
  
  // 创建节点句柄
  ros::NodeHandle nh;
  
  ros::Subscriber tag_Sub = nh.subscribe("/tag_Odometry", 10, &tag_callback);//订阅AprilTag的话题
  ros::Subscriber orb_Sub = nh.subscribe("/posestamped", 10, &orb_callback);//订阅AprilTag的话题


  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ros::Publisher depth_pub = nh.advertise<std_msgs::String>("depth", 1000);

  // 设置循环的频率
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
	// 初始化std_msgs::String类型的消息
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();



	// 发布消息
    ROS_INFO("%s", msg.data.c_str());
    depth_pub.publish(msg);

	// 循环等待回调函数
    ros::spinOnce();
	
	// 按照循环频率延时
    loop_rate.sleep();
    ++count;
  }

  return 0;
}


void tag_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // flight_status = msg->data;
  tag_pose.position_x = msg -> pose.pose.position.x;
  tag_pose.position_y = msg -> pose.pose.position.y;
  tag_pose.position_z = msg -> pose.pose.position.z;
 
  tag_pose.orientation_x = msg -> pose.pose.orientation.x;
  tag_pose.orientation_y = msg -> pose.pose.orientation.y;
  tag_pose.orientation_z = msg -> pose.pose.orientation.z;
  tag_pose.orientation_w = msg -> pose.pose.orientation.w;
}

void orb_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // flight_status = msg->data;
  orb_pose.position_x = msg -> pose.position.x;
  orb_pose.position_y = msg -> pose.position.y;
  orb_pose.position_z = msg -> pose.position.z;
 
  orb_pose.orientation_x = msg -> pose.orientation.x;
  orb_pose.orientation_y = msg -> pose.orientation.y;
  orb_pose.orientation_z = msg -> pose.orientation.z;
  orb_pose.orientation_w = msg -> pose.orientation.w;

}












