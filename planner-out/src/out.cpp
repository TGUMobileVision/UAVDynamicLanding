#include<ros/ros.h>

#include<std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>


#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>


using namespace std;
//接收到消息后进入回调函数

void traj_cur_callback(const nav_msgs::Odometry::ConstPtr &msg);
void traj_planning_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
void drone_cmd_callback(const  quadrotor_msgs::SO3Command::ConstPtr &msg);


struct current_position_def
{
  double x;
  double y;
  double z;
  double w;

  double orb_x;
  double orb_y;  
  double orb_z;
  double x_fu;
  double y_fu;
};

current_position_def current_position, planning_vel,planning_position,controller_force_,controller_orientation_;


int main(int argc,char **argv)

{
//开始保存各种信息
  string traj_cur = "traj_cur.txt";//当前位置
  ofstream f_t_s;
  f_t_s.open(traj_cur.c_str()); // delete previos data from last run.
  f_t_s.close();

  string traj_planning = "traj_planning.txt";//计划轨迹位置
  ofstream f_t_p;
  f_t_p.open(traj_planning.c_str()); // delete previos data from last run.
  f_t_p.close();

  string velocity_planning = "velocity_planning.txt";//计划轨迹速度
  ofstream f_v_p;
  f_v_p.open(velocity_planning.c_str()); // delete previos data from last run.
  f_v_p.close();

  string controller_force = "controller_force.txt";//控制器输出的推力
  ofstream f_c_f;
  f_c_f.open(controller_force.c_str()); // delete previos data from last run.
  f_c_f.close();

  string controller_orientation = "controller_orientation.txt";//控制器输出的oritation
  ofstream f_c_o;
  f_c_o.open(controller_orientation.c_str()); // delete previos data from last run.
  f_c_o.close();


  ros::init(argc,argv,"listener");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/state_ukf/odom",1000,traj_cur_callback);        //odom  topic 
  ros::Subscriber sub2 = n.subscribe("/planning/pos_cmd",1000,traj_planning_callback); //traj_planning  topic 
  ros::Subscriber sub3 = n.subscribe("/so3_cmd",1000,drone_cmd_callback); //drone cmd topic 

  ros::spin();//节点进入循环状态，有消息到达时调用回调函数完成处理
  return 0;

}



//保存订阅的odom话题信息
void traj_cur_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
 // flight_status = msg->data;
  current_position.x = msg -> pose.pose.position.x;
  current_position.y = msg -> pose.pose.position.y;
  current_position.z = msg -> pose.pose.position.z;
  cout<<"保存当前postion"<<endl;
  ros::Time currentTime1 = ros::Time::now(); 
  string traj_cur = "traj_cur.txt";//当前位姿
  ofstream f_t_s;
  f_t_s.open(traj_cur.c_str(),ios::app);
  f_t_s << fixed;
  f_t_s << currentTime1 << setprecision(9) <<" "<< current_position.x  << " " << current_position.y << " " << current_position.z <<endl;
  f_t_s.close();

}

//保存订阅的期望planning位置话题信息以及速度信息
void traj_planning_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  planning_position.x = msg -> position.x;
  planning_position.y = msg -> position.y;
  planning_position.z = msg -> position.z;
  cout<<"保存期望planning postion"<<endl;
  ros::Time currentTime2 = ros::Time::now(); 
  string traj_planning = "traj_planning.txt";//当前位姿
  ofstream f_t_p;
  f_t_p.open(traj_planning.c_str(),ios::app);
  f_t_p << fixed;
  f_t_p << currentTime2 << setprecision(9) <<" "<< planning_position.x  << " " << planning_position.y << " " << planning_position.z <<endl;
  f_t_p.close();


  planning_vel.x = msg -> velocity.x;
  planning_vel.y = msg -> velocity.y;
  planning_vel.z = msg -> velocity.z;
  cout<<"保存期望planning velocity"<<endl;
  ros::Time currentTime3 = ros::Time::now(); 
  string velocity_planning = "velocity_planning.txt";//当前位姿
  ofstream f_v_p;
  f_v_p.open(velocity_planning.c_str(),ios::app);
  f_v_p << fixed;
  f_v_p << currentTime3 << setprecision(9) <<" "<< planning_vel.x  << " " << planning_vel.y << " " << planning_vel.z <<endl;
  f_v_p.close();

}

void drone_cmd_callback(const  quadrotor_msgs::SO3Command::ConstPtr &msg)
{

  controller_force_.x = msg -> force.x;
  controller_force_.y = msg -> force.y;
  controller_force_.z = msg -> force.z;
  cout<<"保存控制器推力f"<<endl;
  ros::Time currentTime4 = ros::Time::now(); 
  string controller_force = "controller_force.txt";//当前推力
  ofstream f_c_f;
  f_c_f.open(controller_force.c_str(),ios::app);
  f_c_f << fixed;
  f_c_f << currentTime4 << setprecision(9) <<" "<< controller_force_.x  << " " << controller_force_.y << " " << controller_force_.z <<endl;
  f_c_f.close();


  controller_orientation_.x = msg -> orientation.x;
  controller_orientation_.y = msg -> orientation.y;
  controller_orientation_.z = msg -> orientation.z;
  controller_orientation_.w = msg -> orientation.w;
  //控制器的姿态四元数
  Eigen::Quaterniond q_controller(controller_orientation_.w,controller_orientation_.x,controller_orientation_.y,controller_orientation_.z);
  //四元数转欧拉角(Z-Y-X，即RPY)
  Eigen::Vector3d eulerAngle = q_controller.matrix().eulerAngles(2,1,0);
  /*
  cout << "roll = "  << eulerAngle(2) << endl;
  cout << "pitch = " << eulerAngle(1) << endl;
  cout << "yaw = "   << eulerAngle(0) << endl;
  */
  
  cout<<"保存控制器推力姿态"<<endl;
  ros::Time currentTime5 = ros::Time::now(); 
  string controller_orientation = "controller_orientation.txt";//当前位姿
  ofstream f_c_o;
  f_c_o.open(controller_orientation.c_str(),ios::app);
  f_c_o << fixed;
  f_c_o << currentTime5 << setprecision(9) <<" "<< eulerAngle(2)  << " " << eulerAngle(1) << " " << eulerAngle(0) <<endl;
  f_c_o.close();


}