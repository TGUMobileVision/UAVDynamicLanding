/**

 */
 
#include <sstream>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include "std_msgs/String.h"
#include "talk.h"
#include "orb.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <Eigen/Dense>

#include <iostream>
using namespace Eigen;
using namespace std;


void vins_callback(const nav_msgs::Odometry::ConstPtr &msg);
void fast_planner_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
void fast_planner_camera_callback(const nav_msgs::Odometry::ConstPtr &msg);

// geometry_msgs::Quaternion q_orb;
// Eigen::Quaterniond q_orb;

struct pose
{
  double position_x;
  double position_y;
  double position_z;

  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;

  double linear_x;
  double linear_y;
  double linear_z;

  double angular_x;
  double angular_y;
  double angular_z;
};
pose orb_pose, tag_pose,vins_pose,drone_pose,fast_planner_odom,fast_planner_camera;

int flag = 0;
int flag_c = 0;
int flag_o = 0;
int counter = 0;


   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_pose");
    ros::NodeHandle nh;
    ros::Subscriber orb_Sub = nh.subscribe("/posestamped", 1, &orb_callback);//订阅orb的话题
    ros::Subscriber tag_Sub = nh.subscribe("/tag_Odometry", 1, &tag_callback);//订阅tag的话题
    ros::Subscriber vins_Sub = nh.subscribe("/vins_estimator/camera_pose", 1, &vins_callback);//订阅vins-mono的话题

    ros::Subscriber vins_odom_Sub = nh.subscribe("/vins_estimator/odometry", 1, &fast_planner_odom_callback);
    ros::Subscriber vins_camera_Sub = nh.subscribe("/vins_estimator/camera_pose", 1, &fast_planner_camera_callback);

    // 创建一个Publisher，发布名为pose_by_orb的topic，消息类型为nav_msgs::Path
    //ros::Publisher pose_by_orb_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_by_orb", 1,true);
    //ros::Publisher pose_by_tag_pub = nh.advertise<nav_msgs::Odometry>("pose_by_tag", 1,true);
    ros::Publisher pose_by_vins_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_by_vins", 1,true);
    ros::Publisher trans_vins_odom = nh.advertise<nav_msgs::Odometry>("trans_vins_odom", 1,true);
    ros::Publisher trans_vins_camera = nh.advertise<geometry_msgs::PoseStamped>("trans_vins_camera", 1,true);


    // 设置循环的频率
    ros::Rate loop_rate(50);

    double count = 0.0;
    while (ros::ok())
    {

      if (flag == 0)
      {
        cout << "waiting......"<<endl;
      }
      if(flag ==1)
      {
        //以下是计算无人机位姿的程序

        //下面是求orb变换矩阵的程序
        //Eigen::Quaterniond q_orb(orb_pose.orientation_w,orb_pose.orientation_x,orb_pose.orientation_y,orb_pose.orientation_z);//orb的姿态，四元数表示
        Eigen::Quaterniond q_vins(vins_pose.orientation_w,vins_pose.orientation_x,vins_pose.orientation_y,vins_pose.orientation_z);//vins的姿态，四元数表示
        // Eigen::Quaterniond q_vins(1,0,0,0);
        Eigen::Matrix3d R_vins = q_vins.toRotationMatrix();//将四元数转换为旋转矩阵
        Eigen::Vector3d t_vins (vins_pose.position_x,vins_pose.position_y,vins_pose.position_z);//vins的平移向量
        Eigen::MatrixXd T_vins_p(3,4);//先创建一个3*4的矩阵，里面只包括R和t，
        T_vins_p.block(0,0,3,3) = R_vins;//前3*3是旋转矩阵
        T_vins_p.col(3) = t_vins;//将vins的t赋值给T_vins的第四列
        // cout << T_vins_p << endl;
        Eigen::MatrixXd T_vins(4,4);//vins的矩阵变换
        T_vins.block(0,0,3,4) = T_vins_p;
        Eigen::RowVector4d r(0,0,0,1);//定义一个行向量
        T_vins.row(3) = r;
        Eigen::MatrixXd T_w_c1 = T_vins;
        cout << "vins变换矩阵 T_w_c1 = " << endl << T_w_c1  << endl;

        //将相机坐标系在世界坐标系下的表示转换到相机坐标系在相机坐标系下的表示
        Eigen::MatrixXd T_c0_w(4,4);
        T_c0_w << 1,0,0,0,
                  0,0,-1,0,
                  0,1,0,0,
                  0,0,0,1;
        Eigen::MatrixXd T_c0_c1(4,4);
        T_c0_c1 = T_c0_w * T_w_c1;
        cout << "T_c0_c1 = "<< endl << T_c0_c1 << endl;



        //下面是求相机到无人机机体的变化矩阵的过程
        //过程是：知道机体到相机的旋转矩阵和平移向量，这就知道机体到相机的T1,so相机到机体的T2等于T1的逆矩阵
        Eigen::Matrix3d R_CtoB;//机体到相机的旋转矩阵
        R_CtoB << 0,   0,   1,
                 -1,   0,   0,
                  0,   -1,   0;
        Eigen::Vector3d t_CtoB(0.145,0.05,0.04);//机体到相机的平移向量  ??????????????????????????????????
        Eigen::MatrixXd T_CtoB_p(3,4);//先创建一个3*4的矩阵，里面只包括R和t，
        T_CtoB_p.block(0,0,3,3) = R_CtoB;//前3*3是旋转矩阵
        T_CtoB_p.col(3) = t_CtoB;//将机体到相机的t赋值给T_BtoC的第四列
        Eigen::MatrixXd T_CtoB(4,4);//机体到相机的矩阵变换
        T_CtoB.block(0,0,3,4) = T_CtoB_p;
        T_CtoB.row(3) = r;
        // cout << "机体到相机的变换矩阵T_BtoC为"<< endl <<  T_BtoC << endl;
        Eigen::Matrix4d T_BtoC;//相机到机体的旋转矩阵
        T_BtoC = T_CtoB.inverse();
        // cout << "相机到机体的变换矩阵T_CtoB为"<< endl <<  T_CtoB << endl;

        
        //下面是求无人机位姿的过程
        Eigen::Matrix4d T_drone;
        T_drone = T_CtoB * T_c0_c1 * T_BtoC;//通过vins计算无人机的变换矩阵
        cout << "无人机的变换矩阵T_drone为" << endl << T_drone <<endl;

        Eigen::MatrixXd T_drone_p(3,4);//定义一个3*4的矩阵，里面只包括R和T
        T_drone_p = T_drone.block(0,0,3,4);
        Eigen::Matrix3d R_drone;//无人机的旋转矩阵，即姿态
        R_drone = T_drone_p.block(0,0,3,3);
        cout << " 无人机的旋转矩阵为 " << endl << R_drone << endl;
        Eigen::MatrixXd t_drone(3,1);//无人机的平移向量，即位置
        t_drone = T_drone_p.col(3);
        drone_pose.position_x = t_drone(0,0);
        drone_pose.position_y = t_drone(1,0);
        drone_pose.position_z = t_drone(2,0);
        cout << "无人机的平移向量为 " << endl << t_drone << endl;
        cout << "无人机位姿 x = " << drone_pose.position_x << endl;
        cout << "无人机位姿 y = " << drone_pose.position_y << endl;
        cout << "无人机位姿 z = " << drone_pose.position_z << endl;
        // cout << currentTime << endl;
        Quaterniond q_drone(R_drone);//将旋转矩阵转为四元数
        cout << "四元数为" << endl << q_drone.coeffs() << endl;
        cout << " w = "<<q_drone.w()<< " x = " <<q_drone.x()<< " y = " <<q_drone.y()<< " z = " <<q_drone.z()<<endl;

        //以下是发布无人机的位姿 
        geometry_msgs::PoseStamped msg;
        ros::Time currentTime = ros::Time::now();
        msg.header.stamp = currentTime;
        msg.pose.position.x = drone_pose.position_x;
        msg.pose.position.y = drone_pose.position_y;
        msg.pose.position.z = drone_pose.position_z;

        msg.pose.orientation.x = q_drone.x();
        msg.pose.orientation.y = q_drone.y();
        msg.pose.orientation.z = q_drone.z();
        msg.pose.orientation.w = q_drone.w();

        msg.header.frame_id = "odom";
        pose_by_vins_pub.publish(msg);
      }

      if (flag_o = 1)
      {
  
        nav_msgs::Odometry msg1;
        ros::Time currentTime = ros::Time::now();
        msg1.header.stamp = currentTime;
        msg1.header.frame_id = "world";
        msg1.child_frame_id  = "world";
        msg1.pose.pose.position.x = fast_planner_odom.position_x;
        msg1.pose.pose.position.y = fast_planner_odom.position_y;
        msg1.pose.pose.position.z = fast_planner_odom.position_z;

        msg1.pose.pose.orientation.x = fast_planner_odom.orientation_x;
        msg1.pose.pose.orientation.y = fast_planner_odom.orientation_y;
        msg1.pose.pose.orientation.z = fast_planner_odom.orientation_z;
        msg1.pose.pose.orientation.w = fast_planner_odom.orientation_w;

        msg1.twist.twist.linear.x = fast_planner_odom.linear_x ;
        msg1.twist.twist.linear.y = fast_planner_odom.linear_y ;
        msg1.twist.twist.linear.z = fast_planner_odom.linear_z ;

        msg1.twist.twist.angular.x = fast_planner_odom.angular_x ;
        msg1.twist.twist.angular.y = fast_planner_odom.angular_y ;
        msg1.twist.twist.angular.z = fast_planner_odom.angular_z ;

        cout <<"begin public trans_vins_odom"<<endl;
        trans_vins_odom.publish(msg1);
      }

      if (flag_c = 1)
      {
  
        geometry_msgs::PoseStamped msg2;
        ros::Time currentTime = ros::Time::now();
        msg2.header.stamp = currentTime;
        msg2.header.frame_id = "world";
        msg2.pose.position.x = fast_planner_camera.position_x;
        msg2.pose.position.y = fast_planner_camera.position_y;
        msg2.pose.position.z = fast_planner_camera.position_z;

        msg2.pose.orientation.x = fast_planner_camera.orientation_x;
        msg2.pose.orientation.y = fast_planner_camera.orientation_y;
        msg2.pose.orientation.z = fast_planner_camera.orientation_z;
        msg2.pose.orientation.w = fast_planner_camera.orientation_w;


        cout <<"begin public trans_vins_camera"<<endl;
        trans_vins_camera.publish(msg2);
      }

      // 循环等待回调函数
        ros::spinOnce();
      
      // 按照循环频率延时
        loop_rate.sleep();

        ++count;

    }
  
    return 0;
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

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
 // flight_status = msg->data;
  vins_pose.position_x = msg -> pose.pose.position.x;
  vins_pose.position_y = msg -> pose.pose.position.y;
  vins_pose.position_z = msg -> pose.pose.position.z;
 
  vins_pose.orientation_x = msg -> pose.pose.orientation.x;
  vins_pose.orientation_y = msg -> pose.pose.orientation.y;
  vins_pose.orientation_z = msg -> pose.pose.orientation.z;
  vins_pose.orientation_w = msg -> pose.pose.orientation.w;
  flag = 1;
}

void fast_planner_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
 // flight_status = msg->data;
  fast_planner_odom.position_x = msg -> pose.pose.position.y;
  fast_planner_odom.position_y = -(msg -> pose.pose.position.x);
  fast_planner_odom.position_z = msg -> pose.pose.position.z;
  
  fast_planner_odom.orientation_x = msg -> pose.pose.orientation.x;
  fast_planner_odom.orientation_y = msg -> pose.pose.orientation.y;
  fast_planner_odom.orientation_z = msg -> pose.pose.orientation.z;
  fast_planner_odom.orientation_w = msg -> pose.pose.orientation.w;

  fast_planner_odom.linear_x = msg -> twist.twist.linear.x;
  fast_planner_odom.linear_y = msg -> twist.twist.linear.y;
  fast_planner_odom.linear_z = msg -> twist.twist.linear.z;

  fast_planner_odom.angular_x = msg -> twist.twist.angular.x;
  fast_planner_odom.angular_y = msg -> twist.twist.angular.y;
  fast_planner_odom.angular_z = msg -> twist.twist.angular.z;

  flag_o == 1;
}

void fast_planner_camera_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
 // flight_status = msg->data;
  fast_planner_camera.position_x = msg -> pose.pose.position.y;
  fast_planner_camera.position_y = -(msg -> pose.pose.position.x);
  fast_planner_camera.position_z = msg -> pose.pose.position.z;
  
  fast_planner_camera.orientation_x = msg -> pose.pose.orientation.x;
  fast_planner_camera.orientation_y = msg -> pose.pose.orientation.y;
  fast_planner_camera.orientation_z = msg -> pose.pose.orientation.z;
  fast_planner_camera.orientation_w = msg -> pose.pose.orientation.w;

  fast_planner_camera.linear_x = msg -> twist.twist.linear.x;
  fast_planner_camera.linear_y = msg -> twist.twist.linear.y;
  fast_planner_camera.linear_z = msg -> twist.twist.linear.z;

  fast_planner_camera.angular_x = msg -> twist.twist.angular.x;
  fast_planner_camera.angular_y = msg -> twist.twist.angular.y;
  fast_planner_camera.angular_z = msg -> twist.twist.angular.z;

  flag_c == 1;
}





