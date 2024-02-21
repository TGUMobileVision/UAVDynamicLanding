/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;





ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;

ros::ServiceClient drone_arm_service;

ros::ServiceClient query_version_service;

ros::Publisher ctrlThrYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
// geometry_msgs::Quaternion current_atti;
//geometry_msgs::Vector3Stamped current_vel;

struct current_position_def
{
  double x;
  double y;
  double z;
  double orb_x;
  double orb_y;  
  double orb_z;
  double x_fu;
  double y_fu;
};

current_position_def current_position, current_vel,tag_position,vins_position,planning_position,vins_camera_position;

geometry_msgs::Quaternion current_atti;//mocap 提供的姿态



double yawAngle;
double yawAngle_imu;
double yawAngle_vins;
double fcmd;
double fcmd_out;
double x, y;



double d_x,d_y,d_z;
double xx_1 = 0;
double yy_1 = 0;
double zz_1 = 0;
double xx_2 = 0;
double yy_2 = 0;
double zz_2 = 0;
double xx_3 = 0;
double yy_3 = 0;
double zz_3 = 0;
int i = 1;
int ii = 0;
int flag_tag = 0;
double vins_0_x,vins_0_y,vins_0_z;
double tag_0_x,tag_0_y,tag_0_z;
double end_x,end_y,end_z;


//double m = 0.0381;
//double m = 0.045;
double m = 0.040;
double g = 9.8;
double r = 1;
const double Pi = 3.1415926;
double w = 2 * Pi / 1500; //3000次跑一圈即i加到3000时跑完360度
double s = 1;  //orb的深度比例

int flag_a = 0;

int Num_tag = 0;
enum Plan
{
  Myself = 0,
  fast_planner = 1,
} plan;
double Distance_begin_x = 0;
double Distance_begin_y = 0;
double vins_begin_x = 0;
double vins_begin_y = 0;
double vins_begin_z = 0;
double tag_begin_x = 0;
double tag_begin_y = 0;
double tag_begin_z = 0;
int flag_plan = fast_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
/*
    //以下是保存位姿和速度等信息到txt文件中
  string desire_pose = "desire_pose.txt";//期望位姿
  ofstream f_d_p;
  f_d_p.open(desire_pose.c_str()); // delete previos data from last run.
  f_d_p.close();

  string cur_pose = "cur_pose.txt";//当前位姿
  ofstream f_c_p;
  f_c_p.open(cur_pose.c_str()); // delete previos data from last run.
  f_c_p.close();
  // ros::Subscriber sub = nh.subscribe("/pose_by_orb", 10, save_cur_pose_callback);//里面包括位置和yaw角



  string tag_pose = "tag_pose.txt";//当前位姿
  ofstream f_t_p;
  f_t_p.open(tag_pose.c_str()); // delete previos data from last run.
  f_t_p.close();


  // ros::Subscriber sub = nh.subscribe("/dji_sdk/velocity", 10, save_cur_vel_callback);
  
  string cur_yaw = "cur_yaw.txt";//当前yaw
  ofstream f_c_yaw;
  f_c_yaw.open(cur_yaw.c_str()); // delete previos data from last run.
  f_c_yaw.close();

*/

  string vins_trans_pose = "/home/nuc/桌面/cur_position.txt";//当前位置
  ofstream f_v_p;
  f_v_p.open(vins_trans_pose.c_str()); // delete previos data from last run.
  f_v_p.close();

  string planning_pose = "/home/nuc/桌面/desire_position.txt";//当前位姿
  ofstream f_p_p;
  f_p_p.open(planning_pose.c_str()); // delete previos data from last run.
  f_p_p.close();

  string cur_Euler = "/home/nuc/桌面/cur_Euler.txt";//当前欧拉角
  ofstream f_c_E;
  f_c_E.open(cur_Euler.c_str()); // delete previos data from last run.
  f_c_E.close();

  string cur_vel = "/home/nuc/桌面/cur_vel.txt";//当前速度
  ofstream f_c_v;
  f_c_v.open(cur_vel.c_str()); // delete previos data from last run.
  f_c_v.close();

  string cur_f = "/home/nuc/桌面/cur_f.txt";//当前推力
  ofstream f_c_f;
  f_c_f.open(cur_f.c_str()); // delete previos data from last run.
  f_c_f.close();

  string cur_control = "/home/nuc/桌面/日志记录.txt";//当前控制量输出
  ofstream f_c_control;
  f_c_control.open(cur_control.c_str()); // delete previos data from last run.
  f_c_control.close();

  string tag_pose1 = "tag_pose1.txt";//当前位姿
  ofstream f_t_p1;
  f_t_p1.open(tag_pose1.c_str()); // delete previos data from last run.
  f_t_p1.close();

  string tag_pose2 = "tag_pose2.txt";//当前位姿
  ofstream f_t_p2;
  f_t_p2.open(tag_pose2.c_str()); // delete previos data from last run.
  f_t_p2.close();



  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);

  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber velocity = nh.subscribe("/dji_sdk/velocity", 10, &velocity_callback);

  ros::Subscriber mocapsub = nh.subscribe("/qualisys/M100/odom", 10, &mocap_callback); //mocap提供的位姿及速度信息
  ros::Subscriber pose_by_orb_sub = nh.subscribe("/pose_by_orb", 10, &pose_by_orb_callback);//orb提供的位姿及速度信息
  ros::Subscriber pose_by_vins_sub = nh.subscribe("/pose_by_vins", 10, &pose_by_vins_callback);//vins提供的位姿及速度信息
  ros::Subscriber tag_Sub = nh.subscribe("/tag_Odometry", 10, &tag_callback);//订阅tag的话题
  ros::Subscriber target_Sub = nh.subscribe("/planning/pos_cmd", 10, &planning_pos_callback);//订阅通过fast-planner产生的planning位置话题
  ros::Subscriber vins_camera_Sub = nh.subscribe("/vins_estimator/camera_pose", 10, &vins_camera_callback);//订阅通过vins_camera位置话题



  // Publish the control signal
  ctrlThrYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

  drone_arm_service = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");

  query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position())
  {
    ROS_WARN("GPS health insufficient - No local frame reference for height. Exiting.");
   // return 1;
  }


  int j = 0;
  if (my_takeoff(1)) //启动电机
  {
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
      if (i == 1)
      {
        if(vins_position.z <= 0.4)
        {
          zz_1 = zz_1 + 0.005;
          if(zz_1<=0.4)
          {
            zz_1 = zz_1;
          }
          if (zz_1 > 0.4)
          {
            zz_1 = 0.41;
          }
          d_x = 0;
          d_y = 0;
          d_z = zz_1;
          local_position_ctrl1();
          cout<<"zz_1"<<zz_1<<endl;
        }
        if(vins_position.z > 0.4)
        {
          
          zz_1 = 0.401;
          d_x = 0;
          d_y = 0;
          d_z = zz_1;
          local_position_ctrl1();
          
          i = 2;
          cout<<"zz_1且ｉ==2"<<zz_1<<endl;
        }
        cout << "在第一阶段　i = 1" << endl;
      }

      if (i == 2)
      {
        cout<<"flag_tag的值"<<flag_tag<<endl;
        /*由于场地的限制，必须要在一米内找到apriltag降落点*/
        if(vins_position.x <= 1)
        {
          if(xx_2 <= 1)
          {
            xx_2 = xx_2;
          }
          if(xx_2 > 1)
          {
            xx_2 = 1;
          }
          xx_2 = xx_2 + 0.01;
          yy_2 = 0;
          zz_2 = 0.51;
          d_x = xx_2;
          d_y = yy_2;
          d_z = zz_2;
          local_position_ctrl1();
          if (flag_tag == 1)
          {
            i = 3;
            vins_0_x = vins_position.x;
            vins_0_y = vins_position.y;
            vins_0_z = vins_position.z;
            tag_0_x = tag_position.x;
            tag_0_y = tag_position.y;
            cout << "此时看到tag "<< "vins: = "<< vins_0_x << "tag: =  "<<tag_0_x<< endl;
            tag_0_z = 0;
            xx_3 = vins_position.x;
            zz_3 = vins_position.z;
            cout<<"此时flag _tag = 1了，设置好 i = 3"<<endl;
            if(1 == Num_tag)
            {
              vins_begin_x = vins_0_x;
              vins_begin_y = vins_0_y;
              vins_begin_z = vins_0_z;
              tag_begin_x  = tag_0_x;
              tag_begin_y  = tag_0_y;
              tag_begin_z  = tag_0_z;
              Distance_begin_x = vins_begin_x + tag_begin_x;
              Distance_begin_y = vins_begin_y + tag_begin_y;
            }

          }
        }
        if(vins_position.x > 1 && vins_position.x < 3.10)
        {
          xx_2 = xx_2 + 0.008;
          if(xx_2>1 && xx_2<3.10)
          {
            xx_2=xx_2;
          }
          if(xx_2>=3.10)
          {
            xx_2=3.10;
          }
      
          //yy_2 = 0;
          yy_2=yy_2+0.006;
          if(yy_2<=1.1)
          {
            yy_2=yy_2;
          }
          if(yy_2>1.1)
          {
            yy_2=1.1;
          }
    
          zz_2 = -0.0625 * (xx_2)*(xx_2) + 0.5625;
          if(zz_2>0)
          {
            zz_2=zz_2;
          }
          if(zz_2<=0)
          {
            zz_2=-0.1;
          }
          d_x = xx_2;
          d_y = yy_2;
          d_z = zz_2;
  
          local_position_ctrl1();
          cout << "xx_2 > 1 && xx_2 < 3" <<endl;
        }
        if(vins_position.x > 3.01)
        {
          local_position_ctrl0();
          cout <<"在第二阶段使用use local_position_ctrl０"<<endl;
        }
        cout << "在第二阶段　i = 2" << endl;
      }

      if( i == 3 )
      {
        if((Distance_begin_x != (vins_0_x + tag_0_x)) && (Distance_begin_x != 0))
        {
          end_x = vins_0_x + tag_0_x + 0.5;
          cout <<"end_x = "<< end_x <<endl;
          cout << "target x is moved" << endl;
        }
        else
        {
          end_x = Distance_begin_x;
        }
        if((Distance_begin_y != (vins_0_y + tag_0_y)) && (Distance_begin_y != 0))
        {
          end_y = vins_0_y + tag_0_y;
          cout <<"end_y = "<< end_y <<endl;
          cout << "target y is moved" << endl;
        }
        else
        {
          end_y = Distance_begin_y;
        }        

        end_z = 0;
        
        if((vins_position.x < end_x) || (vins_position.y < end_y) || (vins_position.z > end_z))
        {
          
          if(Myself == flag_plan)
          {
            //xx_3 = xx_3 + 0.004;
            xx_3 = xx_3 + 0.008;
            if(xx_3 <= end_x)
            {
              xx_3 = xx_3;
            }
            if(xx_3 > end_x)
            {
              xx_3 = end_x;
            }
            cout<<"xx_3 = "<<xx_3<<endl;

            yy_3=yy_3+0.006;
            if(yy_3 <= end_y)
            {
              yy_3 = yy_3;
            }
            if(yy_3 > end_y)
            {
              yy_3 = end_y;
            }
            cout<<"yy_3="<< yy_3<<endl;

            zz_3 = 2 * vins_begin_x * vins_begin_z / (end_z - vins_begin_x) / (end_z - vins_begin_x) * xx_2 
                    -vins_begin_z / (end_z - vins_begin_x) / (end_z - vins_begin_x) * (xx_2)*(xx_2)  
                    + vins_begin_z - (vins_begin_x * vins_begin_x)  * ( vins_begin_z ) / (end_z - vins_begin_x) / (end_z - vins_begin_x);
            if(zz_3>0)
            {
              zz_3=zz_3;
            }
            if(zz_3<=0)
            {
              zz_3=-0.1;
            }
          }
          else if(fast_planner == flag_plan)
          {
            xx_3 = planning_position.x;
            if(xx_3 <= end_x)
            {
              xx_3 = xx_3;
            }
            if(xx_3 > end_x)
            {
              xx_3 = end_x;
            }

            yy_3 = planning_position.y;
            if(yy_3 <= end_y)
            {
              yy_3 = yy_3;
            }
            if(yy_3 > end_y)
            {
              yy_3 = end_y;
            }  

            zz_3 = planning_position.z;
            if(zz_3 > 0)
            {
              zz_3 = zz_3;
            }          
            else if(zz_3 <= 0)
            {
              zz_3 = -0.1;
            }
          }

          d_x = xx_3;
          d_y = yy_3;
          d_z = zz_3;
          cout << "xx_3 < end_x" <<endl;
          local_position_ctrl1();
        }
        if(vins_position.x >= end_x - 0.1  )
        {
          local_position_ctrl0();
          cout <<"在第三阶段使用use local_position_ctrl0"<<endl;
        }

        ros::Time currentTime11 = ros::Time::now();
        string tag_pose1 = "/home/nuc/桌面/tag_position1.txt";//规划位姿
        ofstream f_t_p1;
        f_t_p1.open(tag_pose1.c_str(),ios::app);
        f_t_p1 << fixed;
        f_t_p1 << currentTime11 << setprecision(9) <<" "<< tag_0_x << " " << tag_0_y << " " << tag_0_z << endl;
        f_t_p1.close();

        ros::Time currentTime12 = ros::Time::now();
        string tag_pose2 = "/home/nuc/桌面/tag_position2.txt";//规划位姿
        ofstream f_t_p2;
        f_t_p2.open(tag_pose2.c_str(),ios::app);
        f_t_p2 << fixed;
        f_t_p2 << currentTime12 << setprecision(9) <<" "<< end_x - vins_position.x << " " << end_y - vins_position.y << " " << 0 << endl;
        f_t_p2.close();

        cout <<"在第三阶段　　ｉ　= 3"<<endl;
      }


        
     
 
      



      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  double xcmd, ycmd, zcmd;
  sensor_msgs::Joy controlPosYaw;
  ////
  ///
  //

}

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */

void local_position_ctrl0() 
{


  cout << "now used local_position_ctrl0" << endl;
/*
  //Eigen::Vector3d p_d(0, 0, 0.5);//使用fast-planner产生的目标点
  Eigen::Vector3d p_d(0, 0, 0.49);
 // cout << "p_d = " << p_I_x << "  "<< p_I_y << "  "<< p_I_z << endl;


//  Eigen::Vector3d p_I(p_I_x, p_I_y, p_I_z);//使用dji模拟器时的当前位置提供

  //Eigen::Vector3d p_I(p_I_x_, p_I_y_, p_I_z_);//使用vins-mono时的当前位置提供
  Eigen::Vector3d p_I(vins_position.x, vins_position.y, vins_position.z);
  //Eigen::Vector3d p_I(vins_camera_position.x, vins_camera_position.y, vins_camera_position.z);
  cout << "p_I = "  << endl<<p_I<<endl;

  Eigen::Vector3d e_p = p_I - p_d;
  cout<<"ep="<<e_p<<endl;


  Eigen::Vector3d v_d(0, 0, 0);
  //Eigen::Vector3d v_I(0, 0, 0);//in mocap
  Eigen::Vector3d v_I(current_vel.x, current_vel.y, current_vel.z);
  Eigen::Vector3d e_v = v_I - v_d;
  cout << "v_I = " << current_vel.x << "  "<< current_vel.y << "  "<< current_vel.z << endl;
  cout<<"ev="<<e_v<<endl;
  Eigen::Vector3d e3(0, 0, 1);
  Eigen::Vector3d p_d_dao(0, 0, 0);
  Eigen::Matrix3d k_p;

     k_p << 0.09,   0 , 0 ,
              0 , 0.36 , 0 ,
              0 ,   0 , 0.10 ;
  Eigen::Matrix3d k_v;

 k_v << 0.09, 0, 0,
      0, 0.15, 0,
      0, 0, 0.12;
  Eigen::Vector3d F = -(k_p * e_p) - (k_v * e_v) + (m * g * e3) - (m * p_d_dao);
  double f = F.norm(); //模长
  cout << "f = " << f << endl;
  Eigen::Vector3d r_3d = F / f;
  Eigen::Vector3d n_2d(0, 1, 0);
  Eigen::Vector3d r_1d_1 = n_2d.cross(r_3d); //cross()来实现叉乘
  Eigen::Vector3d r_1d = r_1d_1 / r_1d_1.norm();
  Eigen::Vector3d r_2d = r_3d.cross(r_1d);
  Eigen::Matrix3d R_d;
  R_d.col(0) = r_1d;
  R_d.col(1) = r_2d;
  R_d.col(2) = r_3d;
  Eigen::Vector3d eulerAngle = R_d.eulerAngles(2, 1, 0); //旋转矩阵转欧拉角
  // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));

  double yaw_cmd;        //yaw的角速率
  //double yaw = yawAngle;       //当前的yaw（通过orb得到的）
  double yaw = yawAngle_vins;       //当前的yaw（通过vins得到的）
  //double yaw = yawAngle_imu; //当前的yaw（通过dji-imu得到的）

   cout << "回调函数计算的偏航角为: " << yaw << endl;

  //yaw_cmd = 2.8 * (0 - yaw); //期望的yaw为0,
  //yaw_cmd = - 2.8 * (0 - yaw); //期望的yaw为0,
  yaw_cmd = 2.8 * ( 0 - yaw);


   cout << "偏航角的控制量为：" << yaw_cmd << endl;

  double fcmd = f * 100;
  double rcmd = eulerAngle(2);
  double pcmd = eulerAngle(1);
  

  cout << "计算的推力="<< fcmd << endl;
  //炸机时悬停
  if (fcmd > 60)
  {
    fcmd = 40;
    yaw_cmd = 0;
    rcmd = 0;
    pcmd = 0;
    cout << "推力大于60,强制变成"<< fcmd << endl;
  }
  cout << "实际的推力="<< fcmd << endl;
 //fcmd = 20;
*/
  
  double fcmd = 10;
  double rcmd = 0;
  double pcmd = 0;
  double yaw_cmd = 0;

  sensor_msgs::Joy controlThrustYawRate;
  uint8_t flag = (DJISDK::VERTICAL_THRUST |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);
  controlThrustYawRate.axes.push_back(rcmd);
  controlThrustYawRate.axes.push_back(pcmd);
  controlThrustYawRate.axes.push_back(fcmd);
  controlThrustYawRate.axes.push_back(yaw_cmd);
  controlThrustYawRate.axes.push_back(flag);

  ctrlThrYawPub.publish(controlThrustYawRate);


}



void local_position_ctrl1() 
{

  cout << "now used local_position_ctrl1" << endl;
  //Eigen::Vector3d p_d(0, 0, 0.5);//使用fast-planner产生的目标点
  //Eigen::Vector3d p_d(0, 0, 0.49);

  //Eigen::Vector3d p_d(planning_position.x, planning_position.y, planning_position.z);
 // cout << "p_d = " << p_I_x << "  "<< p_I_y << "  "<< p_I_z << endl;
  Eigen::Vector3d p_d(d_x, d_y, d_z);
  cout << "p_d = "  << endl<< p_d <<endl;

//  Eigen::Vector3d p_I(p_I_x, p_I_y, p_I_z);//使用dji模拟器时的当前位置提供

  //Eigen::Vector3d p_I(p_I_x_, p_I_y_, p_I_z_);//使用vins-mono时的当前位置提供
  Eigen::Vector3d p_I(vins_position.x, vins_position.y, vins_position.z);
  //Eigen::Vector3d p_I(vins_camera_position.x, vins_camera_position.y, vins_camera_position.z);
  cout << "p_I = "  << endl<<p_I<<endl;

  Eigen::Vector3d e_p = p_I - p_d;
  cout<<"ep="<< e_p <<endl;


  Eigen::Vector3d v_d(0, 0, 0);
  //Eigen::Vector3d v_I(0, 0, 0);//in mocap
  Eigen::Vector3d v_I(current_vel.x, current_vel.y, current_vel.z);
  Eigen::Vector3d e_v = v_I - v_d;
  cout << "v_I = " << current_vel.x << "  "<< current_vel.y << "  "<< current_vel.z << endl;
  cout<<"ev="<<e_v<<endl;
  Eigen::Vector3d e3(0, 0, 1);
  Eigen::Vector3d p_d_dao(0, 0, 0);
  Eigen::Matrix3d k_p;
  // k_p << 0.09,   0 , 0 ,
  //          0 , 0.09, 0 ,
  //          0 ,   0 , 0.1;

     k_p << 0.10,   0 , 0 ,
              0 , 0.36 , 0 ,
              0 ,   0 , 0.10 ;
  Eigen::Matrix3d k_v;
  // k_v << 0.09, 0, 0,
  //     0, 0.09, 0,
  //     0, 0, 0.12;

 k_v << 0.09, 0, 0,
      0, 0.15, 0,
      0, 0, 0.11;
  Eigen::Vector3d F = -(k_p * e_p) - (k_v * e_v) + (m * g * e3) - (m * p_d_dao);
  double f = F.norm(); //模长
  cout << "f = " << f << endl;
  Eigen::Vector3d r_3d = F / f;
  Eigen::Vector3d n_2d(0, 1, 0);
  Eigen::Vector3d r_1d_1 = n_2d.cross(r_3d); //cross()来实现叉乘
  Eigen::Vector3d r_1d = r_1d_1 / r_1d_1.norm();
  Eigen::Vector3d r_2d = r_3d.cross(r_1d);
  Eigen::Matrix3d R_d;
  R_d.col(0) = r_1d;
  R_d.col(1) = r_2d;
  R_d.col(2) = r_3d;
  Eigen::Vector3d eulerAngle = R_d.eulerAngles(2, 1, 0); //旋转矩阵转欧拉角
  // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));

  double yaw_cmd;        //yaw的角速率
  //double yaw = yawAngle;       //当前的yaw（通过orb得到的）
  double yaw = yawAngle_vins;       //当前的yaw（通过vins得到的）
  //double yaw = yawAngle_imu; //当前的yaw（通过dji-imu得到的）

   cout << "回调函数计算的偏航角为: " << yaw << endl;

  //yaw_cmd = 2.8 * (0 - yaw); //期望的yaw为0,
  //yaw_cmd = - 2.8 * (0 - yaw); //期望的yaw为0,
  yaw_cmd = 2.8 * ( 0 - yaw);


   cout << "偏航角的控制量为：" << yaw_cmd << endl;

  double fcmd = f * 100;
  double rcmd = eulerAngle(2);
  double pcmd = eulerAngle(1);
  cout << "横滚角的控制量为：" << rcmd << endl;
  cout << "俯仰角的控制量为：" << pcmd << endl;

  cout << "计算的推力="<< fcmd << endl;
  //炸机时悬停
  if (fcmd > 60)
  {
    fcmd = 40;
    yaw_cmd = 0;
    rcmd = 0;
    pcmd = 0;
    cout << "推力大于60,强制变成"<< fcmd << endl;
  }
  cout << "实际的推力="<< fcmd << endl;
 //fcmd = 20;

  
//cout<<"最后发给控制器的fcmd = "<< fcmd_out << endl;

  sensor_msgs::Joy controlThrustYawRate;
  uint8_t flag = (DJISDK::VERTICAL_THRUST |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);
  controlThrustYawRate.axes.push_back(rcmd);
  controlThrustYawRate.axes.push_back(pcmd);
  controlThrustYawRate.axes.push_back(fcmd);
  controlThrustYawRate.axes.push_back(yaw_cmd);
  controlThrustYawRate.axes.push_back(flag);

  ctrlThrYawPub.publish(controlThrustYawRate);

  ros::Time currentTime4 = ros::Time::now();
  string cur_control = "/home/nuc/桌面/日志记录.txt";
  ofstream f_c_control;
  f_c_control.open(cur_control.c_str(),ios::app);
  f_c_control << fixed;
  f_c_control <<endl<<"当前质量m = " << m*100 <<endl<< "期望位置为"<< endl << p_d << endl <<"实际输出的推力  = "<< fcmd <<endl << "  实际位置为  "<<endl<< p_I <<  endl<<"当前e_p: "<<endl<<e_p <<"__________________________________"<<endl;
  f_c_control.close();

  ros::Time currentTime5 = ros::Time::now();
  string cur_Euler = "/home/nuc/桌面/cur_Euler.txt";//当前控制量输出
  ofstream f_c_E;
  f_c_E.open(cur_Euler.c_str(),ios::app);
  f_c_E << fixed;
  f_c_E << currentTime5 << setprecision(9) <<" "<< yaw << " " << yaw_cmd << " " << pcmd << " "<< rcmd << endl;
  f_c_E.close();


  ros::Time currentTime6 = ros::Time::now();
  string cur_f = "/home/nuc/桌面/cur_f.txt";//当前控制量输出
  ofstream f_c_f;
  f_c_f.open(cur_f.c_str(),ios::app);
  f_c_f << fixed;
  f_c_f << currentTime5 << setprecision(9) <<" "<< fcmd << endl;
  f_c_f.close();

  ros::Time currentTime7 = ros::Time::now();
  string planning_pose = "/home/nuc/桌面/desire_position.txt";//规划位姿
  ofstream f_p_p;
  f_p_p.open(planning_pose.c_str(),ios::app);
  f_p_p << fixed;
  f_p_p << currentTime7 << setprecision(9) <<" "<< d_x << " " << d_y << " " << d_z << endl;
  f_p_p.close();

}


bool my_takeoff(int arm) //自己的起飞函数
{
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = arm;
  drone_arm_service.call(droneArmControl);

  if (!droneArmControl.response.result)
  {
    ROS_ERROR("my takeoff fail");
    return false;
  }
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if (!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(authority);

  if (!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if (query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  display_mode = msg->data;
}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (ros::Time::now() - start_time > ros::Duration(5))
  {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }

  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (ros::Time::now() - start_time > ros::Duration(20))
  {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ((display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}

//以下为模拟器状态下用的位姿和速度
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
   local_position = *msg;
}

/*
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) //
{
  // current_atti = msg->quaternion;
  // yawAngle = toEulerAngle(current_atti).z;
}
*/


//以下为mocap实验条件下用的位姿和速度
void mocap_callback(const nav_msgs::Odometry::ConstPtr &msg) //mocap 的回调函数，提供位姿及速度信息
{
/*
  current_position.x = msg->pose.pose.position.x; //提供当前位置
  current_position.y = msg->pose.pose.position.y;
  current_position.z = msg->pose.pose.position.z;
*/
/*
  current_atti = msg->pose.pose.orientation; //提供当前姿态
  yawAngle = toEulerAngle(current_atti).z;
*/
/*
  current_vel.x =msg->twist.twist.linear.x; //提供当前速度
  current_vel.y = msg->twist.twist.linear.y;
  current_vel.z = msg->twist.twist.linear.z;
*/
}
/*
//以下为orb实验条件下用的位姿和速度
void orb_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)//orb的回调函数，提供位姿信息
{

  current_position.orb_x = msg->pose.position.x;//提供当前位置
  current_position.orb_y = msg->pose.position.y;
  current_position.orb_z = msg->pose.position.z;
  current_position.x =  s * current_position.orb_z;
  current_position.y =  s * (- current_position.orb_x );
  current_position.z =  s * (- current_position.orb_y );

  current_atti = msg->pose.orientation; //提供当前姿态
  yawAngle = toEulerAngle(current_atti).z;

}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) //myself
{
  current_vel.x_fu = msg->vector.x;//提供当前速度
  current_vel.y_fu = msg->vector.y;
  current_vel.x = -current_vel.x_fu;
  current_vel.y = -current_vel.y_fu;
  current_vel.z = msg->vector.z;
}
*/

void pose_by_orb_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)//orb的回调函数，提供位姿信息
{
/*
  current_position.orb_x = msg->pose.position.x;//提供当前位置
  current_position.orb_y = msg->pose.position.y;
  current_position.orb_z = msg->pose.position.z;
  current_position.x =  s * (current_position.orb_x);
  current_position.y =  s * (current_position.orb_y);
  current_position.z =  s * (current_position.orb_z);
  cout << "回调函数里的深度s为" << s << endl;

  ros::Time currentTime7 = ros::Time::now();
  string cur_pose = "cur_pose.txt";//当前位姿
  ofstream f_c_p;
  f_c_p.open(cur_pose.c_str(),ios::app);
  f_c_p << fixed;
  f_c_p << currentTime7 << setprecision(9) <<" "<< current_position.x << " " << current_position.y << " " << current_position.z << endl;
  f_c_p.close();


  current_atti = msg->pose.orientation; //提供当前姿态
  yawAngle = toEulerAngle(current_atti).z;

  ros::Time currentTime8 = ros::Time::now();
  string cur_yaw = "cur_yaw.txt";//当前位姿
  ofstream f_c_yaw;
  f_c_yaw.open(cur_yaw.c_str(),ios::app);
  f_c_yaw << fixed;
  f_c_yaw << currentTime8 << setprecision(9) <<" "<< yawAngle  << endl;
  f_c_yaw.close();

  // cout << "回调函数计算的偏航角为 " << yawAngle << endl;

  */
}

void pose_by_vins_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)//vins的回调函数，提供位姿信息
{
  cout<< " 听到了pose_by_vins的话题"<< endl;
  vins_position.x = msg->pose.position.x;//提供当前位置
  vins_position.y = msg->pose.position.y;
  vins_position.z = msg->pose.position.z;

  current_atti = msg->pose.orientation; //提供当前姿态
  yawAngle_vins = toEulerAngle(current_atti).z;

  ros::Time currentTime8 = ros::Time::now();
  string vins_trans_pose = "/home/nuc/桌面/cur_position.txt";//当前位姿
  ofstream f_v_p;
  f_v_p.open(vins_trans_pose.c_str(),ios::app);
  f_v_p << fixed;
  f_v_p << currentTime8 << setprecision(9) <<" "<< vins_position.x << " " << vins_position.y << " " << vins_position.z << endl;
  f_v_p.close();


  // cout << "回调函数计算的偏航角为 " << yawAngle << endl;

  // cout << "pose_by_vins_callback 回调函数de flag_a = " << flag_a << endl;
}

void vins_camera_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  cout<<"听到了vins_camera"<<endl;
 // flight_status = msg->data;
  vins_camera_position.x = msg -> pose.pose.position.y;
  vins_camera_position.y_fu = msg -> pose.pose.position.x;
  vins_camera_position.y = - vins_camera_position.y_fu;
  vins_camera_position.z = msg -> pose.pose.position.z;


  current_atti = msg->pose.pose.orientation; //提供当前姿态
  yawAngle_vins = toEulerAngle(current_atti).z;


}


void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) //myself
{
  current_vel.x = msg->vector.x;//提供当前速度
  current_vel.y = msg->vector.y;
  current_vel.z = msg->vector.z;
/*
  cout << "当前x方向的速度为" << current_vel.x << endl;
  cout << "当前y方向的速度为" << current_vel.y << endl;
  cout << "当前z方向的速度为" << current_vel.z << endl;
*/
  ros::Time currentTime9 = ros::Time::now();
  string cur_vel = "/home/nuc/桌面/cur_vel.txt";//当前位姿
  ofstream f_c_v;
  f_c_v.open(cur_vel.c_str(),ios::app);
  f_c_v << fixed;
  f_c_v << currentTime9 << setprecision(9) <<" "<< current_vel.x  << " " << current_vel.y << " " << current_vel.z <<endl;
  f_c_v.close();


}


void tag_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
 // flight_status = msg->data;
  tag_position.x_fu = msg -> pose.pose.position.x;
  tag_position.x = -tag_position.x_fu;
  tag_position.y_fu = msg -> pose.pose.position.y;
  tag_position.y = -tag_position.y_fu;
  tag_position.z = msg -> pose.pose.position.z;

  flag_tag = 1;
/*
  ros::Time currentTime10 = ros::Time::now(); 
  string tag_pose = "tag_pose.txt";//当前位姿
  ofstream f_t_p;
  f_t_p.open(tag_pose.c_str(),ios::app);
  f_t_p << fixed;
  f_t_p << currentTime10 << setprecision(9) <<" "<< tag_position.x  << " " << tag_position.y << " " << tag_position.z <<endl;
  f_t_p.close();
*/
}



void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) //
{
   current_atti = msg->quaternion;
   yawAngle_imu = toEulerAngle(current_atti).z;
}


void planning_pos_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)//target的回调函数，提供目标点信息
{

  planning_position.x = msg->position.x;//提供当前位置
  planning_position.y = msg->position.y;
  planning_position.z = msg->position.z;

/*
  ros::Time currentTime8 = ros::Time::now();
  string planning_pose = "/home/nuc/桌面/desire_position.txt";//规划位姿
  ofstream f_p_p;
  f_p_p.open(planning_pose.c_str(),ios::app);
  f_p_p << fixed;
  f_p_p << currentTime8 << setprecision(9) <<" "<< planning_position.x << " " << planning_position.y << " " << planning_position.z << endl;
  f_p_p.close();
*/

  flag_a = 1;
  cout << "planning_pos_callback 回调函数de flag_a = " << flag_a << endl;

}





