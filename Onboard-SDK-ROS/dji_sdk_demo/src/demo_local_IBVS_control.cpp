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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>


using namespace cv;
using namespace std;
using namespace Eigen;






ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;

ros::ServiceClient drone_arm_service;

ros::ServiceClient query_version_service;

ros::Publisher ctrlThrYawPub;

ros::Publisher ctrlBrakePub;

ros::Publisher ctrlVelYawRatePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
geometry_msgs::Quaternion current_atti;//mocap 提供的姿态
//sensor_msgs::Imu current_imu;
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

struct current_imu_def //自己用结构体定义，也可以按照以前的方法单独进行定义
{
  double rx;
  double ry;
  double rz;
  
};
current_imu_def current_angular_velocity ;



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
double zz_3 = 1.15;
int jj = 2; 
int ii = 0;
int flag_tag = 0;
double vins_0_x,vins_0_y,vins_0_z;
double tag_0_x,tag_0_y,tag_0_z;
double end_x,end_y,end_z;


//double m = 0.0381;
double m = 0.045;
//double m = 0.040;
//double g = 9.8;
double r = 1;
const double Pi = 3.1415926;
double w = 2 * Pi / 1500; //3000次跑一圈即i加到3000时跑完360度
double s = 1;  //orb的深度比例

int flag_a = 0;

//int i;
float roll, pitch, yaw, yawspeed, quat[4];
float rolld = 0, pitchd = 0, roll1 = 0, roll2 = 0, pitch1 = 0, pitch2 = 0;
//float rp_thr = 0.173, thr_sp = 0.660;
int i;
float hd, z, v;

int reason = 0;
float qzd = 1, qxd = 0, qyd = 0, alphad = 0, dtt = 0.01, k5 = 0.4, k6 = 0.3,
zd = 1.5, c1 = 0.2,  c2 = 0.2,  g = 9.81,  k4 =2,  dx6;
//double ad = 0.00000045;
//double ad = 0.0000009;
//double ad = 0.00000072; 
double ad = 0.00000032; 
double fl =0.00188;
double k1 = 2;
double k2 = 2;
double k3 = 2;
//double kpp = 0.25;
double kpp = 0.67; 
double mass = 4.50;
double gra = mass*9.81;
//double dt = 0.10;
double dt = 0.005;
// double rx ;
// double ry ;
// double rz ;
double f11, f12, f13, v11, v12, v13 ;	
float rp_thr = 0.173, thr_sp = 0.660, r_err = 0, p_err = 0;
	//0.17365;  
float mm[4], u[4], vv[4];
float m10, m00, m01, ug, vg, u20, u02, u11, a1, qz, qx, qy, u1, alpha, q4;
int cnt, piccnt = 0;
char buffer[20];
float xx[4], yy[4], a[8];
//   double a[];
	/***************************************
	*
	* PID Variables
	*
	***************************************/
	float pE = 0, iE = 0, dE = 0, ek = 0, ek1 = 0, ek2 = 0;
	float kp = 0, ki = 0, kd = 0;
	//Matrix and vectors
	Matrix3f Rx4, Rx5, R;  // 
	Vector3f p[4], tmpv_arr[4], q, q1, tmpv,tmpv1, voe, V, dvoe, qe1, dqe1, qee, ve, q2, q3, df, f; // 
	RowVector3f tmph; // 


void psort(float a[])  //a={x1,y1,x2,y2,x3,y3,x4,y4}
{
	float tempx, tempy;
	for (int i = 0; i<2; i++)
	{
		for (int j = i; j<3; j++)
		{
			if (a[j * 2]<a[j * 2 + 2])
			{
				
				tempx = a[j * 2];
				tempy = a[j * 2 + 1];
				a[j * 2] = a[j * 2 + 2];
				a[j * 2 + 1] = a[j * 2 + 3];
				a[j * 2 + 2] = tempx;
				a[j * 2 + 3] = tempy;

			}
		}
	}

	if (a[1]>a[3])
	{
		tempx = a[0];
		tempy = a[1];
		a[0] = a[2];
		a[1] = a[3];
		a[2] = tempx;
		a[3] = tempy;
	}
	if (a[5]>a[7])
	{
		tempx = a[4];
		tempy = a[5];
		a[4] = a[6];
		a[5] = a[7];
		a[6] = tempx;
		a[7] = tempy;
	}
}


Mat img;
Mat templ; 
Mat result;
Mat Frame;
IplImage*  pFrame  ;
CvMemStorage *storage = cvCreateMemStorage(); //创建内存块
CvSeq *seq = NULL;
CvSeq *c = NULL ;
CvScalar cs;
CvRect rc;
const char* image_window = "Image Window";
const char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_local_IBVS_control1_node");
  ros::NodeHandle nh;

  string vins_trans_pose = "/home/nuc/桌面/cur_position.txt";//当前位置
  ofstream f_v_p;
  f_v_p.open(vins_trans_pose.c_str()); // delete previos data from last run.
  f_v_p.close();

  string vins_trans_pose1 = "/home/nuc/桌面/cur_position1.txt";//当前位置
  ofstream f_c_pp;
  f_c_pp.open(vins_trans_pose1.c_str()); // delete previos data from last run.
  f_c_pp.close();

  string planning_pose = "/home/nuc/桌面/desire_position.txt";//当前位姿
  ofstream f_p_p;
  f_p_p.open(planning_pose.c_str()); // delete previos data from last run.
  f_p_p.close();

  string cur_Euler = "/home/nuc/桌面/cur_Euler.txt";//当前欧拉角
  ofstream f_c_E;
  f_c_E.open(cur_Euler.c_str()); // delete previos data from last run.
  f_c_E.close();

  string cur_Euler1 = "/home/nuc/桌面/cur_Euler1.txt";//当前欧拉角
  ofstream f_c_E1;
  f_c_E1.open(cur_Euler1.c_str()); // delete previos data from last run.
  f_c_E1.close();

  string Virtual_Frame = "/home/nuc/桌面/Virtual_Frame.txt";//当前虚拟坐标
  ofstream f_V_F;
  f_V_F.open(Virtual_Frame.c_str()); // delete previos data from last run.
  f_V_F.close();

  string cur_vel = "/home/nuc/桌面/cur_vel.txt";//当前速度
  ofstream f_c_v;
  f_c_v.open(cur_vel.c_str()); // delete previos data from last run.
  f_c_v.close();

  string cur_vv = "/home/nuc/桌面/cur_vv.txt";//当前速度
  ofstream f_c_vv;
  f_c_vv.open(cur_vv.c_str()); // delete previos data from last run.
  f_c_vv.close();

  string cur_f = "/home/nuc/桌面/cur_f.txt";//当前推力
  ofstream f_c_f;
  f_c_f.open(cur_f.c_str()); // delete previos data from last run.
  f_c_f.close();

  string cur_ff = "/home/nuc/桌面/cur_ff.txt";//当前推力
  ofstream f_c_ff;
  f_c_ff.open(cur_ff.c_str()); // delete previos data from last run.
  f_c_ff.close();

  string cur_q = "/home/nuc/桌面/cur_q.txt";//当前图像据
  ofstream f_c_qq;
  f_c_qq.open(cur_q.c_str()); // delete previos data from last run.
  f_c_qq.close();

  string cur_qq = "/home/nuc/桌面/cur_qq.txt";//当前图像据
  ofstream f_c_qqq;
  f_c_qqq.open(cur_qq.c_str()); // delete previos data from last run.
  f_c_qqq.close();
  
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

    ofstream outfile("/home/nuc/桌面/圆的中心坐标.txt");
	  ofstream out("/home/nuc/桌面/真实平面下的像素坐标.txt");
    ofstream out_V("/home/nuc/桌面/虚拟平面下的像素坐标.txt");
    ofstream out_q("/home/nuc/桌面/图像距.txt");
    ofstream out_u("/home/nuc/桌面/推力.txt");
    ofstream out_E("/home/nuc/桌面/欧拉角.txt");



  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);

  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber velocity = nh.subscribe("/dji_sdk/velocity", 10, &velocity_callback);

  ros::Subscriber imuSub = nh.subscribe("/dji_sdk/imu", 10, &imu_callback);

  ros::Subscriber mocapsub = nh.subscribe("/qualisys/M100/odom", 10, &mocap_callback); //mocap提供的位姿及速度信息
  ros::Subscriber pose_by_orb_sub = nh.subscribe("/pose_by_orb", 10, &pose_by_orb_callback);//orb提供的位姿及速度信息
  ros::Subscriber pose_by_vins_sub = nh.subscribe("/pose_by_vins", 10, &pose_by_vins_callback);//vins提供的位姿及速度信息
  ros::Subscriber tag_Sub = nh.subscribe("/tag_Odometry", 10, &tag_callback);//订阅tag的话题
  ros::Subscriber target_Sub = nh.subscribe("/planning/pos_cmd", 10, &planning_pos_callback);//订阅通过fast-planner产生的planning位置话题
  ros::Subscriber vins_camera_Sub = nh.subscribe("/vins_estimator/camera_pose", 10, &vins_camera_callback);//订阅通过vins_camera位置话题



  // Publish the control signal
  ctrlThrYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  ctrlBrakePub =  nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
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


	  VideoCapture pCapture(0);

	 if (pCapture.isOpened()) {
	 	cout << "open the camera" << endl;
	 	// return -1;
	 }
	 else
	 {
	 	cout << "can not the camera" << endl;
	 	//return -1;
	 }
	 //VideoCapture cap;
	 //cap.open(1);
   double frame_width=0;
	 double frame_height=0;
	 double frame_fps = 0;
   double width = 640 ;
   double height = 480;
   pCapture.set(CAP_PROP_FRAME_WIDTH,width);
   pCapture.set(CAP_PROP_FRAME_HEIGHT,height);
	 frame_width=pCapture.get(CV_CAP_PROP_FRAME_WIDTH);
	 frame_height=pCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
	 frame_fps=pCapture.get(CV_CAP_PROP_FPS);

	 cout << "frame_width=" << frame_width << endl;
	 cout << "frame_height=" << frame_height << endl;
	 cout << "frame_fps=" << frame_fps << endl;

	 VideoWriter write;
	 write.open("/home/nuc/桌面/1.avi", CAP_ANY, 30.0, Size(pCapture.get(CAP_PROP_FRAME_WIDTH), pCapture.get(CAP_PROP_FRAME_HEIGHT)));

	 if (!write.isOpened())
	 {
	 	cout << "can not write" << endl;
	 	return -1;
	 }
    pCapture >> Frame;
    IplImage* pFrame = new IplImage(Frame);


   IplImage* rgbImg = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 3);
	 IplImage* r1Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* g1Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* b1Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* r2Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* g2Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* b2Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);

	 IplImage* hsvImg = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 3);
	 IplImage* hImg = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* h1Img = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* sImg = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);
	 IplImage* vImg = cvCreateImage(cvGetSize(pFrame), IPL_DEPTH_8U, 1);

  
  
  int j = 0;
  if (my_takeoff(1)) //启动电机
  {

    ros::Rate loop_rate(50);
    while (ros::ok())
    {

     pCapture >> Frame;
		 //IplImage*  pFrame = new IplImage(Frame);
     IplImage* pFrame = new IplImage(Frame);
     
		
		 if (!pFrame) {
		 	cout << "can not read pFrame" << endl;
		 	//break;
		 	//return -1;
		 }

		 //cvShowImage("xiangshi", pFrame);
		 cvCopy(pFrame, rgbImg); 
		 cvSplit(rgbImg, b1Img, g1Img, r1Img, NULL);
		 cvThreshold(r1Img, r2Img, 180, 255, CV_THRESH_BINARY); //
		 cvThreshold(r1Img, r1Img, 240, 255, CV_THRESH_BINARY_INV);
		 //cvShowImage("r1mg", r1Img);
		 cvAnd(r1Img, r2Img, r1Img); 
		 //cvShowImage("Andr1mg", r1Img);
		 cvThreshold(g1Img, g2Img, 90, 255, CV_THRESH_BINARY);
		 //cvShowImage("g1Img", g1Img);
		 cvThreshold(g1Img, g1Img, 150, 255, CV_THRESH_BINARY_INV);
		 //cvShowImage("g2Img", g2Img);
		 cvAnd(g1Img, g2Img, g1Img);
		 //cvShowImage("ANDg1Img", g1Img);
		 cvThreshold(b1Img, b2Img, 110, 255, CV_THRESH_BINARY);
		 cvThreshold(b1Img, b1Img, 230, 255, CV_THRESH_BINARY_INV);
		 cvAnd(b1Img, b2Img, b1Img);
		 //cvShowImage("Andb1Img", b1Img);
		 cvAnd(r1Img, g1Img, r1Img);
		 //cvShowImage("Andr1Img", r1Img);
		 cvAnd(r1Img, b1Img, r1Img);
		 //cvShowImage("Andrr1Img", r1Img);

		 cvCvtColor(pFrame, hsvImg, CV_BGR2HSV);
		 cvSplit(hsvImg, hImg, sImg, vImg, NULL);
		 cvThreshold(hImg, h1Img, 9, 255, CV_THRESH_BINARY_INV);
		 //cvShowImage("h1Img", h1Img);
		 cvThreshold(hImg, hImg, 160, 255, CV_THRESH_BINARY);
		 //cvShowImage("hImg", hImg);
		 cvOr(hImg, h1Img, hImg);
		 //cvShowImage("OrgImg", hImg);
		 cvThreshold(sImg, sImg, 80, 255, CV_THRESH_BINARY);
		 cvAnd(hImg, sImg, hImg);
		 //cvShowImage("AndhImg", hImg);
		 cvOr(hImg, r1Img, hImg);
		 //cvShowImage("OOrImg", hImg);
		 cvFindContours(hImg, storage, &seq, sizeof(CvContour), CV_RETR_EXTERNAL);//检测颜色轮廓

   

      if (jj == 1) //垂直起飞阶段
      {
        if(vins_position.z <= 0.75)
        {
          //zz_1 = zz_1 + 0.005;
           zz_1 = zz_1 + 0.003;
          if(zz_1<=0.75)
          {
            zz_1 = zz_1;
          }
          if (zz_1 > 0.751)
          {
            zz_1 = 0.755;
          }
          d_x = 0;
          d_y = 0;
          d_z = zz_1;
          local_position_ctrl1();
          cout<<"zz_1"<<zz_1<<endl;
        }
        if(vins_position.z > 0.751)
        {
          
          zz_1 = 0.751;
          d_x = 0;
          d_y = 0;
          d_z = zz_1;
          local_position_ctrl1();
          
          jj = 2;
          cout<<"zz_1且 jj==2"<<zz_1<<endl;
        }
        cout << "在第一阶段　jj = 1" << endl;
      }

      if (jj == 2) // 进入目标搜索阶段
      {
        cout<< "flag_tag的值"<<flag_tag<< endl;
        cout << "在第二阶段　jj = ２" << endl;
        //if(vins_position.x <= 1.0) //这个值需要调！！！！！
        if(vins_position.x <= 2.51) //这个值需要调！！！！！
        {
          if(xx_2 <= 2.51)
          {
            xx_2 = xx_2;
          }
          if(xx_2 > 2.51)
          {
            xx_2 = 2.51;
          }
          xx_2 = xx_2 + 0.008;
          if(yy_2 > -0.31){
             yy_2 = yy_2;
          }
          if(yy_2 < -0.31){
            yy_2 = -0.31;
          }
          yy_2 = yy_2 - 0.002;
          zz_2 = 1.19;
          d_x = xx_2;
          d_y = yy_2;
          d_z = zz_2;

            int cnt = 0;

        
            for (c = seq; c!= NULL; c = c->h_next) {  
            rc = cvBoundingRect(c, 0);//生成矩形框的一个的定点的二维点值
            // if (rc.width > 30 && rc.width < 89  &&  rc.height > 30 && rc.height < 89) {
            if (rc.width > 100 &&  rc.height > 100) {
          // if (rc.width > 20) {
            //cout << "rc.width=  " << rc.width << endl;

            xx[cnt] = rc.x + rc.width / 2;  
            //cout << "rc.x=  " << rc.x << endl;
            //cout << "rc.width" << rc.width << endl;
            //cout << "x[cnt]=   " <<x[cnt]  <<  "   cnt=  "<<  cnt << endl;
            yy[cnt] = rc.y + rc.height / 2;
            //cout << "y[cnt]=   " << y[cnt] << "   cnt=  " <<  cnt << endl;
            //cout << "rc.y=   " << rc.y << endl;
            //cout << "rc.height=  " << rc.height << endl;


            //cvDrawRect(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));
            cvRectangle(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));//给检测的轮廓加矩形框
            cvShowImage("jiance", pFrame);

            if (!pFrame) {
            cout << "can not read pFrame" << endl;
            //break;
            //return -1;                                        
            }
            else{
              cout<<"open the pFrame"<< endl;
            }

            Mat mmm = cvarrToMat(pFrame, true);
            write << mmm;

            cnt++; 
            cout << "cnt=  " << cnt << endl;
            //cvWaitKey(10);
            
          }
          cvWaitKey(1);

          
          if (cnt == 4) {

            for (cnt=0; cnt<4; cnt++) {
              //out << "i= " << i << "\t" << "   x[i]=  " << x[i] << "\n";
              outfile << "cnt= " << cnt << "\t" << "   xx[cnt]=  " << xx[cnt] << "\n";
            }
            for (cnt = 0; cnt<4; cnt++) {
              //out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
              outfile << "cnt= " << cnt << "\t" << "   yy[cnt]=  " << yy[cnt] << "\n";
            }
            
            break;
          }
            }

          if (cnt == 4) {
            for (i = 0; i<4; i++) {
            a[2 * i] = xx[i];      // 
            a[2 * i + 1] = yy[i]; //
          }
            psort(a); // 
            for (i = 0; i<4; i++) {
            xx[i] = a[2 * i];
            yy[i] = a[2 * i + 1];
            xx[i] -= 325; //这个应该是把图像平面坐标系移到左上角的坐标系
            yy[i] -= 252;
            //xx[i] = -xx[i];
            //yy[i] = -yy[i];

          }

            for (i = 0; i<4; i++) {
            //	out << "i= " << i << "\t" << "   xx[i]=  " << xx[i] << "\n";
            out << xx[i] << "  \t  " << yy[i] << " \t ";

            if (i == 3) {
              out << " \n ";
            }
            }
            }
         
              if (cnt == 4) //进入 IBVS阶段
            {
              jj = 3;


              }
        
      
        
              //  cvWaitKey(1);


            cout << "now used local_position_ctrl2" << endl;
            //Eigen::Vector3d p_d(0, 0, 0.5);//使用fast-planner产生的目标点
            //Eigen::Vector3d p_d(0, 0, 0.49);

            //Eigen::Vector3d p_d(planning_position.x, planning_position.y, planning_position.z);
            // cout << "p_d = " << p_I_x << "  "<< p_I_y << "  "<< p_I_z << endl;
            Eigen::Vector3d p_d(d_x, d_y, d_z);
            cout << "p_d = "  << endl << p_d <<endl;

            //  Eigen::Vector3d p_I(p_I_x, p_I_y, p_I_z);//使用dji模拟器时的当前位置提供

            //Eigen::Vector3d p_I(p_I_x_, p_I_y_, p_I_z_);//使用vins-mono时的当前位置提供
            Eigen::Vector3d p_I(vins_position.x, vins_position.y, vins_position.z);
            //Eigen::Vector3d p_I(vins_camera_position.x, vins_camera_position.y, vins_camera_position.z);
            cout << "p_I = "  << endl << p_I <<endl;
            Eigen::Vector3d e_p = p_I - p_d;

            cout<<"ep="<< e_p <<endl;
            Eigen::Vector3d v_d(0, 0, 0);
            //Eigen::Vector3d v_I(0, 0, 0);//in mocap
            Eigen::Vector3d v_I(current_vel.x, current_vel.y, current_vel.z);
            Eigen::Vector3d e_v = v_I - v_d;
            cout << "v_I = " << current_vel.x << "  "<< current_vel.y << "  "<< current_vel.z << endl;
            cout<<"ev="<< e_v <<endl;
            Eigen::Vector3d e3(0, 0, 1);
            Eigen::Vector3d p_d_dao(0, 0, 0);
            Eigen::Matrix3d k_p;
            // k_p << 0.09,   0 , 0 ,
            //          0 , 0.09, 0 ,
            //          0 ,   0 , 0.1;

            k_p << 0.10,   0 , 0 ,
                      0 , 0.12 , 0 ,
                      0 ,   0 , 0.10 ;
            Eigen::Matrix3d k_v;
            // k_v << 0.09, 0, 0,
            //     0, 0.09, 0,
            //     0, 0, 0.12;

            k_v << 0.09, 0, 0,
              0, 0.12, 0,
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
            //yaw_cmd = 2.8 * ( 0 - yaw);
            yaw_cmd = 0;


            cout << "偏航角的控制量为：" << yaw_cmd << endl;

              double fcmd = f * 100;
              double rcmd = eulerAngle(2);
              double pcmd = eulerAngle(1);
              cout << "横滚角的控制量为：" << rcmd << endl;
              cout << "俯仰角的控制量为：" << pcmd << endl;

              cout << "计算的推力="<< fcmd << endl;
              f11 = F(0) ;
              f12 = F(1) ;
              f13 = F(2) ;
              ros::Time currentTime13 = ros::Time::now();
              string cur_ff = "/home/nuc/桌面/cur_ff.txt";//当前控制量输出
              ofstream f_c_ff;
              f_c_ff.open(cur_ff.c_str(), ios::app);
              f_c_ff << fixed;
              f_c_ff << currentTime13 << setprecision(9) << " " << fcmd << " " << f11 << " " << f12 << " " << f13 <<endl;
              f_c_ff.close();
              //炸机时悬停
              if (fcmd > 60)
              {
              
              fcmd = 42;
              yaw_cmd = 0;
              rcmd = 0;
              pcmd = 0;
              cout << "推力大于60,强制变成"<< fcmd << endl;
            }
            
            cout << "实际的推力="<< fcmd << endl;
            //fcmd = 20;

            

            sensor_msgs::Joy controlThrustYawRate;
            uint8_t flag = (DJISDK::VERTICAL_THRUST |
                            DJISDK::HORIZONTAL_ANGLE |
                            DJISDK::YAW_ANGLE |
                            DJISDK::HORIZONTAL_BODY |
                            DJISDK::STABLE_ENABLE);
            controlThrustYawRate.axes.push_back(rcmd);
            controlThrustYawRate.axes.push_back(pcmd);
            controlThrustYawRate.axes.push_back(fcmd);
            controlThrustYawRate.axes.push_back(yaw_cmd);
            controlThrustYawRate.axes.push_back(flag);

            ctrlThrYawPub.publish(controlThrustYawRate);

            
            ros::Time currentTime5 = ros::Time::now();
            string cur_Euler = "/home/nuc/桌面/cur_Euler.txt";//当前控制量输出
            ofstream f_c_E;
            f_c_E.open(cur_Euler.c_str(), ios::app);
            f_c_E << fixed;
            f_c_E << currentTime5 << setprecision(9) << " " << yaw << " " << yaw_cmd << " " << pcmd << " " << rcmd << endl;
            f_c_E.close();


            ros::Time currentTime6 = ros::Time::now();
            string cur_f = "/home/nuc/桌面/cur_f.txt";//当前控制量输出
            ofstream f_c_f;
            f_c_f.open(cur_f.c_str(), ios::app);
            f_c_f << fixed;
            f_c_f << currentTime6 << setprecision(9) << " " << fcmd << endl;
            f_c_f.close();

            ros::Time currentTime7 = ros::Time::now();
            string planning_pose = "/home/nuc/桌面/desire_position.txt";//规划位姿
            ofstream f_p_p;
            f_p_p.open(planning_pose.c_str(), ios::app);
            f_p_p << fixed;
            f_p_p << currentTime7 << setprecision(9) << " " << d_x << " " << d_y << " " << d_z << endl;
            f_p_p.close();


         
        }
      }
    

            if(jj ==3) // 进入IBVS阶段
            {
              

              cout <<"在第三阶段　　jj　= 3"<<endl;
            int cnt = 0;

          for (c = seq; c != NULL; c = c->h_next) {
            rc = cvBoundingRect(c, 0);
          //	if (rc.width > 30 && rc.width < 89 && rc.height > 30 && rc.height < 89) {
            if (rc.width > 65 && rc.height > 65 ) {
              //cout << "rc.width=  " << rc.width << endl;

              xx[cnt] = rc.x + rc.width / 2;
              yy[cnt] = rc.y + rc.height / 2;
              cvRectangle(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));
              cvShowImage("jiance", pFrame);
              Mat mm = cvarrToMat(pFrame);
              write << mm;

              cnt++;
              //cout << "cnt=  " << cnt << endl;

            }
            cvWaitKey(1);


            if (cnt == 4) {

              for (cnt = 0; cnt<4; cnt++) {
                //out << "i= " << i << "\t" << "   x[i]=  " << x[i] << "\n";
                outfile << "cnt= " << cnt << "\t" << "   xx[cnt]=  " << xx[cnt] << "\n";
              }
              for (cnt = 0; cnt<4; cnt++) {
                //out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
                outfile << "cnt= " << cnt << "\t" << "   yy[cnt]=  " << yy[cnt] << "\n";
              }

              break;
            }
            }

            if (cnt == 4) {
            for (i = 0; i<4; i++) {
              a[2 * i] = xx[i];      // 
              a[2 * i + 1] = yy[i]; //
            }
            psort(a); // 
            for (i = 0; i<4; i++) {
              xx[i] = a[2 * i];
              yy[i] = a[2 * i + 1];
              xx[i] -= 315; //这个应该是把图像平面坐标系移到左上角的坐标系
              yy[i] -= 240; 
             // xx[i] = -xx[i];
             // yy[i] = -yy[i];


            }

            for (i = 0; i<4; i++) {
              out <<  xx[i]   <<  "  \t  "  <<  yy[i]  << " \t ";
              
              if (i == 3) {
                out << " \n ";
              }
            }
            
          }
        

          



          // for (i = 0; i<4; i++) {
          // 	xx[i] *= 4 * 0.000003;
          // 	yy[i] *= 4 * 0.000003;
          // }

           for (i = 0; i<4; i++) {
             xx[i] *= 4 * 0.0000007;
             yy[i] *= 4 * 0.0000007;
           }

          Rx4 << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);

          Rx5 << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);

          R = Rx4*Rx5;

          tmph << 0, 0, 1; // 


          for (i = 0; i<4; i++) {
            tmpv_arr[i] << xx[i], yy[i], fl;
            mm[i] = fl / (tmph*R*tmpv_arr[i]);  // 
            p[i] = mm[i] * R*tmpv_arr[i]; // 
            u[i] = p[i](0); // 
            vv[i] = p[i](1);
          }

          for (i = 0; i<4; i++) {
            //out_V <<  " i=  " << i <<   "  u[i]=  "  << u[i] / 4 / 0.000003 <<   "    vv[i]=   "  << vv[i] / 4 / 0.000003 << "\t" << "\n";
                  out_V <<   u[i]  << " \t " << vv[i]  << " \t " ;
              if(i==3){
              out_V << " \n " ;
              }
          }
          
          m10 = u[0] + u[1] + u[2] + u[3];    m00 = 4;
          m01 = vv[0] + vv[1] + vv[2] + vv[3];
          ug = m10 / m00; vg = m01 / m00;

          u20 = (u[0] - ug)*(u[0] - ug) + (u[1] - ug)*(u[1] - ug)
            + (u[2] - ug)*(u[2] - ug) + (u[3] - ug)*(u[3] - ug);
          u02 = (vv[0] - vg)*(vv[0] - vg) + (vv[1] - vg)*(vv[1] - vg)
            + (vv[2] - vg)*(vv[2] - vg) + (vv[3] - vg)*(vv[3] - vg);
          u11 = (u[0] - ug)*(vv[0] - vg) + (u[1] - ug)*(vv[1] - vg)
            + (u[2] - ug)*(vv[2] - vg) + (u[3] - ug)*(vv[3] - vg);
          a1 = u20 + u02;

          qz = sqrt(ad / a1);

          double qzz = qz ;

          cout << " qz = " << qz << endl;

          if ( qz > 2.0 || qz < -0.7){
            qz = 0.5 ;
            cout << "qz >=1.5 || qz < -0.1  ==  " << qz << endl;
          }

            qx = qz*ug / fl; qy = qz*vg / fl;
            alpha = 0.5*atan((2 * u11) / (u20 - u02));
            q4 = alpha - alphad;

          ros::Time currentTime16 = ros::Time::now();
          string cur_qq = "/home/nuc/桌面/cur_qq.txt";//实际的图像据
          ofstream f_c_qqq;
          f_c_qqq.open(cur_qq.c_str(), ios::app);
          f_c_qqq << fixed;
          f_c_qqq << currentTime16 << setprecision(9) << " " << qx << " " << qy << " " << qz  << " " << qzz << endl;
          f_c_qqq.close();

          
          q << qx, qy, qz;
          tmpv << qxd, qyd, qzd;
          yawspeed = current_angular_velocity.rz;
          tmpv1 << 0, 0, yawspeed; // 
          q1 = q - tmpv;
      
          

          // Vector3f V(current_vel.x, current_vel.y, current_vel.z); //通过大疆获取速度消息
          // q2 = V / k1 - q1;   //速度V需要 通过大疆话题得到
          // q3 = f / (k1*k2) + q2;
          // df = -mass*kpp*k1*k1*k1*q1 -mass*kpp*k1*k1*k1*q2 - mass*k1*k2*k2*q3 - mass*k1*k1*k3*q3 - f.cross(tmpv1);
          // f(0) = f(0) + df(0)*dt;
          // f(1) = f(1) + df(1)*dt; 
          // f(2) = f(2) + df(2)*dt;
          

          voe = voe + dvoe*dtt;
          qe1 = qe1 + dqe1*dtt;
          qee = q1 - qe1;
          ve = voe - k6 * qee;
          dvoe = -tmpv1.cross(ve) +f-k5*k6*qee -qee/zd;
          dqe1 = -ve/zd-tmpv1.cross(q1)+k5*qee;
          q2 = q1 -ve/c1;
          f = qee/zd+c1*c2*q2+c1*c1*q2/zd;
          //f = qee/zd+c1*c2*q2;

          v11 = ve(0) + 0.1;
          v12 = -ve(1) + 0.15;
    
         // v13 = ve(2) + 0.2 ;
          v13 = ve(2) - current_vel.z;
          
          f11 = f(0) ;
          f12 = f(1) ;
          f13 = f(2) ;

          // roll = atan(f(0) / (f(2) - g));
          // pitch = atan((-cos(roll)*f(1))/ (f(2) - g));



          // ros::Time currentTime17 = ros::Time::now();
          // string cur_Euler1 = "/home/nuc/桌面/cur_Euler1.txt";//实际的欧拉角
          // ofstream f_c_E1;
          // f_c_E1.open(cur_Euler1.c_str(), ios::app);
          // f_c_E1 << fixed;
          // f_c_E1 << currentTime17 << setprecision(9) <<  " " << pitch << " " << roll << endl;
          // f_c_E1.close();

        

          // if (roll > 0.10) {
          // 	roll = 0.01;
          // 	cout << "roll > 0.17 此时的 roll  ==  " << roll << endl;

          // }

          // if (pitch > 0.10) {
          // 	pitch = 0.01;
          // 	cout << " pitch>0.15,此时的pitch  ==  " << pitch << endl;
          // }
          
          double yaw_cmd;        //yaw的角速率
          double yawspeedRate;
          //double yaw = yawAngle;       //当前的yaw（通过orb得到的）
          double yaw = yawAngle_vins;       //当前的yaw（通过vins得到的）
          //double yaw = yawAngle_imu; //当前的yaw（通过dji-imu得到的）

          // cout << "回调函数计算的偏航角为: " << yaw << endl;

          yaw_cmd = 2.8 * (0 - yaw); //期望的yaw为0,
         // yaw_cmd = 0 ; //期望的yaw为0,
          yawspeedRate = 1.5 *(0- yawspeed);

          Vector3f tmpv2;
          tmpv2 << 0, 0, 9.81;
          RowVector3f tmpv3;
          tmpv3 << cos(roll)*sin(pitch), -sin(roll), cos(roll)*cos(pitch);

          //Vector3f F;
          //F=mass*tmpv3*(tmpv2-f);
          Vector3f ff;
         // f(2) = 0.15*f(2) ;
          if(f(2) < 0){
            f13 = f(2);
          }
          if(f(2) > 0){
            f13 = 3.5*f(2);
          }
          ff << f(0) , -f(1) , f13;
          //ff << 0 , 0 , 0;
          double u1 = mass*tmpv3*(tmpv2 - ff);
         // double u1 = gra;
          double u11 = u1 ;

          roll = atan(f(0) / (f(2) - g));
          pitch = atan((-cos(roll)*f(1))/ (f(2) - g));

          ros::Time currentTime17 = ros::Time::now();
          string cur_Euler1 = "/home/nuc/桌面/cur_Euler1.txt";//实际的欧拉角
          ofstream f_c_E1;
          f_c_E1.open(cur_Euler1.c_str(), ios::app);
          f_c_E1 << fixed;
          f_c_E1 << currentTime17 << setprecision(9) <<  " " << roll << " " << pitch <<  " " << yaw <<  " " << yaw_cmd << " " << yawspeedRate <<endl;
          f_c_E1.close();

          if (roll > 0.15 || roll < -0.15) {
            roll = 0.01;
            cout << "roll > 0.17 此时的 roll  ==  " << roll << endl;

          }
          
          if (pitch > 0.15 || pitch < -0.15) {
            pitch = 0.01;
            cout << " pitch>0.15,此时的pitch  ==  " << pitch << endl;
          }
          


          ros::Time currentTime13 = ros::Time::now();
          string cur_ff = "/home/nuc/桌面/cur_ff.txt";// 实际的推力输出
          ofstream f_c_ff;
          f_c_ff.open(cur_ff.c_str(), ios::app);
          f_c_ff << fixed;
          f_c_ff << currentTime13 << setprecision(9) << " " << u1 << " " << f11 << " " << -f12 << " " << f13 << endl;
          f_c_ff.close();

          ros::Time currentTime18 = ros::Time::now();
          string cur_vv = "/home/nuc/桌面/cur_vv.txt";// 实际的推力输出
          ofstream f_c_vv;
          f_c_vv.open(cur_vv.c_str(), ios::app);
          f_c_vv << fixed;
          f_c_vv << currentTime18 << setprecision(9) << " " << v11 << " " << v12 << " " << v13 <<  endl;
          f_c_vv.close();

        if (u1 > 49)
        {
          u1 = 42.5;
          //yaw_cmd = 0;
          //roll = 0;
          //pitch = 0;
          cout << "推力大于50,强制变成"<< u1 << endl;
        }
        if( u1 < 42){
          
          u1 = 42.5;
          // yaw_cmd = 0;
          //roll = 0;
          // pitch = 0;
          cout << "推力小于40,强制变成"<< u1 << endl;
        }

        if(v11 > 0.5 || v11 < -0.5){

          v11 = 0.1;
        }
        if(v12 > 0.5 || v12 < -0.5){

          v12 = 0.1;
        }
        if(v13 > 0.7 || v13 < -0.4){

          v13 = -0.1;
        }
        //v12 = 0.25;
        //v12 = -1.7*current_vel.y ;

            sensor_msgs::Joy controlVelYawRate;
            uint8_t flag = (DJISDK::VERTICAL_THRUST |
            DJISDK::HORIZONTAL_VELOCITY |
            DJISDK::YAW_ANGLE |
            DJISDK::HORIZONTAL_BODY |
            DJISDK::STABLE_ENABLE);
            controlVelYawRate.axes.push_back(v11);
            controlVelYawRate.axes.push_back(v12);
            controlVelYawRate.axes.push_back(u1);
            controlVelYawRate.axes.push_back(yaw_cmd);
            controlVelYawRate.axes.push_back(flag);
            ctrlBrakePub.publish(controlVelYawRate);

                // sensor_msgs::Joy controlVelYawRate;
                // uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                //             DJISDK::HORIZONTAL_VELOCITY |
                //             DJISDK::YAW_RATE            |
                //             DJISDK::HORIZONTAL_BODY   |
                //             DJISDK::STABLE_ENABLE);
                // controlVelYawRate.axes.push_back(v11);
                // controlVelYawRate.axes.push_back(v12);
                // controlVelYawRate.axes.push_back(v13);
                // controlVelYawRate.axes.push_back(yaw_cmd);
                // controlVelYawRate.axes.push_back(flag);

                // ctrlBrakePub.publish(controlVelYawRate);

            
        
            // sensor_msgs::Joy controlThrustYawRate;
            // uint8_t flag = (DJISDK::VERTICAL_THRUST |
            // DJISDK::HORIZONTAL_ANGLE |
            // DJISDK::YAW_RATE |
            // DJISDK::HORIZONTAL_BODY |
            // DJISDK::STABLE_ENABLE);
            // controlThrustYawRate.axes.push_back(roll);
            // controlThrustYawRate.axes.push_back(pitch);
            // controlThrustYawRate.axes.push_back(u1);
            // controlThrustYawRate.axes.push_back(yaw_cmd);
            // controlThrustYawRate.axes.push_back(flag);

            // ctrlThrYawPub.publish(controlThrustYawRate);



          out_u << " u1= " << u1 <<" \t " << " u11= " << u11 << "\n";
          out_q << " qx= " << qx << " " << " qy= " << qy << " " << " qz= " << qz << " " << " q4= " << q4 << "\n";
          //out_q << " qy= " << qy << "\n";
          //out_q << " qz= " << qz << "\n";
              //out_q << " q4= " << q4 << "\n";
          out_E << " roll= " << roll << " " << " pitch= " << pitch << " " << " yaw= " << yaw << "\n";;
          //out_E << " pitch= " << pitch << "\n";
          //out_E << " yaw= " << yaw << "\n";
          //else if(u1<20){
          // u1=20;

          //}
          cout << "u1=  " << u1 << endl;
          cout << "roll=  " << roll << endl;
          cout << "pitch=  " << pitch << endl;
          cout << "yaw=  " << yaw << endl;
          cout << "yaw_cmd=  " << yaw_cmd << endl;
          cout << "u1=  " << u1 << endl;
          cout << "yawspeed= " << yawspeed << endl;
          cout << "qx=  " << qx << " qy= " << qy << " qz= " << qz << endl;
          cout << " x= " << vins_position.x << " y= " << vins_position.y << " z= " << vins_position.z << endl;
          //f_c_f << currentTime5 << setprecision(9) << " " << u1 << endl;
          ros::Time currentTime6 = ros::Time::now();
          string cur_f = "/home/nuc/桌面/cur_f.txt";//当前控制量输出
          ofstream f_c_f;
          f_c_f.open(cur_f.c_str(), ios::app);
          f_c_f << fixed;
          f_c_f << currentTime6 << setprecision(9) << " " << u1 << endl;
          f_c_f.close(); 

          ros::Time currentTime5 = ros::Time::now();
          string cur_Euler = "/home/nuc/桌面/cur_Euler.txt";//当前控制量输出
          ofstream f_c_E;
          f_c_E.open(cur_Euler.c_str(), ios::app);
          f_c_E << fixed;
          f_c_E << currentTime5 << setprecision(9) << " " << yaw << " " << yaw_cmd << " " << pitch << " " << roll << endl;
          f_c_E.close();
          

          ros::Time currentTime11 = ros::Time::now();
          string cur_q = "/home/nuc/桌面/cur_q.txt";//当前控制量输出
          ofstream f_c_qq;
          f_c_qq.open(cur_q.c_str(), ios::app);
          f_c_qq << fixed;
          f_c_qq << currentTime11 << setprecision(9) << " " << qx << " " << qy << " " << qz << " " <<  q1(0) << "" << q1(1) << " " << q1(2) << endl;
          f_c_qq.close();

          ros::Time currentTime14 = ros::Time::now();
          string cur_position1 = "/home/nuc/桌面/cur_position1.txt";//当前控制量输出
          ofstream f_c_pp;
          f_c_pp.open(cur_position1.c_str(), ios::app);
          f_c_pp << fixed;
          f_c_pp << currentTime14 << setprecision(9) << " " << vins_position.x << " " << vins_position.y<< " " << vins_position.z << endl;
          f_c_pp.close();

          ros::Time currentTime4 = ros::Time::now();
          string cur_control = "/home/nuc/桌面/日志记录.txt";
          ofstream f_c_control;
          f_c_control.open(cur_control.c_str(),ios::app);
          f_c_control << fixed;
          f_c_control << endl << "当前质量m = " << m*100 << endl << "期望图像据 = " <<  ad  << endl;
          f_c_control.close();

          ros::Time currentTime19 = ros::Time::now();
          string Virtual_Frame = "/home/nuc/桌面/Virtual_Frame.txt";
          ofstream f_V_F;
          f_V_F.open(Virtual_Frame.c_str(),ios::app);
          f_V_F << fixed;
          f_V_F << currentTime19 << setprecision(9) << " " <<  u[0] /4 /0.0000007 <<  " " <<  vv[0]/4 /0.0000007  <<  " " <<
          u[1] /4 /0.0000007 <<  " " <<  vv[1]/4 /0.0000007  <<  " " <<
          u[2] /4 /0.0000007 <<  " " <<  vv[2]/4 /0.0000007  <<  " " <<
          u[3] /4 /0.0000007 <<  " " <<  vv[3]/4 /0.0000007  << endl;
          f_V_F.close();

          // ros::Time currentTime19 = ros::Time::now();
          // string Virtual_Frame = "/home/nuc/桌面/Virtual_Frame.txt";
          // ofstream f_V_F;
          // f_V_F.open(Virtual_Frame.c_str(),ios::app);
          // f_V_F << fixed;
          // f_V_F << currentTime19 << setprecision(9) << " " <<  u[0]  <<  " " <<  vv[0]  <<  " " <<
          // u[1]  <<  " " <<  vv[1] <<  " " <<
          // u[2]  <<  " " <<  vv[2]  <<  " " <<
          // u[3]  <<  " " <<  vv[3]  << endl;
          // f_V_F.close();

         

          if(vins_position.z <= 1.82){
          zz_3 = zz_3 + 0.003;

          if(zz_3 <= 1.52){

            zz_3 = zz_3;

          }
          if(zz_3 > 1.52){

            zz_3 = 1.52;
            
          }
          d_x = 2.51;
          d_y = -0.25;
          d_z = zz_3;
          local_position_ctrl2();
          
        }
        if(vins_position.z > 1.82){
          d_x = 2.51;
          d_y = vins_position.y;
          d_z =  1.51;
          local_position_ctrl2();
        }
  
          }


              
          
      
      


     // delete pFrame;
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  double xcmd, ycmd, zcmd;
  sensor_msgs::Joy controlPosYaw;
  ////
  ///
  delete pFrame;
  //cvReleaseCapture(&pCapture);
  //cvReleaseVideoWriter(&writer);
    outfile.close();
	  out.close();
    out_V.close();
    out_q.close();
    out_u.close();
    out_E.close();
    //f_c_f.close();
    return 0;

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
              0 , 0.15 , 0 ,
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
  //yaw_cmd = 2.8 * ( 0 - yaw);
  yaw_cmd = 2.8 * ( 0 - yaw);


   cout << "偏航角的控制量为：" << yaw_cmd << endl;

  double fcmd = f * 100;
  double rcmd = eulerAngle(2);
  double pcmd = eulerAngle(1);
  cout << "横滚角的控制量为：" << rcmd << endl;
  cout << "俯仰角的控制量为：" << pcmd << endl;
  cout << "计算的推力="<< fcmd << endl;
  f11 = F(0) ;
  f12 = F(1) ;
  f13 = F(2) ;

  ros::Time currentTime13 = ros::Time::now();
  string cur_ff = "/home/nuc/桌面/cur_ff.txt";//当前控制量输出
  ofstream f_c_ff;
  f_c_ff.open(cur_ff.c_str(), ios::app);
  f_c_ff << fixed;
  f_c_ff << currentTime13 << setprecision(9) << " " << fcmd << " " << f11 << " " << f12 << " " << f13 << endl;
  f_c_ff.close();
  //炸机时悬停
  if (fcmd > 50)
  {
    // if (fcmd > 60){

    // fcmd = 37;
    // yaw_cmd = 0;
    // rcmd = 0;
    // pcmd = 0;
    // cout << "推力大于60,强制变成"<< fcmd << endl;

    // }
    fcmd = 42;
    yaw_cmd = 0;
    rcmd = 0;
    pcmd = 0;
    cout << "推力大于50,强制变成"<< fcmd << endl;
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

  //delete pFrame;

  
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
  f_c_f << currentTime6 << setprecision(9) <<" "<< fcmd << endl;
  f_c_f.close();

  ros::Time currentTime7 = ros::Time::now();
  string planning_pose = "/home/nuc/桌面/desire_position.txt";//规划位姿
  ofstream f_p_p;
  f_p_p.open(planning_pose.c_str(),ios::app);
  f_p_p << fixed;
  f_p_p << currentTime7 << setprecision(9) <<" "<< d_x << " " << d_y << " " << d_z << endl;
  f_p_p.close();

 // cvWaitKey(1);

}

void local_position_ctrl2() 
{
   
   
  //  if(vins_position.z <= 1.52){
  //    zz_3 = zz_3 + 0.003;

  //    if(zz_3 <= 1.52){

  //      zz_3 = zz_3;

  //    }
  //    if(zz_3 > 1.52){

  //      zz_3 = 1.52;

  //    }
  //    d_x = 2.51;
  //    d_y = vins_position.y;
  //    d_z = zz_3;
     
  //  }
  //  if(vins_position.z > 1.52){
  //    d_x = 2.51;
  //    d_y = vins_position.y;
  //    d_z =  1.51;
  //  }
  

    

  //cout << "now used local_position_ctrl2" << endl;
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
/*  k_p << 0.09,   0 , 0 ,
           0 , 0.09, 0 ,
           0 ,   0 , 0.1;
*/
     k_p << 0.10,   0 , 0 ,
              0 , 0.09 , 0 ,
              0 ,   0 , 0.10 ;
  Eigen::Matrix3d k_v;
/*  k_v << 0.09, 0, 0,
      0, 0.09, 0,
      0, 0, 0.12;
*/
 k_v << 0.09, 0, 0,
      0, 0.09, 0,
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
  //yaw_cmd = 2.8 * ( 0 - yaw);
  yaw_cmd = 0;


   //cout << "偏航角的控制量为：" << yaw_cmd << endl;

  double fcmd = f * 100;
  double rcmd = eulerAngle(2);
  double pcmd = eulerAngle(1);
  cout << "横滚角的控制量为：" << rcmd << endl;
  cout << "俯仰角的控制量为：" << pcmd << endl;

  //cout << "计算的推力="<< fcmd << endl;
  //炸机时悬停
  if (fcmd > 50)
  {
    fcmd = 42;
    yaw_cmd = 0;
    
    cout << "推力大于50,强制变成"<< fcmd << endl;
  }
  cout << "实际的推力="<< fcmd << endl;
 //fcmd = 20;


  
//cout<<"最后发给控制器的fcmd = "<< fcmd_out << endl;

  sensor_msgs::Joy controlThrustYawRate;
  uint8_t flag = (DJISDK::VERTICAL_THRUST |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_ANGLE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);
  controlThrustYawRate.axes.push_back(rcmd);
  controlThrustYawRate.axes.push_back(pcmd);
  controlThrustYawRate.axes.push_back(fcmd);
  controlThrustYawRate.axes.push_back(yaw_cmd);
  controlThrustYawRate.axes.push_back(flag);

  ctrlThrYawPub.publish(controlThrustYawRate);

 

}

void local_position_ctrl3() 
{
//     cout << "now used local_position_ctrl3" << endl;

//     ofstream outfile("/home/nuc/桌面/圆的中心坐标.txt");
// 	  ofstream out("/home/nuc/桌面/真实平面下的像素坐标.txt");
//     ofstream out_q("/home/nuc/图像距.txt");
//     ofstream out_u("/home/nuc/桌面/推力.txt");
//     ofstream out_E("/home/nuc/桌面/欧拉角.txt");

  
//     int cnt = 0;
		
// 		for (c = seq; c!= NULL; c = c->h_next) {  
// 			rc = cvBoundingRect(c, 0); 
// 			if (rc.width > 65 && rc.width < 89  &&  rc.height > 65 && rc.height < 89) {
// 				//cout << "rc.width=  " << rc.width << endl;

// 				xx[cnt] = rc.x + rc.width / 2;  
// 				//cout << "rc.x=  " << rc.x << endl;
// 				//cout << "rc.width" << rc.width << endl;
// 				//cout << "x[cnt]=   " <<x[cnt]  <<  "   cnt=  "<<  cnt << endl;
// 				yy[cnt] = rc.y + rc.height / 2;
// 				//cout << "y[cnt]=   " << y[cnt] << "   cnt=  " <<  cnt << endl;
// 			  //cout << "rc.y=   " << rc.y << endl;
// 				//cout << "rc.height=  " << rc.height << endl;


// 				//cvDrawRect(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));
// 				cvRectangle(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));
// 				cvShowImage("jiance", pFrame);

//         //CvVideoWriter *writer=cvCreateVideoWriter("home/nuc/桌面/video.avi",CV_FOURCC('M','J','P','G'),5,cvGetSize(pFrame));

// 				//Mat mmm = cvarrToMat(pFrame, true);
//         // Mat cvarrToMat(const CvArr* arr, bool copyData=false,
//         //               bool allowND=true, int coiMode=0,
//         //               AutoBuffer* buf=0);


//         // Mat mmm = cvarrToMat(pFrame);
//         // write << mmm;

// 				cnt++; 
// 				//cout << "cnt=  " << cnt << endl;
				
// 			}

			
// 			if (cnt == 4) {

// 				for (cnt=0; cnt<4; cnt++) {
// 					//out << "i= " << i << "\t" << "   x[i]=  " << x[i] << "\n";
// 					outfile << "cnt= " << cnt << "\t" << "   xx[cnt]=  " << xx[cnt] << "\n";
// 				}
// 				for (cnt = 0; cnt<4; cnt++) {
// 					//out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
// 					outfile << "cnt= " << cnt << "\t" << "   yy[cnt]=  " << yy[cnt] << "\n";
// 				}
				
// 				break;
// 			}
// 		}

// 		if (cnt == 4) {
// 			for (i = 0; i<4; i++) {
// 				a[2 * i] = xx[i];      // 
// 				a[2 * i + 1] = yy[i]; //
// 			}
// 			psort(a); // 
// 			for (i = 0; i<4; i++) {
// 				xx[i] = a[2 * i];
// 				yy[i] = a[2 * i + 1];
// 				//x[0] = x[0] + 341 - 290 - 74 - 37;
// 				//cout << "x[0]=  " << x[0] << endl;
// 				xx[i] -= 160; //这个应该是把图像平面坐标系移到左上角的坐标系
// 				yy[i] -= 128; //改成自己的
// 				//x[i] = -x[i];
// 				//y[i] = -y[i];


// 			}

// 			for (i = 0; i<4; i++) {
// 				out << "i= " << i << "\t" << "   xx[i]=  " << xx[i] << "\n";
// 			}
// 			for (i = 0; i<4; i++) {
// 				out << "i=  " << i << "\t" << "  yy[i]=  " << yy[i] << "\n";
// 			}
// 		}
// 		else {
// 			//isfirst = true;
// 			reason = 1;
// 		}
		
// 		/*for (i = 0; i<4; i++) {
// 			out <<"i= "<< i << "\t" <<  "   x[i]=  "<< x[i] << "\n";
// 		}
// 		for (i = 0; i<4; i++) {
// 			out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
// 		}*/
		
// 	    cvWaitKey(100);


  
//   	for (i = 0; i<4; i++) {
// 			xx[i] *= 4 * 0.000003; 
// 			yy[i] *= 4 * 0.000003;
// 	  }

// 		Rx4 << 1, 0, 0,
// 			     0, cos(roll), -sin(roll),
// 			     0, sin(roll), cos(roll);

// 		Rx5 << cos(pitch), 0, sin(pitch),
// 			     0,          1,   0,
// 			     -sin(pitch), 0, cos(pitch);

// 		R = Rx4*Rx5;

// 		tmph << 0, 0, 1; // 

						 
// 		for (i = 0; i<4; i++) {
// 			tmpv_arr[i] << xx[i], yy[i], fl;
// 			mm[i] = fl / (tmph*R*tmpv_arr[i]);  // 
// 			p[i] = mm[i] * R*tmpv_arr[i]; // 
// 			u[i] = p[i](0); // 
// 			vv[i] = p[i](1);
// 		}

// 		// for (i = 0; i<4; i++) {
// 		// 	out_V <<  "i= " << i <<   "   u[i]= "  << u[i] / 4 / 0.000003 << "\n";
// 		// }
// 		// for (i = 0; i<4; i++) {
// 		// 	out_V << "i= " << i <<  "    vv[i]=  "  << vv[i] / 4 / 0.000003 << "\n";
// 		// }

// 		//
// 		m10 = u[0] + u[1] + u[2] + u[3];    m00 = 4;
// 		m01 = vv[0] + vv[1] + vv[2] + vv[3];
// 		ug = m10 / m00; vg = m01 / m00;

// 		u20 = (u[0] - ug)*(u[0] - ug) + (u[1] - ug)*(u[1] - ug)
// 			+ (u[2] - ug)*(u[2] - ug) + (u[3] - ug)*(u[3] - ug);
// 		u02 = (vv[0] - vg)*(vv[0] - vg) + (vv[1] - vg)*(vv[1] - vg)
// 			+ (vv[2] - vg)*(vv[2] - vg) + (vv[3] - vg)*(vv[3] - vg);
// 		u11 = (u[0] - ug)*(vv[0] - vg) + (u[1] - ug)*(vv[1] - vg)
// 			+ (u[2] - ug)*(vv[2] - vg) + (u[3] - ug)*(vv[3] - vg);
// 		a1 = u20 + u02;

// 		if (ad / a1<5) {
// 			qz = sqrt(ad / a1);
// 		}
// 		else {
// 			qz = sqrt(5); // 
// 		}

// 		qx = qz*ug / fl; qy = qz*vg / fl;
// 		alpha = 0.5*atan((2 * u11) / (u20 - u02));
// 		q4 = alpha - alphad;

// 		q << qx, qy, qz;
// 		tmpv << qxd, qyd, qzd;
// 		tmpv1 << 0, 0, yawspeed; // 需要解决 通过大疆话题读取到

// 		//
// 		q1 = q - tmpv;

//     Vector3f V(current_vel.x, current_vel.y, current_vel.z); //通过大疆获取速度消息
// 		q2 = V/k1 - q1;   //速度V需要 通过大疆话题得到
// 		q3 = f / (k1*k2) + q2;
// 		df = -kpp*k1*k1*k1*q1-kpp*k1*k1*k1*q2 - k1*k2*k2*q3 - k1*k1*k3*q3 - f.cross(tmpv1);
// 		f(1) = f(1) + df(1)*dt; f(2) = f(2) + df(2)*dt; f(3) = f(3) + df(3)*dt;
// 		roll= atan(f(1) / (f(3) - g));
// 		pitch= atan(-(cos(roll)*f(2) / (f(3) - g)));
// 		//dx6 = -k4*q4;
// 		//yaw = yaw + dx6*dt;
// 	  double yaw_cmd;        //yaw的角速率
//        //double yaw = yawAngle;       //当前的yaw（通过orb得到的）
//        //double yaw = yawAngle_vins;       //当前的yaw（通过vins得到的）
//     double yaw = yawAngle_imu; //当前的yaw（通过dji-imu得到的）

//        // cout << "回调函数计算的偏航角为: " << yaw << endl;

//     yaw_cmd = 2.8 * (0 - yaw); //期望的yaw为0,

// 	   Vector3f tmpv2;
// 	   tmpv2 << 0,0,9.81;
// 	   RowVector3f tmpv3;
// 	   tmpv3 << cos(roll)*sin(pitch),-sin(roll),cos(roll)*cos(pitch);

// 	   //Vector3f F;
// 	   //F=mass*tmpv3*(tmpv2-f);
// 	   double u1=mass*tmpv3*(tmpv2-f);

// 	   if(u1>55){
// 		   u1=40;
//        yaw_cmd =0;
//        roll =0;
//        pitch =0;
//        cout<<"推力大于55,强制变为 "<< u1 <<endl;

// 	   }

//        out_u << " u1= " << u1 << "\n";
//        out_q << " qx= " << qx << "\n";
//        out_q << " qy= " << qy << "\n";
//        out_q << " qz= " << qz << "\n";
//        out_E << " roll= " << roll << "\n";
//        out_E << " pitch= " << pitch << "\n";
//        out_E << " yaw= " << yaw << "\n";
//      //else if(u1<20){
// 		  // u1=20;

// 	   //}
// 	   cout<<"u1=  "<< u1 <<endl;
// 	   cout<<"roll=  "<< roll <<endl;
// 	   cout<<"pitch=  "<< pitch <<endl;
// 	   cout<<"yaw=  "<< yaw <<endl;
// 	   cout<<"yaw_cmd=  "<< yaw_cmd <<endl;
// 	   cout<<"u1=  "<< u1 << endl;

       
// 	   sensor_msgs::Joy controlThrustYawRate;
//      uint8_t flag = (DJISDK::VERTICAL_THRUST |
//                   DJISDK::HORIZONTAL_ANGLE |
//                   DJISDK::YAW_RATE |
//                   DJISDK::HORIZONTAL_BODY |
//                   DJISDK::STABLE_ENABLE);
//      controlThrustYawRate.axes.push_back(roll);
//      controlThrustYawRate.axes.push_back(pitch);
//      controlThrustYawRate.axes.push_back(u1);
//      controlThrustYawRate.axes.push_back(yaw_cmd);
//      controlThrustYawRate.axes.push_back(flag);
//   /*
//   float roll      = pMsg->axes[0];
//   float pitch     = pMsg->axes[1];
//   float pz        = pMsg->axes[2];
//   float yawRate   = pMsg->axes[3];
// */
//      ctrlThrYawPub.publish(controlThrustYawRate);






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
  f_v_p << currentTime8 << setprecision(9) <<" "<< vins_position.x  << " " << vins_position.y  << " " << vins_position.z << endl;
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

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) //myself
	{

	
		//  rx = msg->angular_velocity.x;
		//  ry = msg->angular_velocity.y;
		//  rz = msg->angular_velocity.z;
		// Eigen::Vector3d acc(rx, ry, rz);

     current_angular_velocity.rx = msg->angular_velocity.x;
		 current_angular_velocity.ry = msg->angular_velocity.y;
		 current_angular_velocity.rz = msg->angular_velocity.z;


		// cout << "当前x方向的机体角速度为" << rx << endl;
		// cout << "当前y方向的机体角速度为" << ry << endl;
		// cout << "当前z方向的机体角速度为" << rz << endl;

		ros::Time currentTime10 = ros::Time::now();
		string cur_imu = "/home/nuc/桌面/cur_imu.txt";//当前位姿
		ofstream f_c_v;
		f_c_v.open(cur_imu.c_str(), ios::app);
		f_c_v << fixed;
		f_c_v << currentTime10 << setprecision(9) << " " << current_angular_velocity.rx << " " <<current_angular_velocity.ry << " " << current_angular_velocity.rz << endl;
		f_c_v.close();


	}






