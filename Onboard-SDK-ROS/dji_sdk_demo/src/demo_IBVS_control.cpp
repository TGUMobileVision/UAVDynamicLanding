#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
//#include <iostream>
//#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
#include <nav_msgs/Odometry.h>
#include <sstream>

using namespace cv;
using namespace std;
using namespace Eigen;

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

current_position_def current_position, current_vel,tag_position,vins_position,planning_position;

geometry_msgs::Quaternion current_atti;//mocap 提供的姿态



double yawAngle;
double yawAngle_imu;
double yawAngle_vins;
double fcmd;
double x, y;

int flag_a = 0;

int i;
float roll, pitch, yaw, yawspeed, quat[4];
float rolld = 0, pitchd = 0, roll1 = 0, roll2 = 0, pitch1 = 0, pitch2 = 0;
//int i;
float hd, z, v;

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

				//cout << "a[j * 2]=" << a[j * 2] << endl;
				//cout << "a[j * 2 + 1]=" << a[j * 2 + 1] << endl;
				//cout << "a[j * 2 + 2]=" << a[j * 2 + 2] << endl;
				//cout << "a[j * 2 + 3]=" << a[j * 2 + 3] << endl;

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


Mat img; Mat templ; Mat result;
const char* image_window = "tuxiang";
const char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;
void MatchingMethod(int, void*);

int main(int argc, char ** argv) {

	ros::init(argc, argv, "demo_IBVS_control_node");
  ros::NodeHandle nh;

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



	//bool isfirst = true;
	int reason = 0;
	float fl = 0.0036, ad = 0.00000045, qzd = 1, qxd = 0, qyd = 0, alphad = 0, dt = 0.01,
		zd = 1.5, c1 = 1, k1 = 4, k2 = 4, c2 = 2, mass = 1.56, gra = mass*9.81 , g = 9.81, k3 =2, k4 =2, kpp =0.25, dx6;
	
	float rp_thr = 0.173, thr_sp = 0.660, r_err = 0, p_err = 0;
	//0.17365;  
	float m[4], u[4], v[4];
	float m10, m00, m01, ug, vg, u20, u02, u11, a1, qz, qx, qy, u1, alpha, q4;
	/***************************************
	*
	* PID Variables
	*
	***************************************/
	float pE = 0, iE = 0, dE = 0, ek = 0, ek1 = 0, ek2 = 0;
	float kp = 0, ki = 0, kd = 0;
	//Matrix and vectors
	Matrix3d Rx4, Rx5, R;  // 
  Vector3d tmpv_arr[4] ;
	Vector3d p[4],  q, q1, tmpv,tmpv1, voe, V, dvoe, qe1, dqe1, qee, ve, q2, q3, df, f; // 
	RowVector3d tmph; // 

	//templ = imread("G:\\2\\muban6.jpg", 1);    //

	            
	//ofstream outfile("home/nuc/桌面/圆的中心坐标.txt");
	//ofstream out("home/nuc/桌面/真实平面下的像素坐标.txt");
  ofstream outfile("/home/nuc/桌面/圆的中心坐标.txt");
	ofstream out("/home/nuc/桌面/真实平面下的像素坐标.txt");
  ofstream out_q("/home/nuc/桌面/图像距.txt");
  ofstream out_u("/home/nuc/桌面/推力.txt");
  ofstream out_E("/home/nuc/桌面/欧拉角.txt");
	bool isfirst = true;
	//int reason = 0;
	
	Mat Frame;

	
	VideoCapture pCapture(5);

	if (pCapture.isOpened()) {
		cout << "open the camera" << endl;
		// return -1;
	}
	else
	{
		cout << "can not the camera" << endl;
		return -1;
	}
	//VideoCapture cap;
	//cap.open(1);

	VideoWriter write;
	write.open("home/nuc/桌面/1.avi", CAP_ANY, 30.0, Size(pCapture.get(CAP_PROP_FRAME_WIDTH), pCapture.get(CAP_PROP_FRAME_HEIGHT)));

	if (!write.isOpened())
	{
		cout << "can not write" << endl;
		//return -1;
	}

	
	double frame_width=0;
	double frame_height=0;
	double frame_fps = 0;
	frame_width=pCapture.get(CV_CAP_PROP_FRAME_WIDTH);
	frame_height=pCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
	frame_fps=pCapture.get(CV_CAP_PROP_FPS);

	cout << "frame_width=" << frame_width << endl;
	cout << "frame_height=" << frame_height << endl;
	cout << "frame_fps=" << frame_fps << endl;

	pCapture >> Frame;


	IplImage*  pFrame = new IplImage(Frame);
	
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

	CvMemStorage *storage = cvCreateMemStorage();
	CvSeq *seq = NULL;
	CvSeq *c = NULL;
	int cnt, piccnt = 0;
	char buffer[20];
	float x[4], y[4], a[8];

	CvScalar cs;
	CvRect rc;
	 
  if (my_takeoff(1)) //启动电机
  {
    ros::Rate loop_rate(50);
	  while (ros::ok()) {
		
		pCapture >> Frame;

		IplImage*  pFrame = new IplImage(Frame);
		
		if (!pFrame) {
			cout << "can not read pFrame" << endl;
			break;
			//return -1;
		}

		cvShowImage("xiangshi", pFrame);
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
		cvFindContours(hImg, storage, &seq, sizeof(CvContour), CV_RETR_EXTERNAL);


		int cnt = 0;
		
		for (c = seq; c!= NULL; c = c->h_next) {  
			rc = cvBoundingRect(c, 0); 
			if (rc.width > 65 && rc.width < 89  &&  rc.height > 65 && rc.height < 89) {
				//cout << "rc.width=  " << rc.width << endl;

				x[cnt] = rc.x + rc.width / 2;  
				//cout << "rc.x=  " << rc.x << endl;
				//cout << "rc.width" << rc.width << endl;
				//cout << "x[cnt]=   " <<x[cnt]  <<  "   cnt=  "<<  cnt << endl;
				y[cnt] = rc.y + rc.height / 2;
				//cout << "y[cnt]=   " << y[cnt] << "   cnt=  " <<  cnt << endl;
			  //cout << "rc.y=   " << rc.y << endl;
				//cout << "rc.height=  " << rc.height << endl;


				//cvDrawRect(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));
				cvRectangle(pFrame, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(0, 0, 255));
				cvShowImage("jiance", pFrame);

				Mat m = cvarrToMat(pFrame, true);
				write << m;

				cnt++; 
				cout << "cnt=  " << cnt << endl;
				
			}

			
			if (cnt == 4) {

				for (cnt=0; cnt<4; cnt++) {
					//out << "i= " << i << "\t" << "   x[i]=  " << x[i] << "\n";
					outfile << "cnt= " << cnt << "\t" << "   x[cnt]=  " << x[cnt] << "\n";
				}
				for (cnt = 0; cnt<4; cnt++) {
					//out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
					outfile << "cnt= " << cnt << "\t" << "   y[cnt]=  " << y[cnt] << "\n";
				}
				
				break;
			}
		}

		if (cnt == 4) {
			for (i = 0; i<4; i++) {
				a[2 * i] = x[i];      // 
				a[2 * i + 1] = y[i]; //
			}
			psort(a); // 
			for (i = 0; i<4; i++) {
				x[i] = a[2 * i];
				y[i] = a[2 * i + 1];
				//x[0] = x[0] + 341 - 290 - 74 - 37;
				//cout << "x[0]=  " << x[0] << endl;
				x[i] -= 160; //这个应该是把图像平面坐标系移到左上角的坐标系
				y[i] -= 128;
				//x[i] = -x[i];
				//y[i] = -y[i];


			}

			for (i = 0; i<4; i++) {
				out << "i= " << i << "\t" << "   x[i]=  " << x[i] << "\n";
			}
			for (i = 0; i<4; i++) {
				out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
			}
		}
		else {
			isfirst = true;
			reason = 1;
		}
		
		/*for (i = 0; i<4; i++) {
			out <<"i= "<< i << "\t" <<  "   x[i]=  "<< x[i] << "\n";
		}
		for (i = 0; i<4; i++) {
			out << "i=  " << i << "\t" << "  y[i]=  " << y[i] << "\n";
		}*/
		
	    cvWaitKey(100);
		
		for (i = 0; i<4; i++) {
			x[i] *= 4 * 0.000003; 
			y[i] *= 4 * 0.000003;
	  }

		Rx4 << 1, 0, 0,
			     0, cos(roll), -sin(roll),
			     0, sin(roll), cos(roll);

		Rx5 << cos(pitch), 0, sin(pitch),
			     0,          1,   0,
			     -sin(pitch), 0, cos(pitch);

		R = Rx4*Rx5;

		tmph << 0, 0, 1; // 

						 
		for (i = 0; i<4; i++) {
			tmpv_arr[i] << x[i], y[i], fl;
			m[i] = fl / (tmph*R*tmpv_arr[i]);  // 
			p[i] = m[i] * R*tmpv_arr[i]; // 
			u[i] = p[i](0); // 
			v[i] = p[i](1);
		}

		for (i = 0; i<4; i++) {
			outfile << u[i] / 4 / 0.000003 << "\t";
		}
		for (i = 0; i<4; i++) {
			outfile << v[i] / 4 / 0.000003 << "\t";
		}

		//
		m10 = u[0] + u[1] + u[2] + u[3];    m00 = 4;
		m01 = v[0] + v[1] + v[2] + v[3];
		ug = m10 / m00; vg = m01 / m00;

		u20 = (u[0] - ug)*(u[0] - ug) + (u[1] - ug)*(u[1] - ug)
			+ (u[2] - ug)*(u[2] - ug) + (u[3] - ug)*(u[3] - ug);
		u02 = (v[0] - vg)*(v[0] - vg) + (v[1] - vg)*(v[1] - vg)
			+ (v[2] - vg)*(v[2] - vg) + (v[3] - vg)*(v[3] - vg);
		u11 = (u[0] - ug)*(v[0] - vg) + (u[1] - ug)*(v[1] - vg)
			+ (u[2] - ug)*(v[2] - vg) + (u[3] - ug)*(v[3] - vg);
		a1 = u20 + u02;

		if (ad / a1<5) {
			qz = sqrt(ad / a1);
		}
		else {
			qz = sqrt(5); // 
		}

		qx = qz*ug / fl; qy = qz*vg / fl;
		alpha = 0.5*atan((2 * u11) / (u20 - u02));
		q4 = alpha - alphad;

    out_q << " qx= " << qx << "\n" ;
    out_q << " qy= " << qy << "\n" ;
    out_q<< " qz= " << qz << "\n" ;

		q << qx, qy, qz;
		tmpv << qxd, qyd, qzd;
		tmpv1 << 0, 0, yawspeed; // 需要解决 通过大疆话题读取到

		//
		q1 = q - tmpv;

    //f << f(1),f(2),f(3);
    //df << df(1),df(2),df(3);

    Vector3d v1(current_vel.x, current_vel.y, current_vel.z); //通过大疆获取速度消息
		q2 = v1/k1 - q1;   //速度V需要 通过大疆话题得到
		q3 = f / (k1*k2) + q2;
		df = -kpp*k1*k1*k1*q1-kpp*k1*k1*k1*q2 - k1*k2*k2*q3 - k1*k1*k3*q3 - f.cross(tmpv1);
    //double f(1) , f(2) , f(3);
		f(0) = f(0) + df(0)*dt; f(1) = f(1) + df(1)*dt; f(2) = f(2) + df(2)*dt;
    //roll=0.12;
    //pitch=0.12;
		roll= atan(f(0) / (f(2) - g));
		pitch= atan(-(cos(roll)*f(1) / (f(2) - g)));
		//dx6 = -k4*q4;
		//yaw = yaw + dx6*dt;
	  double yaw_cmd;        //yaw的角速率
       //double yaw = yawAngle;       //当前的yaw（通过orb得到的）
       //double yaw = yawAngle_vins;       //当前的yaw（通过vins得到的）
    double yaw = yawAngle_imu; //当前的yaw（通过dji-imu得到的）

       // cout << "回调函数计算的偏航角为: " << yaw << endl;

    yaw_cmd = 2.8 * (0 - yaw); //期望的yaw为0,

	   Vector3d tmpv2;
	   tmpv2 << 0,0,9.81;
	   RowVector3d tmpv3;
	   tmpv3 << cos(roll)*sin(pitch),-sin(roll),cos(roll)*cos(pitch);

	   //Vector3f F;
	   //F=mass*tmpv3*(tmpv2-f);
	   double u1=mass*tmpv3*(tmpv2-f);

	   if(u1>60){
		   u1=40;
       yaw_cmd =0;
       roll =0;
       pitch =0;
       cout<<"推力大于60,强制变为 "<< u1 <<endl;

	   }
     out_u << " u= " << u1 << "\n" ;
     //else if(u1<20){
		  // u1=20;

	   //}
	   cout<<"u1=  "<< u1 <<endl;
	   cout<<"roll=  "<< roll <<endl;
	   cout<<"pitch=  "<< pitch <<endl;
	   cout<<"yaw=  "<< yaw <<endl;
	   cout<<"yaw_cmd=  "<< yaw_cmd <<endl;
	   cout<<"u1=  "<< u1 << endl;

       
	   sensor_msgs::Joy controlThrustYawRate;
     uint8_t flag = (DJISDK::VERTICAL_THRUST |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);
     controlThrustYawRate.axes.push_back(roll);
     controlThrustYawRate.axes.push_back(pitch);
     controlThrustYawRate.axes.push_back(u1);
     controlThrustYawRate.axes.push_back(yaw_cmd);
     controlThrustYawRate.axes.push_back(flag);
  /*
  float roll      = pMsg->axes[0];
  float pitch     = pMsg->axes[1];
  float pz        = pMsg->axes[2];
  float yawRate   = pMsg->axes[3];
*/
     ctrlThrYawPub.publish(controlThrustYawRate);







    loop_rate.sleep();
    ros::spinOnce();
	}


  }


    outfile.close();
	  out.close();
    out_E.close();
    out_q.close();
    out_u.close();

    
	return 0;
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


//以下为mocap实验条件下用的位姿和速度 运动捕捉系统下使用的
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

  vins_position.x = msg->pose.position.x;//提供当前位置
  vins_position.y = msg->pose.position.y;
  vins_position.z = msg->pose.position.z;

  ros::Time currentTime8 = ros::Time::now();
  string vins_pose = "vins_pose.txt";//当前位姿
  ofstream f_v_p;
  f_v_p.open(vins_pose.c_str(),ios::app);
  f_v_p << fixed;
  f_v_p << currentTime8 << setprecision(9) <<" "<< vins_position.x << " " << vins_position.y << " " << vins_position.z << endl;
  f_v_p.close();


  current_atti = msg->pose.orientation; //提供当前姿态
  yawAngle_vins = toEulerAngle(current_atti).z;

  ros::Time currentTime81 = ros::Time::now();
  string cur_yaw = "cur_yaw.txt";//当前位姿
  ofstream f_c_yaw;
  f_c_yaw.open(cur_yaw.c_str(),ios::app);
  f_c_yaw << fixed;
  f_c_yaw << currentTime81 << setprecision(9) <<" "<< yawAngle  << endl;
  f_c_yaw.close();

  // cout << "回调函数计算的偏航角为 " << yawAngle << endl;

  cout << "pose_by_vins_callback 回调函数de flag_a = " << flag_a << endl;
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
  string cur_vel = "cur_vel.txt";//当前位姿
  ofstream f_c_v;
  f_c_v.open(cur_vel.c_str(),ios::app);
  f_c_v << fixed;
  f_c_v << currentTime9 << setprecision(9) <<" "<< current_vel.x  << " " << current_vel.y << " " << current_vel.z <<endl;
  f_c_v.close();


}


void tag_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
 // flight_status = msg->data;
  tag_position.x_fu = msg -> pose.pose.position.z;
  tag_position.x = - tag_position.x_fu;
  tag_position.y = msg -> pose.pose.position.y;
  tag_position.z = msg -> pose.pose.position.x;
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

  ros::Time currentTime8 = ros::Time::now();
  string planning_pose = "/home/acl/桌面/planning_pose.txt";//当前位姿
  ofstream f_p_p;
  f_p_p.open(planning_pose.c_str(),ios::app);
  f_p_p << fixed;
  f_p_p << currentTime8 << setprecision(9) <<" "<< planning_position.x << " " << planning_position.y << " " << planning_position.z << endl;
  f_p_p.close();

/*
  current_atti = msg->pose.orientation; //提供当前姿态
  yawAngle_vins = toEulerAngle(current_atti).z;

  ros::Time currentTime81 = ros::Time::now();
  string cur_yaw = "cur_yaw.txt";//当前位姿
  ofstream f_c_yaw;
  f_c_yaw.open(cur_yaw.c_str(),ios::app);
  f_c_yaw << fixed;
  f_c_yaw << currentTime81 << setprecision(9) <<" "<< yawAngle  << endl;
  f_c_yaw.close();

  // cout << "回调函数计算的偏航角为 " << yawAngle << endl;
*/
  flag_a = 1;
  cout << "planning_pos_callback 回调函数de flag_a = " << flag_a << endl;

}

