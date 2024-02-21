/**
 * 该例程将发布/move_base_simple/goal话题，消息类型geometry_msgs/PoseStamped
 */
 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  //geometry_msgs/PoseStamped类型的消息对应的头文件是geometry_msgs/PoseStamped.h

int main(int argc, char **argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "my_target");

	// 创建节点句柄
	ros::NodeHandle nh;

	// 创建一个Publisher，发布名为/move_base_simple/goal的topic，消息类型为geometry_msgs::PoseStamped，队列长度1000

  	ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

	// 设置循环的频率
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
	    int i = 0;
		if (i == 0)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 0;
			target_msg.pose.position.y = 0;
			target_msg.pose.position.z = 0.5;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(0.3).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);
			
	
		}

		if (i == 1)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 0.6;
			target_msg.pose.position.y = 0.6;
			target_msg.pose.position.z = 0.5;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(3).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);
			
		}

		if (i == 2)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 0.6;
			target_msg.pose.position.y = 0.6;
			target_msg.pose.position.z = 0;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(2).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);
			break;
		}
/*
		if (i == 3)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 12;
			target_msg.pose.position.y = 0;
			target_msg.pose.position.z = 0;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(5).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);

		}

		if (i == 4)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 14;
			target_msg.pose.position.y = 0;
			target_msg.pose.position.z = 0;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(2.1).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);

		}

		if (i == 5)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 14;
			target_msg.pose.position.y = 0;
			target_msg.pose.position.z = 0;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(13).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);

		}

		if (i == 6)
		{
			// 初始化geometry_msgs::PoseStamped类型的消息
			geometry_msgs::PoseStamped target_msg;

			target_msg.header.stamp = ros::Time::now();
			target_msg.header.frame_id = "world" ;

			target_msg.pose.position.x = 11;
			target_msg.pose.position.y = 0;
			target_msg.pose.position.z = 0;

			target_msg.pose.orientation.x = 0.5;
			target_msg.pose.orientation.y = 0.6;
			target_msg.pose.orientation.z = 0.9;
			target_msg.pose.orientation.w = 1.5;
			
			// 发布消息
			ros::Duration(1).sleep();
			target_pub.publish(target_msg);
			i ++;
			ROS_INFO("i am publising %lf,%lf,%lf", target_msg.pose.position.x,target_msg.pose.position.y,target_msg.pose.position.z);

			break;
		}
*/

		ros::spin();
	    // 按照循环频率延时
	    loop_rate.sleep();
	}

	return 0;
}













