#include <ros/ros.h>

#include <stdio.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib_msgs/GoalID.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ecl/threads.hpp>
#include <ecl/time/sleep.hpp>

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/GetMap.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "Car_Setting_Pos.h"
#include "RS485_TECO_Motor.h"
#include "xmlReadv2.h"
#include "map_maker.h"
//#include "GPIO.h"

#include <vector>
#include <iostream>



using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)

//UDP_SERVER EtherS;

double g_vel=0 , t_vel=0;
double x = 0, y = 0, z = 0, w = 0;

#define moveControlModeType 3
int baseControlMode = 0;

#define FPS_bufferSize 10
double FPS_buffer[FPS_bufferSize] = {0};
double FPS_avg=0;
int FPS_buffer_count = 0;

double Path_A_Now = 0;
double Path_L_Now = 0;

int Path_buffer_count = 0;
cv::Point2d golbalFinalPoint = cv::Point2d(0,0);

#define movTag_bufferSize 5
cv::Point2d movTag_buffer[movTag_bufferSize] = { cv::Point2d(0,0) };
cv::Point2d movTag_avg = cv::Point2d(0, 0);
int movTag_buffer_count = 0;

double frontSafeDistance = 0.2 + 0.2;
double sideSafeDistance = 0.25;

cv::Mat Map_Monitor;
cv::Mat Para_Monitor;
cv::Mat Laser_Monitor;
cv::Mat ALL_Monitor;
cv::Mat User_GUI_Button;

#define ALL_Monitor_SizeW 400
#define ALL_Monitor_SizeH 800
#define Para_Monitor_rect_X 0
#define Para_Monitor_rect_Y 200
#define MotorMovCmd_AutoStop_Count 10000

int Motor_Step_L, Motor_Step_R;

PosTran TranFunc;
cv::Rect Map_Rect;
ecl::Mutex mutex;

SOWAN_PARAMETERv2 SOWANv2("/map/sowan.xml");
#define SOWAN_PARA SOWANv2
std::string HomeToSowanPath = SOWAN_PARA.SowanPath;

CarPosCalsulate RobotPosInfo;
CarPosCalsulate RobotOdomInfo;
CarPosCalsulate::Position odometryPoint;

ros::Time Odom_current_time, Odom_last_time;
ros::Time Path_current_time, Path_last_time;

int MotorMovCmd = 0;
int MotorMovCmdPre = 0;
int MotorMovCmd_AutoStop = 0;
int DeviceCmd = -1;

bool AutoMove = false;

bool ObjectClose = false;
bool MotorAutoMove = false;
bool SomeThingClose_L = false;
bool SomeThingClose_R = false;
bool goal_pub_switch = false;
bool goal_cancel_pub = false;
bool odom_pub_switch = false;
bool GetSaveCmd = false;
nav_msgs::OccupancyGrid Map_Save;
double map_theta ;

void switchOnMouse(int Event, int x, int y, int flags, void* param) {
	int y_div50 = y / 50;
	int x_div100 = x / 100;

	//ButtonSizeW ButtonSizeH
	if (x_div100 < 4 && y_div50 < 4) {
		int dir = (y_div50 * 4) + x_div100;
		if (Event == CV_EVENT_LBUTTONDOWN) {
			printf("CV_EVENT_LBUTTON DOWN = (%d,%d)\n", x, y);
			if ((dir >= 4) && (dir <= 7)) {
				MotorMovCmd = dir;
				MotorMovCmd_AutoStop = MotorMovCmd_AutoStop_Count;
			}
			//else {
			//	//DeviceCmd = dir;
			//}
		}
		else if (Event == CV_EVENT_LBUTTONUP) {
			printf("CV_EVENT_LBUTTON UP = (%d,%d)\n", x, y);
			MotorMovCmd = 0;
			MotorMovCmd_AutoStop = 0;
			if ((dir >= 4) && (dir <= 7)) {
				MotorMovCmd = 0;
				MotorMovCmd_AutoStop = 0;
				DeviceCmd = -1;
			}
			else {
				DeviceCmd = dir;
			}
		}
	}

}


void SaveMapFunction() {
	printf("%d , SaveMapFunction \n", GetSaveCmd);
	if (GetSaveCmd) {
		std::string mapname_ = "/home/pi/catkin_ws/src/map_maker/map/MapAutoSave";
		std::string mapdatafile = mapname_ + ".pgm";

		printf("%d - %s \n", GetSaveCmd, mapname_.c_str());

		ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
		FILE* out = fopen(mapdatafile.c_str(), "w");

		int threshold_occupied_default = 65;
		int threshold_free_default = 25;
		int threshold_occupied_ = threshold_occupied_default;
		int threshold_free_ = threshold_free_default;

		if (!out)
		{
			ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
			return;
		}

		fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
			Map_Save.info.resolution, Map_Save.info.width, Map_Save.info.height);

		ROS_INFO("Save a %d X %d map @ %.3f m/pix_%f_%f_%f",
			Map_Save.info.width,
			Map_Save.info.height,
			Map_Save.info.resolution,
			Map_Save.info.origin.position.x,
			Map_Save.info.origin.position.y,
			map_theta
		);

		for (unsigned int y = 0; y < Map_Save.info.height; y++) {
			for (unsigned int x = 0; x < Map_Save.info.width; x++) {
				unsigned int i = x + (Map_Save.info.height - y - 1) * Map_Save.info.width;
				if (Map_Save.data[i] >= 0 && Map_Save.data[i] <= threshold_free_) { // [0,free)
					fputc(254, out);
				}
				else if (Map_Save.data[i] >= threshold_occupied_) { // (occ,255]
					fputc(000, out);
				}
				else { //occ [0.25,0.65]
					fputc(205, out);
				}
			}
		}

		fclose(out);

		std::string mapmetadatafile = mapname_ + ".yaml";
		ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
		FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
		geometry_msgs::Quaternion orientation = Map_Save.info.origin.orientation;
		tf2::Matrix3x3 mat(tf2::Quaternion(
			orientation.x,
			orientation.y,
			orientation.z,
			orientation.w
		));
		double yaw, pitch, roll;
		mat.getEulerYPR(yaw, pitch, roll);

		fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
			mapdatafile.c_str(), Map_Save.info.resolution, Map_Save.info.origin.position.x, Map_Save.info.origin.position.y, yaw);

		fclose(yaml);

		ROS_INFO("Done\n");

	}

}

int main(int argc, char **argv)
{
  printf("sowan main Start\n");

	ros::init(argc, argv, "map_maker");
  ros::NodeHandle n;

  ecl::Thread Node_Subscriber_Thread;
  //ecl::Thread Node_Publisher_Thread;
  //ecl::Thread ETHER_Thread;

  //ETHER_Thread.start(ETHER_RECV);
  Node_Subscriber_Thread.start(Node_Subscriber);
  //Node_Publisher_Thread.start(Node_Publisher);

  main_init();

  //tf::TransformListener listener(ros::Duration(10));

  ros::Rate loop_rate(20);


  while (ros::ok())
  {
    ros::spinOnce();
/*
	tf::StampedTransform transform;
	try {
		listener.lookupTransform("/map", "/base_link",
			ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		continue;
	}
	x = transform.getOrigin().x();
	y = transform.getOrigin().y();
	z = transform.getRotation().z();
	w = transform.getRotation().w();

	TranFunc.TF_Pos_update(x, y, w, z);
*/
	

	main_function();


    loop_rate.sleep();
  }

  Node_Subscriber_Thread.join();
  //Node_Publisher_Thread.join();

  printf("ros End\n");

  return 0;
  
}

void main_init() {
	Map_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	Para_Monitor = Mat::zeros(cv::Size(400, 200), CV_8UC3);
	Laser_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	ALL_Monitor = Mat::zeros(cv::Size(400, 800), CV_8UC3);


	Odom_current_time = ros::Time::now();
	Odom_last_time = Odom_current_time;

	User_GUI_Button = imread("/home/pi/catkin_ws/src/map_maker/src/ButtonBase.png");
	User_GUI_Button.copyTo(ALL_Monitor(Rect(cv::Point(0, 0), User_GUI_Button.size())));

	Path_last_time = ros::Time::now();
	namedWindow("ALL_Monitor", 0);//cv::WINDOW_NORMAL = 0
	resizeWindow("ALL_Monitor", 400, 800);

	setMouseCallback("ALL_Monitor", switchOnMouse, NULL);

}

void main_function() {

	////
	Odom_current_time = ros::Time::now();


	FPS_buffer[FPS_buffer_count % FPS_bufferSize] = (Odom_current_time - Odom_last_time).toSec();
	SelfPosUpdate(true, FPS_buffer[FPS_buffer_count % FPS_bufferSize]);
	FPS_avg = 0.00000001;
	for (int i = 0; i < FPS_bufferSize; i++) {
		FPS_avg = FPS_avg + (FPS_buffer[i] * 0.1);
	}
	FPS_buffer_count++;

	Odom_last_time = Odom_current_time;
	////



	Para_2_Img();

	//Rect rect1(cv::Point(0, 0), Para_Monitor.size());
	//Para_Monitor.copyTo(ALL_Monitor(rect1));

	Rect rect4(cv::Point(Para_Monitor_rect_X, Para_Monitor_rect_Y), Para_Monitor.size());
	Para_Monitor.copyTo(ALL_Monitor(rect4));

	Rect rect2(cv::Point(0, 400), Laser_Monitor.size());
	Laser_Monitor.copyTo(ALL_Monitor(rect2));



	//cv::Mat Map_Monitor_Draw = Map_Monitor.clone();

	cv::imshow("ALL_Monitor", ALL_Monitor);
	cv::imshow("Map_Monitor", Map_Monitor);

	//cv::imshow("Para_Monitor", Para_Monitor);
	//cv::imshow("Laser_Monitor", Laser_Monitor);

	char ttt = cv::waitKey(1);
	GUI_control(ttt);

	if ((MotorMovCmd_AutoStop <= 0) && (MotorMovCmd != 0)) {
		MotorMovCmd = 0;
	}
	else if (MotorMovCmd_AutoStop > 0) {
		MotorMovCmd_AutoStop--;
	}
	User_GUI_control(DeviceCmd);

	MotorMovCmdPre = MotorMovCmd;
	DeviceCmd = -1;

}

void GUI_control(char ttt) {
	switch(ttt) {
	case 'w':
	case 'x':
	case 'a':
	case 'd':
	case 's':
	case 'q':
	case 'e':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
		break;
	case 'm':
		GetSaveCmd = true;
		SaveMapFunction();
		GetSaveCmd = false;
		break;
		
	}
}
void User_GUI_control(int ttt) {

	switch (ttt) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:

		break;
	case 5:

		break;
	case 6:

		break;
	case 7:

		break;
	case 8:
		GetSaveCmd = true;
		SaveMapFunction();
		GetSaveCmd = false;
		break;
	case 9:

		break;
	case 10:

		break;
	case 11:

		break;
	case 50:

		break;

	default:
		break;
	}

}
void Drew_IMG(cv::Mat *img, std::string Name, std::string input, cv::Point pos) {
	cv::putText(*img, Name, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 255));
	pos.x += 90;
	cv::putText(*img, input, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 255));

}


void SelfPosUpdate(bool speed_on, double time) {
	double p1, p2, s1, s2;

	p1 = 0;
	p2 = 0;
	if (speed_on) {
		//s1 = OrientMotor.Motor_Speed_Return(1);
		//s2 = OrientMotor.Motor_Speed_Return(2);
		RobotPosInfo.GetPos(p1, p2, time);
		RobotOdomInfo.GetPos(p1, p2, time);
	}
	else {
		RobotPosInfo.GetPos(p1, p2);
		RobotOdomInfo.GetPos(p1, p2);
	}
	Motor_Step_L = p1;
	Motor_Step_R = p2;


}
void Para_2_Img() {

	std::string tmpString;
	int setposxString = 10, setposy = 20;

	cv::Mat img = Mat::zeros(cv::Size(400, 400), CV_8UC3);

	Drew_IMG(&img, "TF_x", std::to_string(1000 * TranFunc.TF_Pos.Pos.x), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "COORD.x", std::to_string(RobotPosInfo.SelfPos.COORD.x), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "TF_y", std::to_string(1000 * TranFunc.TF_Pos.Pos.y), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "COORD.y", std::to_string(RobotPosInfo.SelfPos.COORD.y), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "TF_deg", std::to_string(TranFunc.TF_Pos.Degree), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "COORD.D", std::to_string(RobotPosInfo.SelfPos.Degree), cv::Point(setposxString + 200, setposy));

	setposy += 20;
	Drew_IMG(&img, "FPS", std::to_string(1/FPS_avg), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "ObjClo", std::to_string(ObjectClose), cv::Point(setposxString + 200, setposy));
	
	setposy += 20;
	
	

	img.copyTo(Para_Monitor);

}

void Node_Publisher() {

	ros::NodeHandle n;
	//ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	//ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	//ros::Publisher cancel_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	//tf::TransformBroadcaster odom_broadcaster;

	odometryPoint = RobotOdomInfo.SelfPos;

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.0;
	double vy = -0.0;
	double vth = 0.0;

	double dt;
	double delta_x;
	double delta_y;
	double delta_th;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate loop_rate(5);

	while (ros::ok())
	{

		ros::spinOnce();

		current_time = ros::Time::now();

		//////Odom publish
		////if (true) {
		//	//compute odometry in a typical way given the velocities of the robot
		//	dt = (current_time - last_time).toSec();
		//	delta_x = RobotOdomInfo.SelfPos.COORD.x - odometryPoint.COORD.x;
		//	delta_y = RobotOdomInfo.SelfPos.COORD.y - odometryPoint.COORD.y;
		//	delta_th = (RobotOdomInfo.SelfPos.Degree - odometryPoint.Degree);

		//	odometryPoint = RobotOdomInfo.SelfPos;

		//	delta_x = delta_x / 1000.0;
		//	delta_y = delta_y / 1000.0;

		//	//printf("%lf  ", delta_th);
		//	if (delta_th >= 180) {
		//		delta_th -= 360;
		//	}
		//	else if (delta_th <= -180) {
		//		delta_th += 360;
		//	}

		//	delta_th = CV_PI * delta_th / 180.0;

		//	//printf("%lf  \n", delta_th);

		//	x += delta_x;
		//	y += delta_y;
		//	th += delta_th;

		//	vx = delta_x / dt;
		//	vy = delta_y / dt;
		//	vth = delta_th / dt;

		//	//since all odometry is 6DOF we'll need a quaternion created from yaw
		//	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//	//first, we'll publish the transform over tf
		//	geometry_msgs::TransformStamped odom_trans;
		//	odom_trans.header.stamp = current_time;
		//	odom_trans.header.frame_id = "odom";
		//	odom_trans.child_frame_id = "base_link";

		//	odom_trans.transform.translation.x = x;
		//	odom_trans.transform.translation.y = y;
		//	odom_trans.transform.translation.z = 0.0;
		//	odom_trans.transform.rotation = odom_quat;

		//	//send the transform
		//	odom_broadcaster.sendTransform(odom_trans);

		//	//next, we'll publish the odometry message over ROS
		//	nav_msgs::Odometry odom;
		//	odom.header.stamp = current_time;
		//	odom.header.frame_id = "odom";

		//	//set the position
		//	odom.pose.pose.position.x = x;
		//	odom.pose.pose.position.y = y;
		//	odom.pose.pose.position.z = 0.0;
		//	odom.pose.pose.orientation = odom_quat;

		//	//set the velocity
		//	odom.child_frame_id = "base_link";
		//	odom.twist.twist.linear.x = vx;
		//	odom.twist.twist.linear.y = vy;
		//	odom.twist.twist.angular.z = vth;

		//	//publish the message
		//	if (odom_pub_switch)
		//		odom_pub.publish(odom);

		////}

		last_time = current_time;
		loop_rate.sleep();
	}


}


void Node_Subscriber() {

	ros::NodeHandle n;

	ros::Subscriber laser_sub = n.subscribe("scan", 10, scanCallback);
	ros::Subscriber map_sub = n.subscribe("map", 1, mapCallback);

	ros::Rate loop_rate(10);

	printf("Node_Subscriber Working\n");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	printf("Node_Subscriber End %d  \n", ros::ok());

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

	int count = scan->scan_time / scan->time_increment;

	cv::Point2f point_tmp, center;
	cv::Point2f revP,paintP;
	double angle;
	float rawData, maxData = 0, minData = 100;
	maxData = 0;
	Mat img = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	center.x = 200;
	center.y = 200;
	revP.x = 0;
	revP.x = 400;
	bool ThingIsClose = false;

	for (unsigned int i = 0; i < count; ++i)
	{
		rawData = scan->ranges[i];
		if (rawData > maxData && rawData < 30)
			maxData = rawData;
		if (rawData < minData)
			minData = rawData;
	}

	int scalarvalue = 0;
	for (int i = 0; i < count; i++) {
		float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		//ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);

		angle = scan->angle_min + scan->angle_increment * i;
		rawData = scan->ranges[i];
		point_tmp.x = (rawData * cos(angle));
		point_tmp.y = (rawData * sin(angle));
		
		if (point_tmp.x < frontSafeDistance && point_tmp.x > 0 && fabs(point_tmp.y) < sideSafeDistance){
			ThingIsClose = true;
			
		}
		
		paintP = center + (200 * point_tmp / maxData);
		paintP.y = 400 - paintP.y;
		cv::circle(img,
			paintP,
			2,
			cv::Scalar(0, 0, 200),
			CV_FILLED
		);
	}
	cv::circle(img,
		center,
		2,
		cv::Scalar(0, 100, 200),
		CV_FILLED
	);
	
	ObjectClose = ThingIsClose;

	mutex.lock();
	img.copyTo(Laser_Monitor);
	mutex.unlock();

}

void mapCallback(const nav_msgs::OccupancyGridConstPtr &map) {

	geometry_msgs::Quaternion orientation = map->info.origin.orientation;
	double yaw, pitch, roll;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	mat.getEulerYPR(yaw, pitch, roll);
	map_theta = yaw;
	//[1455208385.821892180]: Received a 2048 X 2048 map @ 0.050 m / pix_ - 51.224998_ - 51.224998_0.000000

	//ROS_INFO("Received a %d X %d map @ %.3f m/pix_%f_%f_%f",
	//	map->info.width,
	//	map->info.height,
	//	map->info.resolution,
	//	map->info.origin.position.x,
	//	map->info.origin.position.y,
	//	map_theta
	//);

	Map_Save = *map;
	//Map_Save.info = ->info;
	//Map_Save.data = map->data;

	Mat img = Mat::zeros(cv::Size(map->info.width, map->info.height), CV_8UC1);

	cv::Point MinPos, MaxPos;

	MinPos.x = map->info.width / 2;
	MinPos.y = map->info.height / 2;
	MaxPos = MinPos;

	for (unsigned int y = 0; y < map->info.height; y++) {
		for (unsigned int x = 0; x < map->info.width; x++) {
			unsigned int i = x + (map->info.height - y - 1) * map->info.width;
			int intensity = 205;
			if (map->data[i] >= 0 && map->data[i] <= 100) {
				intensity = round((float)(100.0 - map->data[i])*2.55);

				if (MinPos.x > x)
					MinPos.x = x;
				if (MinPos.y > y)
					MinPos.y = y;
				if (MaxPos.x < x)
					MaxPos.x = x;
				if (MaxPos.y < y)
					MaxPos.y = y;

			}
			img.at<unsigned char>(y, x) = intensity;
		}
	}

	int width = MaxPos.x - MinPos.x;
	int height = MaxPos.y - MinPos.y;
	cv::Rect rect1(MinPos, MaxPos);

	cv::Mat img_out;
	if (width == 0 || height == 0) {
		img_out = Mat::zeros(cv::Size(300, 300), CV_8UC1);
	}
	else {
		img(rect1).copyTo(img_out);

	}
	Map_Rect = rect1;

	TranFunc.PosInit(MaxPos, MinPos, img.size(), map->info.resolution);
	//printf("Get Map Min=(%d,%d)_Max(%d,%d)_width(%d),height(%d)\n", MinPos.x, MinPos.y, MaxPos.x, MaxPos.y, width, height);

	//mutex.lock();
	cv::cvtColor(img_out, Map_Monitor, CV_GRAY2BGR);
	//img_out.copyTo(Map_Monitor);
	//mutex.unlock();

}
