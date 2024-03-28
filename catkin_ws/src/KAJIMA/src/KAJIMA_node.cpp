#include <ros/ros.h>
#include <stdio.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <ecl/threads.hpp>
#include <ecl/time/sleep.hpp>


#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib> 
#include <ctime>  

#include "ina228.h"
#include "Car_Setting_Pos.h"
#include "KAJIMA_node.h"
#include "EthernetRlidar.h"
#include "MQTT_tool.h"
#include "FileRead.h"


uint32_t hI2C;
char* devname = "/dev/i2c-1";
unsigned char i2caddr = 0x4a;
bool reset = false;
double Ina228_Voltage = 0;
int Ina228_UpdateCount = 0;

MQTT_tool mqtt_tool;

//Thread Work
ecl::Thread Node_Subscriber_Thread;
ecl::Thread Node_Publisher_Thread;
ecl::Thread Rlidar_Ether_Thread;

PosTran TranFunc;
double x = 0, y = 0, z = 0, w = 0;
bool Sowan_System_Work = true;
cv::Rect Map_Rect;
cv::Mat Map_Monitor;
cv::Mat Laser_Monitor;
cv::Mat ALL_Monitor;
cv::Mat User_GUI_Button;
cv::Mat Para_Monitor;
#define ALL_Monitor_SizeW 400
#define ALL_Monitor_SizeH 800
#define Para_Monitor_rect_X 0
#define Para_Monitor_rect_Y 200
bool GetMap = false, GetLAser = false;

geometry_msgs::PoseWithCovarianceStamped pose_msg;
bool Publish_InitialPose = false;
double InitialPose_Setting[3] = { 0.0 };

ecl::Mutex mutex;

Remote_Rlidar_Server ServerRlidar;

FileRead FileName;


#define RAD2DEG(x) ((x)*180./M_PI)
using namespace cv;
using namespace std;
std::string PosTxt = "/home/pi/pos.txt";

int main(int argc, char **argv) {
	ros::init(argc, argv, "KAJIMA");
	ros::NodeHandle n;



	hI2C = i2c_init(devname);
	printf("reset %d\r\n", reset);
	if (reset) {
		printf("Resetting device\r\n");
		i2c_write_short(hI2C, i2caddr, INA228_CONFIG, 0x8000);	// Reset
	}
	ina228_init(hI2C, i2caddr);

	//mqtt_tool.Init("192.168.68.150", "RASPI_C3");

	ServerRlidar.Socket_init();

	InitialPose_Setting[0] = 0;
	InitialPose_Setting[1] = 0;
	InitialPose_Setting[2] = 3.1415926 / 2;

	Map_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	Laser_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	ALL_Monitor = Mat::zeros(cv::Size(400, 800), CV_8UC3);
	Para_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	std::string stringX, stringY, stringD;

	Node_Subscriber_Thread.start(Node_Subscriber);
	Node_Publisher_Thread.start(Node_Publisher);
	Rlidar_Ether_Thread.start(RLIDAR_ETHER);

	tf::TransformListener listener(ros::Duration(10));
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ros::spinOnce();

		tf::StampedTransform transform;
		try {
			listener.lookupTransform("map", "base_link",
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

		if(GetLAser && GetMap) {

			

			Para_Monitor = cv::Scalar(0,0,0);
			stringX = std::to_string(TranFunc.TF_Pos.Pos.x);
			stringY = std::to_string(TranFunc.TF_Pos.Pos.y);
			stringD = std::to_string(TranFunc.TF_Pos.Degree);
			
			cv::putText(Para_Monitor, stringX, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 100, 255), 1);
			cv::putText(Para_Monitor, stringY, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 100, 255), 1);
			cv::putText(Para_Monitor, stringD, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 100, 255), 1);
			//Ina228Update();

			Rect rect4(cv::Point(0, 400), Para_Monitor.size());
			Para_Monitor.copyTo(ALL_Monitor(rect4));

			Rect rect2(cv::Point(0, 0), Laser_Monitor.size());
			Laser_Monitor.copyTo(ALL_Monitor(rect2));

			ServerRlidar.UpdatePos(
				TranFunc.TF_Pos.Pos.x, 
				TranFunc.TF_Pos.Pos.y,
				TranFunc.TF_Pos.Degree);

			cv::imshow("ALL_Monitor", ALL_Monitor);
			cv::waitKey(1);
		}

		//main_function();

		loop_rate.sleep();
	}
	
	Node_Subscriber_Thread.join();
	Node_Publisher_Thread.join();
	Rlidar_Ether_Thread.join();
	return 0;

}

void Ina228Update() {
	Ina228_UpdateCount++;
	if (Ina228_UpdateCount == 30) {
		Ina228_Voltage = ina228_voltage(hI2C, i2caddr);
		if (Ina228_Voltage == 0) {
			std::string stringTmp = std::to_string(Ina228_Voltage);
			cv::putText(Para_Monitor, stringTmp, cv::Point(30, 90), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 100, 255), 1);
			return;
		}
		Ina228_UpdateCount = 0;	

		int percent_voltage = (int)((Ina228_Voltage - 21.0) * (100.0 / (29.0 - 21.0)) + 0.5);
		if (percent_voltage > 100)
			percent_voltage = 100;
		if (percent_voltage < 0)
			percent_voltage = 0;
		stringstream ss;
		ss << "C3@bat=" << percent_voltage;
		printf("send string = %s\n", ss.str().c_str());
		mqtt_tool.send("fromRobot", ss.str());
	}	

	std::string stringTmp = std::to_string(Ina228_Voltage);
	cv::putText(Para_Monitor, stringTmp, cv::Point(30, 90), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 100, 255), 1);

	//printf("Voltage:         %.03f V\r\n", );
	//printf("Die Temperature: %.01f degC\r\n", ina228_dietemp(hI2C, i2caddr));
	//printf("Shunt Voltage:   %.03f mV\r\n", ina228_shuntvoltage(hI2C, i2caddr));
	//printf("Current:         %.03f mA\r\n", ina228_current(hI2C, i2caddr));
	//printf("Power:           %.03f W\r\n", ina228_power(hI2C, i2caddr));
	//printf("Energy:          %.05f Wh\r\n", ina228_energy(hI2C, i2caddr) / 3600);
	//printf("Charge:          %.05f Ah\r\n", ina228_charge(hI2C, i2caddr) / 3600);
	//printf("\r\n");
}

void Node_Publisher() {
	ros::NodeHandle n;
	ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

	tf::TransformBroadcaster odom_broadcaster;

	//odometryPoint = RobotOdomInfo.SelfPos;

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
	//current_time = ros::Time::now();
	//last_time = ros::Time::now();

	ros::Rate loop_rate(5);
	bool posexist = false;
	double Read_X, Read_Y, Read_D, tmp;
	
	while (ros::ok() && Sowan_System_Work) {
		ros::spinOnce();

		current_time = ros::Time::now();

		posexist = FileName.FileExist(PosTxt);
		if (posexist) {
			printf("FileName %d\n", posexist);
			posexist = false;

			FileName.FileLoadInit(PosTxt);
			tmp = 0;
			bool res = FileName.FindPara_Double("X:", &tmp);
			if (res == false) {
				Read_X = 0;
			}
			else {
				Read_X = tmp;
			}

			res = FileName.FindPara_Double("Y:", &tmp);
			if (res == false) {
				Read_Y = 0;
			}
			else {
				Read_Y = tmp;
			}
			
			res = FileName.FindPara_Double("D:", &tmp);
			if (res == false) {
				Read_D = 0;
			}
			else {
				Read_D = tmp;
			}
			printf("%f,%f,%f\n", Read_X, Read_Y, Read_D);
			Publish_InitialPose = true;
			InitialPose_Setting[0] = Read_X;
			InitialPose_Setting[1] = Read_Y;
			InitialPose_Setting[2] = Read_D;
			FileName.FileClose();

			FileName.FileRemove(PosTxt);
		}



		if (Publish_InitialPose) {

			pose_msg.header.stamp = ros::Time::now();
			pose_msg.header.frame_id = "map";
			pose_msg.pose.pose.position.x = InitialPose_Setting[0];
			pose_msg.pose.pose.position.y = InitialPose_Setting[1];
			pose_msg.pose.pose.orientation.z = sin(InitialPose_Setting[2]);
			pose_msg.pose.pose.orientation.w = cos(InitialPose_Setting[2]);

			//tf::Quaternion quat;
			//quat.setRPY(0.0, 0.0, theta);
			//tf::quaternionTFToMsg(quat, pose_msg.pose.pose.orientation);

			pose_msg.pose.covariance[0] = 0.25;
			pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
			pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942;


			initial_pose_pub.publish(pose_msg);
			Publish_InitialPose = false;
		}

		last_time = current_time;
		loop_rate.sleep();
	}
	printf( "Node_Publisher End");
}
void Node_Subscriber() {

	ros::NodeHandle n;

	ros::Subscriber laser_sub = n.subscribe("scan", 10, scanCallback);
	ros::Subscriber map_sub = n.subscribe("map", 3, mapCallback);

	ros::Rate loop_rate(10);

	printf( "Node_Subscriber Working");

	while (ros::ok() && Sowan_System_Work)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	printf( "Node_Subscriber End %d  ", ros::ok());

}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

	int count = scan->scan_time / scan->time_increment;

	cv::Point2f point_tmp, center;
	cv::Point2f revP, paintP;
	double angle;
	float rawData, maxData = 0, minData = 100;
	maxData = 0;
	Mat img = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	center.x = 200;
	center.y = 200;
	revP.x = 0;
	revP.x = 400;
	bool ThingIsClose = false;

	//if (ServerRlidar.LidarDis.size() < 2*count) {
	if (ServerRlidar.LidarDis.size() <  count) {
		ServerRlidar.LidarDis.clear();
		ServerRlidar.LidarTh.clear();
		//ServerRlidar.LidarDis.assign(2 * count, 0);
		//ServerRlidar.LidarTh.assign(2 * count, 0);
		ServerRlidar.LidarDis.assign( count, 0);
		ServerRlidar.LidarTh.assign( count, 0);
	}
	for (unsigned int i = 0; i < count; ++i)
	{
		rawData = scan->ranges[i];
		angle = scan->angle_min + scan->angle_increment * i;
		if (rawData > maxData && rawData < 30)
			maxData = rawData;
		if (rawData < minData)
			minData = rawData;
		ServerRlidar.LidarDis.at(i) =(int)(rawData * 10000) ;
		//ServerRlidar.LidarDis.at(i+count) = (int)(rawData * 10000);
		ServerRlidar.LidarTh.at(i) = (int)(angle * 10000);
		//ServerRlidar.LidarTh.at(i+count) = (int)(angle * 10000);
	}


	int scalarvalue = 0;
	for (int i = 0; i < count; i++) {
		float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		//ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);

		angle = scan->angle_min + scan->angle_increment * i;
		rawData = scan->ranges[i];
		point_tmp.x = (rawData * cos(angle));
		point_tmp.y = (rawData * sin(angle));

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

	mutex.lock();
	img.copyTo(Laser_Monitor);
	GetLAser = true;
	mutex.unlock();

}
void mapCallback(const nav_msgs::OccupancyGridConstPtr &map) {
	
	geometry_msgs::Quaternion orientation = map->info.origin.orientation;
	double yaw, pitch, roll;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	mat.getEulerYPR(yaw, pitch, roll);
	double map_theta = yaw;
	//[1455208385.821892180]: Received a 2048 X 2048 map @ 0.050 m / pix_ - 51.224998_ - 51.224998_0.000000

	ROS_INFO("Received a %d X %d map @ %.3f m/pix_%f_%f_%f",
		map->info.width,
		map->info.height,
		map->info.resolution,
		map->info.origin.position.x,
		map->info.origin.position.y,
		map_theta
	);

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
	bool img_out_Work = false;
	if (width == 0 || height == 0) {
		img_out = Mat::zeros(cv::Size(300, 300), CV_8UC1);
	}
	else {
		img(rect1).copyTo(img_out);
		img_out_Work = true;
	}
	Map_Rect = rect1;

	if (img_out_Work) {
		mutex.lock();
		GetMap = true;
		cv::cvtColor(img_out, Map_Monitor, CV_GRAY2BGR);
		
		//Map_Router.MapInit(&Map_Monitor);
		//Map_Router.GetMapRect(Map_Rect);
		
		mutex.unlock();

	}
	TranFunc.PosInit(MaxPos, MinPos, img.size(), map->info.resolution);
	printf( "Get Map Min=(%d,%d)_Max(%d,%d)_width(%d),height(%d)", MinPos.x, MinPos.y, MaxPos.x, MaxPos.y, width, height);

}



void RLIDAR_ETHER(void) {

	ServerRlidar.ether_main();

}
