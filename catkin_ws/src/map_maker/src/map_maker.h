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

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ecl/threads.hpp>
#include <ecl/time/sleep.hpp>

#include "Car_Setting_Pos.h"
#include "RS485_TECO_Motor.h"


#include <vector>
#include <iostream>


using namespace cv;

void switchOnMouse(int Event, int x, int y, int flags, void* param);
void MotorMoveStop(double acc);
void MotorMoveStop();
void User_GUI_control(int ttt);
void GUI_control(char ttt);
void Drew_IMG(cv::Mat *img, std::string Name, std::string input, cv::Point pos);

void MotorMoveControl(char dir);

void SelfPosUpdate(bool speed_on,double time);
void Para_2_Img();

void main_init();
void main_function();
int main(int argc, char **argv);

//void odom_publisher(double dt, ros::Publisher* odom_pub, tf::TransformBroadcaster* odom_broadcaster);
//void odom_publisher(double dt, double *odom_x, double *odom_y, double *odom_th, ros::Publisher* odom_pub, tf::TransformBroadcaster* odom_broadcaster);

void Node_Publisher();

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void mapCallback(const nav_msgs::OccupancyGridConstPtr &map);


void Node_Subscriber();

