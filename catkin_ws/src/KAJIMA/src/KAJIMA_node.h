#ifndef __KAJIMA_NODE_H__
#define __KAJIMA_NODE_H__

#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ecl/threads.hpp>
#include <ecl/time/sleep.hpp>

#include "Car_Setting_Pos.h"
//#include "xmlRead.h"

using namespace cv;

void main_init();
void main_function();

int main(int argc, char **argv);


void GUI_control(char ttt);
void User_GUI_control(int ttt);
void UDP_ethernet_Input();
void GUI_ethernet_Input();

void mapOnMouse(int Event, int x, int y, int flags, void* param);
void switchOnMouse(int Event, int x, int y, int flags, void* param);


void Ina228Update();
void Node_Publisher();
void Node_Subscriber();
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void mapCallback(const nav_msgs::OccupancyGridConstPtr &map);
void RLIDAR_ETHER();

#endif