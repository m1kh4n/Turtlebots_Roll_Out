#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;

enum{
	REGULARMODE = 0,
	SAFEMODE = 1,
};

//-----Odometry Variables-----//
double posX, posY, yaw;
double pi = 3.1416;

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180.0/pi);
}

//-----Bumper Variables-----//
bool bumperLeft = 0;
bool bumperRight = 0;
bool bumperCentre = 0;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr msg){
	if(msg->bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg->bumper == 1)
		bumperCentre = !bumperCentre;
	else if(msg->bumper == 2)
		bumperRight = !bumperRight;
}

//-----Laser Variables-----//
double laserRange = 10.0;
int laserSize = 0;
int laserOffset = 0;
double desiredAngle = 10.0;
double desiredAngleRad = desiredAngle*pi/180.0;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/(msg->angle_increment);
	laserOffset = desiredAngleRad/(msg->angle_increment);
	laserRange = 11.0;

	if(desiredAngleRad < msg->angle_max && -desiredAngleRad > msg->angle_min){
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(laserRange > msg->ranges[i]){
				laserRange = msg->ranges[i];
			}
		}
	}
	else{
		for(int i = 0; i < laserSize; i++){
			if(laserRange > msg->ranges[i]){
				laserRange = msg->ranges[i];
			}
		}
	}

	if(laserRange == 11)
		laserRange = 0;
	
	ROS_INFO("Range of laser is: %lf", laserRange);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

//-----Movement Functions-----//
double angular = 0.0;
double angleSpeed=pi/12;

double linear = 0.0;
double maxSpeed = 0.25;

void stop(){
	linear = 0;
	angular = 0;
}

void moveForward(float speed, int safety){
	angular = 0;
	if (safety == 0){
		if(speed < maxSpeed)
			linear = speed;
		else
			linear = maxSpeed;
	}
	else if (safety == 1){
		if (laserRange < 0.45)
			linear = 0;
		else if (laserRange < 1)
			linear = 0.1;
		else if(speed < maxSpeed)
			linear = speed;
		else
			linear = maxSpeed;
	}
	ROS_INFO("Robot moving forward with speed of: %lf.", linear); 
}

void moveBackwards(){
	angular = 0;
	linear = -0.1;
}

void rotate(float angle){
	linear = 0;
	double starting_yaw = yaw;
	if (angle > 0){
		while (yaw-starting_yaw < angle){
			angular = angleSpeed;	
		}
	}
	else if (angle < 0){
		while (yaw-starting_yaw > angle){
			angular = -angleSpeed;	
		}
	}
	ROS_INFO("Robot turning with speed of: %lf.", angular);
} 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;
    
    	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	
	geometry_msgs::Twist vel;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		
		if (laserRange < 0.45){
			stop();
			rotate(90.0);
		}
		else if (laserRange < 1)
			moveForward(0.1, REGULARMODE);
		else if(laserRange >= 1)
			moveForward(0.2, REGULARMODE);
	        //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
        	ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
		//ROS_INFO("Laser Range: %i", laserRange);

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
