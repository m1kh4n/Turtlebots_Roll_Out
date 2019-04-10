#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc.hpp"
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <sensor_msgs/LaserScan.h>

//Paramenters
#define FEAR 2000
#define LOST 2000
#define BACKUP_DURATION 7000
#define CRYING_FACTOR 3

//For Debugging
#define STATE1
#define STATE2
#define STATE3
#define STATE4
//#define SENSE_TEST
#define VERBOSE

using namespace std;
using namespace cv;

//Global Variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel;
int world_state;
int moveBackCount;
int angryCount;
int lostCount;
bool startFlag0,startFlag1, startFlag2, startFlag3, startFlag4;

//Load Images
cv::Mat neutral = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/neutral.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat surprised = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/surprise.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat surprisedLeft = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/surprise_left.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat surprisedRight = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/surprised_right.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat angry = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/angry.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat fear = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/fear.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat sad1 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/sad1.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat sad2 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/images/sad2.jpg",CV_LOAD_IMAGE_COLOR);

// Callback Function
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

struct Bumper{
    bool centre;
    bool right;
    bool left;
};
struct Bumper bumperState = {0,0,0};

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr msg){
    //Fill with code
    if(msg->bumper == 0)
	    bumperState.left = !bumperState.left;
    else if(msg->bumper == 1)
	    bumperState.centre = !bumperState.centre;
    else if(msg->bumper ==2)
	    bumperState.right = !bumperState.right;
}

bool wheelRight = 0;
bool wheelLeft = 0;

void wheeldropCB(const kobuki_msgs::WheelDropEvent::ConstPtr msg){
	if (msg->state == 1){
        	if (msg->wheel == 1){
            		wheelRight = 1;
        	}
		else if (msg-> wheel == 0){
            		wheelLeft = 1;
        	}
    	}
    	else if (msg -> state == 0){
        	if (msg-> wheel == 1){
            		wheelRight = 0;
        	}
        	else if (msg-> wheel == 0){
            		wheelLeft = 0;
        	}
    	}
    	//std::cout<<"wheel Right: "<<wheelRight<<std::endl;
    	//std::cout<<"wheel Left: "<<wheelLeft<<std::endl;
    	//ROS_INFO("wheelRight: %d, wheelLeft: %d",wheelRight, wheelLeft);
}

double laserRange = 10.0;
int laserSize = 0;
int laserOffset = 0;
double pi = 3.1415926;
double desiredAngle = 7.5;
double desiredAngleRad = desiredAngle*pi/180.0;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/(msg->angle_increment);
    	laserOffset = desiredAngleRad/(msg->angle_increment);
    	laserRange = 11.0;
    
    	//Store Laser Range
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
    
	if(laserRange == 11) laserRange = 0;
    
    //ROS_INFO("Range of laser is: %lf", laserRange);
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

//Helper Functions
bool isLost(){
	if(follow_cmd.linear.z==0 && follow_cmd.linear.x && follow_cmd.linear.y ==0 && (laserRange>2 || laserRange ==0))
		return true;
	else
		return false;
}

void statusUpdate(){
    /*if(prev_world_state != world_state)
        prev_world_state = world_state;
     */
    //Reset Flag
	if(world_state != 0)
		startFlag0 = false;
	if(world_state != 1)
        	startFlag1 = false;
    	if(world_state != 2)
        	startFlag2 = false;
    	if(world_state != 3){
        	startFlag3 = false;
        	moveBackCount = 0;
    	}
    	if(world_state != 4){
        	startFlag4 = false;
        	lostCount = 0;
    	}
}

//Main Function
int main(int argc, char **argv)
{
	//Initialize and Declare Ros Topic Variables
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//Initialize Publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//Initialize Subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
    	ros::Subscriber wheeldrop = nh.subscribe("mobile_base/events/wheel_drop",10,&wheeldropCB);
    	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	//Initialize imageTransport
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    	//Initialize Variables at Startup
    	world_state = 0;
    	//prev_world_state = 0;
   	moveBackCount = 0;
    	lostCount = 0;
   	startFlag1 = startFlag2 = startFlag3 = startFlag4 = 0;

	double angular = 0.2;
	double linear = 0.0;

	vel.angular.z = angular;
	vel.linear.x = linear;
    
    	//Play startup sound and image
	sc.playWave(path_to_sounds + "wakeup.wav");
	ros::Duration(0.5).sleep();
    	cv::namedWindow("Display Window",WINDOW_AUTOSIZE);
    	cv::imshow("Display Window",neutral);
    	cv::waitKey(30);
    	sc.stopWave(path_to_sounds+"wakeup.wav");
    	ROS_INFO("I'm Awake");

    	//Turtlebot Operation Loop
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		// eStop.block();
		//...................................

		//Wall-E Senses
        	if(0){} //For ifdef debugging purposes
        	//1. Detect if Wall-E picked up
#ifdef STATE1
		else if(wheelRight == 1 || wheelLeft == 1){
            		ROS_INFO("Help! I'm picked up");
			world_state = 1;
		}
#endif
#ifdef STATE2
        	//2 Check if obstacle in front
		else if(bumperState.left == 1 || bumperState.right == 1 || bumperState.centre == 1){
            		ROS_INFO("Obstacle Ahead!");
            		world_state = 2;
		}
#endif
#ifdef STATE3
        	//3.Detect if move backed FEAR times
        	else if(follow_cmd.linear.x < 0 || follow_cmd.linear.y < 0){
            		ROS_INFO("moveBackCount: %d",moveBackCount);
            		moveBackCount++;
            		if (moveBackCount > FEAR){
                		ROS_INFO("I'm Scared");
                		world_state = 3;
            		}
        	}
#endif
#ifdef STATE4
        	//Check if Wall-E lost LOST times
		else if(isLost()){
            		ROS_INFO("moveBackCount: %d",lostCount);
            		lostCount++;
            		if (lostCount > LOST){
                		ROS_INFO("I'm Lost");
                		lostCount = 0;
                		world_state = 4;
            		}
		}
#endif
        	//0.If nothing sensed, keep following person
		else{
            		ROS_INFO("Follwing Person");
			world_state = 0;
		}
		
#ifdef VERBOSE
		ROS_INFO("wheelLeft/Right: %d %d | bumperStates: %d %d %d | follow_cmd(x,y): (%f,%f) | isLost?: %d", wheelLeft, wheelRight, bumperState.left,			bumperState.centre, bumperState.right, follow_cmd.linear.x, follow_cmd.linear.y, isLost());
#endif
        
       		 //Update Status
        	statusUpdate();
        
		//Wall-E Emotion Reactions
        	//0.Follow Person
		if(world_state == 0){
			vel_pub.publish(follow_cmd);
           		ROS_INFO("X = %lf, Y = %lf, Z = %lf",follow_cmd.linear.x,follow_cmd.linear.y,follow_cmd.angular.z);
            		sc.playWave(path_to_sounds+"silent.wav");
            
           		sleep(0.25);
		}
#ifndef SENSE_TEST
#ifdef STATE1
        	//1.Suprised
        	else if(world_state == 1){
            		ROS_INFO("Surprised Emotion");
            		//Surprised Emotions
            		if(startFlag1 == false){
                		sc.playWave(path_to_sounds+"suprised.wav");
                		startFlag1 = true;
            		}
            		if (wheelLeft == 1 && wheelRight == 1){
                	// normal suprised image
                		cv::imshow("Display Window",surprised);
                		cv::waitKey(30);
            		}
            		else if (wheelLeft == 1 && wheelRight == 0){
  				//tilted right suprised image
  				cv::imshow("Display Window",surprisedRight);
                		cv::waitKey(30);
  			}
  			else if (wheelLeft == 0 && wheelRight == 1){
  				//tilted left suprised image
  				cv::imshow("Display Window",surprisedLeft);
                		cv::waitKey(30);
  			}
		}
#endif
#ifdef STATE2
        	//2.Angry
		else if(world_state == 2){
            		ROS_INFO("Angry Emotion");
            		//Angry Emotions
            		if(startFlag2 == false){
                		sc.playWave(path_to_sounds+"angry.wav");
                		cv::imshow("Display Window",angry);
                		cv::waitKey(30);
                		startFlag2 = true;
            		}
            		//Move Back
            		vel.linear.x = -1;
            		vel.linear.y = 0;
            		vel.linear.z = 0;
            
            		for(int i=0;i<BACKUP_DURATION;i++){
                		vel_pub.publish(vel);
            		}
		}
#endif
#ifdef STATE3
        	//3.Fear
		else if(world_state == 3){
            		ROS_INFO("Fear Emotion");
            		//Fear Emotions
            		if(startFlag3 == false){
                		sc.playWave(path_to_sounds+"fear.wav");
                		cv::imshow("Display Window",fear);
                		cv::waitKey(30);
                		startFlag3 = true;
            		}
		}
#endif
#ifdef STATE4
        	//4.Sad
		else if(world_state ==4){
            		ROS_INFO("Sad");
			//Display 'sad' image
            		if(lostCount > (CRYING_FACTOR*LOST)){
                		if(startFlag4 == false){
                    			sc.playWave(path_to_sounds+"sad.wav");
                    			cv::imshow("Display Window",sad2);
                    			cv::waitKey(30);
                    			startFlag4 = true;
                		}
            		}
            		else if(lostCount > LOST){
                		sc.playWave(path_to_sounds+"silent.wav");
                		cv::imshow("Display Window",sad1);
                		cv::waitKey(30);
            		}
		}
#endif
#endif
	} //While Loop End Bracket
	return 0;
} //Main End Bracket

//ARCHIVE
/*
 int prev_world_state;
 
 //Reset Playback if new world_state
 if (prev_world_state != world_state){
 if(prev_world_state == 0)
 //sc.stopWave(path_to_sounds+"neutral.wav");
 if(prev_world_state == 1)
 sc.stopWave(path_to_sounds+"surprised.wav");
 if(prev_world_state == 2)
 sc.stopWave(path_to_sounds+"angry.wav");
 if(prev_world_state == 3)
 sc.stopWave(path_to_sounds+"fear.wav");
 if(prev_world_state == 4)
 sc.stopWave(path_to_sounds+"sad.wav");
 }
 */

