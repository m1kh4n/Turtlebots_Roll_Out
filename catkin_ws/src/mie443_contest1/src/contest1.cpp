#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>
#include <stdlib.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#define RAD_TO_DEG 180.0/M_PI
#define DEG_TO_RAD M_PI/180.0

using namespace std;

enum{
	REGULARMODE = 0,
	SAFEMODE = 1,
};

enum{
	INITIAL = 0,
	EXPLORATION = 1,
	SCAN = 2,
};

enum{
	RIGHT =-1,
	LEFT = 1,
};


//-----Odometry Variables-----//
double posX;
double posY;
double yaw;
double correctedYaw;
double pi = 3.1416;

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = (tf::getYaw(msg->pose.pose.orientation))*RAD_TO_DEG;
	
	if (yaw <= 0){
		correctedYaw = (180.0-abs(yaw)) + 180.0;
	}
	else
		correctedYaw = yaw;
			
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
double desiredFOV = 40.0;
double FOVOffset;
double incrementsPerAngle;
double laserArray[58];

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/(msg->angle_increment);
	laserOffset = desiredAngleRad/(msg->angle_increment);
	FOVOffset = (desiredFOV/2.0)/(msg->angle_increment);
	laserRange = 11.0;
	incrementsPerAngle = 1.0/((msg->angle_increment)*RAD_TO_DEG);
	int incrementsPerAngleRounded = round(incrementsPerAngle);	
	
	//Store Laser Array
	for(int j = 0; j < sizeof(msg->ranges)/sizeof(msg->ranges[0]); j++){
		if(j%incrementsPerAngleRounded == 0)
			laserArray[j] = msg->ranges[j];
	}

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
	
	//ROS_INFO("Range of laser is: %lf", laserRange);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

//-----Movement Functions-----//
double angular = 0.0;
double angleSpeed=pi/6;
double startingYaw;
double goalYaw;
int rotateState;
int firstRotate;
double temp;
double desiredRotation;

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

	//if not moving straight
	//if(abs(yaw-movingForward.yaw)>1){
		//add rotation adjustment
	//}
	ROS_INFO("Robot moving forward with speed of: %lf.", linear); 
}

void moveBackwards(){
	angular = 0;
	linear = -0.1;
}

void rotate(int direction, float angularSpeed){
	linear = 0;
	if (direction == LEFT){
		angular = angularSpeed;
	}
	else if (direction == RIGHT){
		angular = -angularSpeed;
	}
	rotateState = 1;
	//ROS_INFO("Robot turning with speed of: %lf.", angular);
}

bool cornered(){
	for (int i; i<(sizeof(laserArray))/(sizeof(laserArray[0]));i++){
		if(laserArray[i]<0.5)
			return true;
	}
	return false;
}

int  turnDirection(){
	double maxLaser=0;
	int maxLaser_index;
	for(int i=0; i<(sizeof(laserArray))/sizeof(laserArray[0]);i++){
		if(laserArray[i]>maxLaser){
			maxLaser = laserArray[i];
			maxLaser_index = i;
		}
	}
	if(maxLaser_index>((sizeof(laserArray))/(sizeof(laserArray[0])))/2)
		return RIGHT;
	else
		return LEFT;	
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

	//Initialize mode
	int mode = INITIAL;
	int state = 0;
	int scanCount;

	while(yaw == 0){
		ros::spinOnce();
	}	
	
	int storeForwardHeading = 0;
	double moveForwardHeading = correctedYaw;
	
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................		

		//angular = 0.2;
		
		if(mode == INITIAL){
			//Scan 360 degrees
			desiredRotation = 350.0; //Default desired rotation for scan state, allows robot to scan pretty much everything around it  	
			
			//----------Scan needs to be able to remember the max range it found during the scan and the yaw of that----------//
			//----------range so it can point the robot in that direction again.----------------------------------------------//
			double maxRange = 0; 
			double maxRangeHeading = 0;

			//-------------------New rotation code, based on while loops and modulus of 360-------------------------------------------------//
		
			//-------------------This code only works for rotating in one direction right now and only in the scan state, so----------------// 
			//-------------------if we want to scan 360(or any angle) in exploration mode, state must first be set to INITIAL --------------//
			//-------------------and then the robot will start rotating the next time it loops through the main while loop.-----------------//
			//-------------------Further improvements can be made by adding a check for a rotation flag at the beginning of the main--------//
			//-------------------while loop so that everytime the main loop runs, the robot checks if another part of the code--------------//
			//-------------------has requested a rotation in the cycle before and if the flag is set as TRUE, it will execute the-----------//
			//-------------------rotation loop.---------------------------------------------------------------------------------------------//  	
			
			//-------------------Initial scan of surroundings-------------------------------------------------------------------------------//	
				
		
			goalYaw = correctedYaw + desiredRotation;
			if(goalYaw > 360.0)
				goalYaw = goalYaw - 360.0;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
					
				ROS_INFO("In first rotate while loop, publishing data, goalYaw: %lf, correctedYaw: %lf, difference: %lf.\n", goalYaw, correctedYaw, abs(goalYaw-correctedYaw));
					
				if (yaw <= 0){
					correctedYaw = (180.0-abs(yaw)) + 180.0;
				}
				else
					correctedYaw = yaw;

				if (laserRange > maxRange){
					maxRange = laserRange;
					maxRangeHeading = correctedYaw;
				}
				rotate(LEFT, 0.3);
				vel.angular.z = angular;
				vel.linear.x = linear;

				vel_pub.publish(vel);
			}
				
			//------------------Reorienting robot to the yaw which had the longest range, and maybe want to check if the--------------------//
			//------------------sides are clear as well? Just so no corners are clipped. Not implemented yet. Maybe store-------------------//
			//------------------second and third longest ranges as well. Test and see.------------------------------------------------------//
			goalYaw = maxRangeHeading;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
				ROS_INFO("In second rotate while loop, should be aligning with direction of longest range, goalYaw: %lf, correctedYaw: %lf.\n", goalYaw, correctedYaw);
					
				if (yaw <= 0){
					correctedYaw = (180.0-abs(yaw)) + 180.0;
				}
				else
					correctedYaw = yaw;

				rotate(LEFT,0.3);
					
				vel.angular.z = angular;
				vel.linear.x = linear;
				vel_pub.publish(vel);
			}
			//------------------Scanning complete, robot correctly oriented, set mode to EXPLORATION for next cycle-------------------------//
			//------------------Also reset linear and angular velocities so publish at the end doesn't publish junk-------------------------//	
			mode = EXPLORATION;
			storeForwardHeading == 1;			
	
			angular = 0;
			linear = 0;
			scanCount = 0;
			vel.angular.z = 0;
			vel.linear.x = 0;
			vel_pub.publish(vel);			
		}

		//Scan Mode
		else if(mode == SCAN){
			//Same as inital but only scan 90 degrees
			desiredRotation = 350.0; //Default desired rotation for scan state, allows robot to scan pretty much everything around it  	
			
			//----------Scan needs to be able to remember the max range it found during the scan and the yaw of that----------//
			//----------range so it can point the robot in that direction again.----------------------------------------------//
			double maxRange = 0; 
			double maxRangeHeading = 0;

			//-------------------New rotation code, based on while loops and modulus of 360-------------------------------------------------//
		
			//-------------------This code only works for rotating in one direction right now and only in the scan state, so----------------// 
			//-------------------if we want to scan 360(or any angle) in exploration mode, state must first be set to INITIAL --------------//
			//-------------------and then the robot will start rotating the next time it loops through the main while loop.-----------------//
			//-------------------Further improvements can be made by adding a check for a rotation flag at the beginning of the main--------//
			//-------------------while loop so that everytime the main loop runs, the robot checks if another part of the code--------------//
			//-------------------has requested a rotation in the cycle before and if the flag is set as TRUE, it will execute the-----------//
			//-------------------rotation loop.---------------------------------------------------------------------------------------------//  	
			
			//-------------------Initial scan of surroundings-------------------------------------------------------------------------------//	
					
			goalYaw = correctedYaw + desiredRotation;
			if(goalYaw > 360.0)
				goalYaw = goalYaw - 360.0;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
					
				ROS_INFO("In first rotate while loop, publishing data, goalYaw: %lf, correctedYaw: %lf, difference: %lf.\n", goalYaw, correctedYaw, abs(goalYaw-correctedYaw));
					
				if (yaw <= 0){
					correctedYaw = (180.0-abs(yaw)) + 180.0;
				}
				else
					correctedYaw = yaw;

				if (laserRange > maxRange){
					maxRange = laserRange;
					maxRangeHeading = correctedYaw;
				}
				rotate(LEFT, 0.4);
				vel.angular.z = angular;
				vel.linear.x = linear;

				vel_pub.publish(vel);
			}
				
			//------------------Reorienting robot to the yaw which had the longest range, and maybe want to check if the--------------------//
			//------------------sides are clear as well? Just so no corners are clipped. Not implemented yet. Maybe store-------------------//
			//------------------second and third longest ranges as well. Test and see.------------------------------------------------------//
			goalYaw = maxRangeHeading;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
				ROS_INFO("In second rotate while loop, should be aligning with direction of longest range, goalYaw: %lf, correctedYaw: %lf.\n", goalYaw, correctedYaw);
					
				if (yaw <= 0){
					correctedYaw = (180.0-abs(yaw)) + 180.0;
				}
				else
					correctedYaw = yaw;

				rotate(LEFT,0.4);
					
				vel.angular.z = angular;
				vel.linear.x = linear;
				vel_pub.publish(vel);
			}
			//------------------Scanning complete, robot correctly oriented, set mode to EXPLORATION for next cycle-------------------------//
			//------------------Also reset linear and angular velocities so publish at the end doesn't publish junk-------------------------//	
			mode = EXPLORATION;
			storeForwardHeading == 1;
				
			angular = 0;
			linear = 0;
			scanCount = 0;
			vel.angular.z = 0;
			vel.linear.x = 0;
			vel_pub.publish(vel);
		}

		//Exploration Mode
		else if(mode == EXPLORATION){
			//if center bumper pressed, do initial. NEED TO DOUBLE CHECK INTERRUPT
			if(bumperCentre == 1){
				ROS_INFO("Bumper Hit");
				while (laserRange<0.75){
					ros::spinOnce();
					moveBackwards();
				}
				mode = INITIAL;
			}
			/*
			//avoid clipping on left			
			else if(laserArray[18]<0.75 && laserRange>0.5){
				ROS_INFO("Clipping on Left, laserArray[18]: %lf\n", laserArray[18]);
				moveForward(0.2,REGULARMODE);
				angular=-0.2;
			}
			//avoid clipping on right
			else if(laserArray[40]<0.75 && laserRange>0.5){
				ROS_INFO("Cliping on Right");
				moveForward(0.2,REGULARMODE);
				angular=0.2;
			}
			*/
			//if distance > 0.5, go forward
			else if(laserRange>0.5){
				moveForward(0.25,REGULARMODE);
				scanCount++;
				if(storeForwardHeading == 1){
					storeForwardHeading = 0;
					moveForwardHeading = correctedYaw;
				}
				if((correctedYaw - moveForwardHeading) > 1){
					angular = -0.1;
				}
				else if((correctedYaw - moveForwardHeading) < -1){
					angular = 0.1;
				}
				if(scanCount>7500){
					mode=SCAN;
					scanCount = 0;
				}
				ROS_INFO("Moving Foward, scanCount:%d",scanCount);
			}
			//if distance < 0.5, rotate until distance > 1.5
			else if(laserRange<0.5 && !cornered()){
				scanCount = 0;
				int direction = turnDirection();
				while(laserRange<1.5){
					ros::spinOnce();

					if(direction==RIGHT)
						rotate(RIGHT,0.4);
					else
						rotate(LEFT,0.4);

					vel.angular.z = angular;
					vel.linear.x = linear;

					vel_pub.publish(vel);

				}
				storeForwardHeading = 1;
				stop();
			}
			//if all distance in laser array < 0.5, got to initial mode
			else if(cornered()){
				scanCount = 0;
				ROS_INFO("Cornered");
				mode=INITIAL;
			}
			else{
				ROS_INFO("Don't know what to do");
				stop();
			}

		}



 		vel.angular.z = angular;
		vel.linear.x = linear;

		vel_pub.publish(vel);
		ROS_INFO("Starting Yaw: %lf.\n", startingYaw);
	}
	return 0;
}

//---------------------------------ARCHIVE-----------------------------------//
/*
			/-------------- Old less efficient rotation code, works but if new code works as well, should be replaced-----------------/		
			if(state == SCAN){
				if (yaw <= 0){
					correctedYaw = (M_PI-abs(yaw)) + M_PI;
					}
				else
					correctedYaw = yaw;	
				if (rotateState == 0){
					startingYaw = correctedYaw;
				}
				if (firstRotate < 3){			
					if (abs(correctedYaw-startingYaw) < ((90.0*M_PI)/180)){
						rotate(1, 0.3);
					}	
					else if (abs(correctedYaw-startingYaw) >= ((90.0*M_PI)/180)){
						stop();
						rotateState = 0;
						firstRotate = firstRotate + 1;
					}
				//temp = yaw-startingYaw;
				temp = correctedYaw-startingYaw;
				ROS_INFO("y - sy is: %lf, sy is: %lf, cy is: %lf, fr is: %d, rs is %d.", temp,startingYaw, yaw, firstRotate, rotateState);
			
				}
				else {
					stop();
					ROS_INFO("Stopping, firstRotate is: %d, cy: %lf, sy: %lf,  cy - sy: %lf, rs: %d", firstRotate, yaw, startingYaw, temp, rotateState);
				}

				ROS_INFO("yaw: %lf, corrected yaw: %lf.", yaw, correctedYaw);

			/-------------- Sprial Exloration Code, compiles but doesn't work-----------------/
			ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range %f", posX, posY, yaw, laserRange);
			if (bumperRight == 0 && bumperLeft == 0 && bumperCentre ==0){
				ros::spinOnce();
				tempYaw = (180 -abs(yaw)) + 180;
				laserinitialYaw =(180-abs(yaw))+180;
				initialposX = posX;
				initialposY = posY;
		//		tempYaw = round(currentYaw);
				lineardistance(tempposX, initialposX, tempposY, initialposY);

				//Nothing in front and not in oringal 
				if (laserRange > 0.5 && round(spiralinitialYaw) != round(tempYaw)) {
			       		while (linearDistance < 1){
				       		moveForward (0.25, 0);
			    			vel.angular.z = angular;
			 			vel.linear.x = linear;
			 			vel_pub.publish(vel);
			 			ros::spinOnce();
			       			tempposX = posX;
			       			tempposY = posY;
						lineardistance(tempposX, initialposX, tempposY, initialposY);
			        	}	
			       	initialposX = tempposX;
			       	initialposY = tempposY;
				}
				else if (laserRange <=0.5 && round(spiralinitialYaw) != round(tempYaw)){
					while (abs(round(tempYaw) - round(laserinitialYaw)) < laserturn){
						rotate(1,0.3);
						vel.angular.z = angular;
						vel.linear.x = linear;
						vel_pub.publish(vel);
						ros::spinOnce();
						tempYaw = (180 -abs(yaw))+180;
					}
					ros::spinOnce();
					laserinitialYaw = round(tempYaw);
				}
				else if (round(spiralinitialYaw) == round(tempYaw)){
					while (abs(round(tempYaw)-round(spiralinitialYaw)) < spiralturn && spiralturn <= 60){
						rotate(1,0.3);
						vel.angular.z = angular;
						vel.linear.x = linear;
						vel_pub.publish(vel);
						ros::spinOnce();
						tempYaw = (180 - abs(yaw))+180;
						spiralturn = spiralturn + 10;
						if (spiralturn > 60){
							stop();
						}	
					
					}
				spiralinitialYaw = (180 - abs(yaw)) + 180;
				tempYaw = 0;
				}
			}
			ROS_INFO("tempYaw: %f degrees, initialYaw: %f degrees, tempposX: %f, tempposY: %f, initialposX: %f, initialposY: %f, laserturn: %f degrees, spiralturn: %f", tempYaw, initialYaw, tempposX, tempposY, initialposX, initialposY, laserturn, spiralturn);

void lineardistance(double tempposX, double initialposX, double tempposY, double initialposY){
	linearDistance = abs(sqrt((tempposX - initialposX)*(tempposX - initialposX) + (tempposY - initialposY)*(tempposY - initialposY)));
}*/
