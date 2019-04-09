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
double desiredAngle = 7.5;
double desiredAngleRad = desiredAngle*pi/180.0;
double laserArray[3]; //laserArray[-15deg distance, 0deg distance, 15deg distance]

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/(msg->angle_increment);
	laserOffset = desiredAngleRad/(msg->angle_increment);
	laserRange = 11.0;
	laserArray[0] = 11.0;
	laserArray[1] = 11.0;
	laserArray[2] = 11.0;

	//Store Laser Array 
	for(int j=1;j<4;j++){
		for(int i = j*laserSize/4 - laserOffset; i < j*laserSize/4 + laserOffset; i++){
			if(laserArray[j-1] > msg->ranges[i]){
				laserArray[j-1] = msg->ranges[i];
			}
		}

		if(laserArray[j-1] == 11)
			laserArray[j-1] = 0;
	}
	

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
	//rotateState = 1;
	//ROS_INFO("Robot turning with speed of: %lf.", angular);
}

int  turnDirection(){
	if(laserArray[0]<laserArray[2])
		return RIGHT;
	else
		return LEFT;	
}


bool cornered(){
	if(laserArray[0]<0.5 && laserArray[2]<0.5)
		return true;
	else
		return false;
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
	int scanCount = 0;

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

			//-------------------Rotation code, based on while loops and modulus of 360-------------------------------------------------//
			
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
				
			//------------------Reorienting robot to the yaw which had the longest range, -------------------//
			goalYaw = maxRangeHeading;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
				ROS_INFO("In second rotate while loop, should be aligning with direction of longest range, goalYaw: %lf, correctedYaw: %lf.\n", goalYaw, correctedYaw);
					
				if (yaw <= 0){
					correctedYaw = (180.0-abs(yaw)) + 180.0;
				}
				else
					correctedYaw = yaw;

				rotate(LEFT,0.5);
					
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
			desiredRotation = 90; 
			
			double maxRange = 0; 
			double maxRangeHeading = 0;

					
			goalYaw = correctedYaw + desiredRotation;
			if(goalYaw > 360.0)
				goalYaw = goalYaw - 360.0;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
					
				ROS_INFO("In first scan while loop, publishing data, goalYaw: %lf, correctedYaw: %lf, difference: %lf.\n", goalYaw, correctedYaw, abs(goalYaw-correctedYaw));
					
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
				
			goalYaw = maxRangeHeading;
			while(abs(goalYaw-correctedYaw) > 1){
				ros::spinOnce();
				ROS_INFO("In second scan while loop, should be aligning with direction of longest range, goalYaw: %lf, correctedYaw: %lf.\n", goalYaw, correctedYaw);
					
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
			//if center bumper pressed, move back a little and do initial.
			if(bumperCentre == 1){
				ROS_INFO("Bumper Hit");
				while (laserRange<0.5){
					ros::spinOnce();
					moveBackwards();

					vel.angular.z = angular;
					vel.linear.x = linear;

					vel_pub.publish(vel);

				}
				scanCount = 0;
				mode = INITIAL;
			}
			
			//if left bumper pressed, move backwards until left distance greater than 0.5
			else if(bumperLeft == 1){
				ROS_INFO("Bumper Left Hit");
				while (laserArray[2]<0.75){
					ros::spinOnce();
					moveBackwards();
					angular = -0.1;
					ROS_INFO("laserArray[0] value: %lf", laserArray[0]);				
					vel.angular.z = angular;
					vel.linear.x = linear;

					vel_pub.publish(vel);
					storeForwardHeading = 1;
					scanCount = 0;
				}
			}
			
			//if right bumper pressed, move backwards until right distance greater than 0.5
			else if(bumperRight == 1){
				ROS_INFO("Bumper Right Hit");
				while (laserArray[0]<0.75){
					ros::spinOnce();
					moveBackwards();
					angular = 0.1;
											
					ROS_INFO("laserArray[0] value: %lf", laserArray[0]);
					vel.angular.z = angular;
					vel.linear.x = linear;

					vel_pub.publish(vel);
					storeForwardHeading = 1;
				}
				scanCount =0;
			}
			
			//if distance > 0.5, go forward
			else if(laserRange>0.5){
				moveForward(0.25,REGULARMODE);
				scanCount++;
				
				/*
				//Flag for deciding when to store the initial forward heading, set to = 1 by other functions and set to 0 first time the moving forward state is entered
				//to ensure that only the yaw angle during the first loop is stored
				if(storeForwardHeading == 1){
					storeForwardHeading = 0;
					moveForwardHeading = correctedYaw;
				}
				
				//Checking if the robot is veering away from the initial yaw stored during the first call of moveForward. Adds an angular speed if the robot is deviating
				//more than 1 degree from the initial yaw
				if((correctedYaw - moveForwardHeading) > 1){
					angular = -0.1;
				}
				else if((correctedYaw - moveForwardHeading) < -1){
					angular = 0.1;
				}
				*/

				//Decide when to enter scan mode by checking scanCount. scanCount is reset when enter any other condition. 
				
				if(scanCount>7500){
					mode=SCAN;
					scanCount = 0;
				}
				//ROS_INFO("Moving Foward, scanCount:%d",scanCount);
				ROS_INFO("Moving forward, laserArray values are: %lf, %lf, %lf, %lf", laserArray[0], laserArray[1], laserArray[2], laserRange);
			}

			//if distance < 0.5, rotate until distance > 1.5
			else if(laserRange<0.5 /*&& !cornered()*/){
				scanCount = 0;
				storeForwardHeading = 1;

				
				//determine direction to turn
				int direction = turnDirection();

				//turn until see distance greater than 1.0
				while(laserRange<1.0){
					ros::spinOnce();

					if(direction==RIGHT)
						rotate(LEFT,0.4);
					else
						rotate(RIGHT,0.4);

					vel.angular.z = angular;
					vel.linear.x = linear;

					vel_pub.publish(vel);
					ROS_INFO("Turning, laserArray values are: %lf, %lf, %lf, %lf", laserArray[0], laserArray[1], laserArray[2], laserRange);
				}
				

				stop();
				ROS_INFO("Turning to find open space");
			}
			/*
			//if cornered, go to initial mode
			else if(cornered()){
				scanCount = 0;
				ROS_INFO("Cornered, laserArray values are: %lf, %lf, %lf, %lf", laserArray[0], laserArray[1], laserArray[2], laserRange);
				//mode=INITIAL;
			}*/
			
			//when lost, got back to initial mode
			else{
				ROS_INFO("Don't know what to do");
				mode = INITIAL;
			}
		}


 		vel.angular.z = angular;
		vel.linear.x = linear;

		vel_pub.publish(vel);
		ROS_INFO("Starting Yaw: %lf.\n", startingYaw);
		ROS_INFO("Cornered, laserArray values are: %lf, %lf, %lf, %lf", laserArray[0], laserArray[1], laserArray[2], laserRange);

	}
	return 0;
}

//---------------------------------ARCHIVE-----------------------------------//
/*
 			/ --------------- Decided to use yaw for control  ---------------------------------------------------------------------------/
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
//
//
/*
			/-------------- Old less efficient rotation code, works but if new code works as well, should be replaced-----------------/	
			int rotateState;
			int firstRotate;
			double temp;
	
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
