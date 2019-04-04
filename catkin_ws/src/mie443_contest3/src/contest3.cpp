#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;


void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

struct Bumper{
	bool center, right, left;
};
struct Bumper bumper = {0,0,0};

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr msg){
    //Fill with code
    if(msg->bumper == 0)
	    bumper.left = !bumper.left;
    else if(msg->bumper == 1)
	    bumper.centre = !bumper.centre;
    else if(msg->bumper ==2)
	    bumper.right = !bumper.right;
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		// eStop.block();
		//...................................
		
		//Sense (in priority)
		if(bumper.left == 1 || bumper.right == 1 || bumper.centre == 1){
			world_state  = 1;
		}
		else if{
			//
			world_state = 2;
		}
		else if{
			//
			world_state = 3;
		}
		else if{
			//
			world_state = 4;
		}
		else{
			//keep following person
			world_state = 0;
		}

	
		//Emotion
		if(world_state == 0){
		//	fill with your code
			vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);
		//	ROS_INFO("follow_cmd = %d",follow_cmd);
		        sleep(0.5);

			sc.playWave(path_to_sounds+"sound.wav");
			sleep(1.0);

			sc.stopWave(path_to_sounds+"sound.wav");

		}
		else if(world_state == 1){
			/*
			...
			...
			*/
		}
		else if(world_state == 2){

		}
		else if(world_state == 3){

		}
		else if(world_state ==4){

		}
	}

	return 0;
}
