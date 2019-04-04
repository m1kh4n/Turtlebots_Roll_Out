#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <time.h>

using namespace std;

//Global Variables
geometry_msgs::Twist follow_cmd;
int world_state;

// Callback Function
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
struct Wheel{
	bool right,left;
};
struct Wheel Wheel = {0,0};

<<<<<<< HEAD
void wheeldropCB(const kobuki_msgs::WheelDropEvent::ConstPtr msg){
	if (msg-> wheel == 1)
		wheel.right = !wheel.right;
	else if(msg->wheel == 0)
		wheel.left = !wheel.left;
}
=======
//Helper Functions
bool isLost(){
	if(vel.angular.z==0 && vel.linear.x && vel.linear.y ==0)
		return true;
	else
		return false;
}

>>>>>>> 49012f6b7956d5576904f8b23029185ebe4a37ec
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
	ros::Subscriber wheeldrop = nh.subscribe("mobile_base/events/wheeldrop",10,&wheeldropCB)

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
		if(wheel.right == 1 || wheel.left == 1){
			world_state  = 1;
		}
		else if(bumper.left == 1 || bumper.right == 1 || bumper.centre == 1){
			//
			world_state = 2;
		}
		else if{
			//
			world_state = 3;
		}
		else if(isLost()){
			//When loose track of person
			clock_t t = clock();
			bool foundPerson = false;

			//look left for 2 seconds
			while((clock()-t) < 2){
				vel.angular.z=1;
				vel_pub.publish(vel);
				ros::spinOnce;
				if(isLost()==false){
					foundPerson = true;
					break;
			}
			if(foundPerson == false){
				//Display almost crying 
			
				//look right for 4 seconds
				t = clock();
				while((clock()-t) < 4){
					vel.angular.z=-1;
					vel.pub.publish(vel);
					ros::spinOnce;
					if(isLost()==true){
						foundPerson = true;
						break
					}
				}
				//if still can't find person, do world_state 4 emotion
				if(foundPerson == false)
					world_state = 4;
			}
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
			if (wheel.left == 1 && wheel.right == 0){
				//tilted right suprised image
				sc.playWave(path_to_sounds+"alert.wav"); // change .wav file to suprised sound clip
				sleep(1.0); // if we want soundclip to loop. set time to length of clip
			}
			else if (wheel.left == 0 && wheel.right == 1){
				//tilted left suprised image
				sc.playWave(path_to_sounds+"alert.wav"); // change .wav file to suprised sound clip
				sleep(1.0); // if we want soundclip to loop. set time to length of clip
			}
			else if (wheel.left == 1 && wheel.right == 1){
				// normal suprised image
				vel.linear.x = 1
				vel_pub.publish(vel);
				sleep(3.0); // dont want to spin wheels endlessly otherwise when robot is placed on ground it will move.
				
				sc.playWave(path_to_sounds+"alert.wav"); // change .wav file to suprised sound clip
				sleep(1.0); // if we want soundclip to loop. set time to length of clip
			}
			else if (wheel.left == 0 && wheel.right == 0 {
				world_state = 0
			}


		}
		else if(world_state == 2){

		}
		else if(world_state == 3){

		}
		else if(world_state ==4){
			//Sad
			//Display 'sad' image
			
			//Play 'sad' sounds
			sc.playWave(path_to_sounds+"sad";
		}
	}

	return 0;
}
