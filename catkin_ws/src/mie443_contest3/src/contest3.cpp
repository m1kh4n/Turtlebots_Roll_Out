#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <imagePipeline.h>
#include <kobuki_msgs/BumperEvent.h>
#include <time.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc.hpp"
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <kobuki_msgs/WheelDropEvent.h>

//For Debugging
//#define STATE2AND3
//#define STATE4

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//Global Variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel;
int world_state;

//Load Images
cv::Mat follow = imread("/images/plant.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat surprised = imread("/images/plant.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat angry = imread("/images/plant.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat happy = imread("/images/plant.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat sad1 = imread("/images/plant.jpg",CV_LOAD_IMAGE_COLOR);
cv::Mat sad2 = imread("/images/plant.jpg",CV_LOAD_IMAGE_COLOR);

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
bool wheelLeft=0;

void wheeldropCB(const kobuki_msgs::WheelDropEvent::ConstPtr msg){
	if (msg-> state == 1){
        if (msg-> wheel == 1){
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
}

//Helper Functions
bool isLost(){
	if(vel.angular.z==0 && vel.linear.x && vel.linear.y ==0)
		return true;
	else
		return false;
}

//Main Function
int main(int argc, char **argv)
{
    //Initialize and Declare Ros Topic Variables
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	//ros::NodeHandle n;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//Initialize Publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//Initialize Subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
    ros::Subscriber wheeldrop = nh.subscribe("mobile_base/events/wheeldrop",10,&wheeldropCB);

    //Initialize imageTransport and imagePipeline
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	ImagePipeline imagePipeline(nh);

    //Initialize Variables at Startup
    int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
    
    //Play startup sound and image
	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();
    cv::namedWindow("Display Window",WINDOW_AUTOSIZE);
    cv::imshow("Display Window",angry);
    cv::waitKey(30);

    //Turtlebot Operation Loop
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		// eStop.block();
		//...................................

		//Wall-E Senses
        //1. Detect if Wall-E picked up
		if(wheelRight == 1 || wheelLeft == 1){
            ROS_INFO("Help! I'm picked up");
			world_state = 1;
		}
        //2&3.Detect if Wall-E hit into something. Then check if plant or obstacle in front
#ifdef STATE2AND3
		else if(bumperState.left == 1 || bumperState.right == 1 || bumperState.centre == 1){
            clock_t t = clock();
            bool seePlant = false;
            
            //Stop first
            vel.angular.z=0;
            vel.linear.x=0;
            vel.linear.y=0;
            vel_pub.publish(vel);
            
            //Check if see plant for 3 seconds
            cv::Mat plant = imread("/images/plant.jpg", IMREAD_GRAYSCALE); //CHANGE IMAGE PATH
            cv::Mat descriptorPlant;
            vector<KeyPoint> keypointsPlant;
            int minHessian = 400;
            cv::Ptr<SURF> detector = SURF::create(minHessian);
            cv::Ptr<SURF> extractor = SURF::create();
            detector->detect(plant, keypointsPlant);
            extractor->compute(plant, keypointsPlant, descriptorPlant);
            while((clock()-t)<3){
                //see plant when bumper is pressed means not mad
                int seePlant = 0;
                cv::Mat sceneImage = imagePipeline.getImg();
                
                vector<KeyPoint> keypointsSceneImage;
                detector->detect(sceneImage, keypointsSceneImage);

                cv::Mat descriptorSceneImage;
                extractor->compute(sceneImage, keypointsSceneImage, descriptorSceneImage);

                int notEnoughMatches = 0;
                cv::FlannBasedMatcher matcher;
                vector<DMatch> matches;
                matcher.match(descriptorPlant, descriptorSceneImage, matches);

                double max_dist = 0; double min_dist = 100;
                for( int i = 0; i < descriptorPlant.rows; i++ ){
                    double dist = matches[i].distance;
                    if( dist < min_dist ) min_dist = dist;
                    if( dist > max_dist ) max_dist = dist;
                }
                vector<DMatch>good_matches;
                for(int i = 0; i < descriptorPlant.rows; i++){
                    if( matches[i].distance <= max(2*min_dist, 0.02) ){
                        good_matches.push_back( matches[i]);
                    }
                }
                cv::Mat img_matches;
                cv::drawMatches(plant, keypointsPlant, sceneImage, keypointsSceneImage,
                                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                if(good_matches.size() < 10) notEnoughMatches = 1;
                vector<Point2f>obj;
                vector<Point2f>scene;

                int HMDFlag = 0; int CNFlag = 0;
                int rows; int cols;
                double conditionNumber;
                cv::Mat homographyMatrix, singularValues, U, Vt = Mat();
                std::vector<Point2f> obj_corners(4);
                std::vector<Point2f> scene_corners(4);
                cv::Size s;

                if(!notEnoughMatches){

                    for(int i = 0; i<good_matches.size(); i++){
                        //-- Get the keypoints from the good matches
                        obj.push_back(keypointsPlant[good_matches[i].queryIdx].pt);
                        scene.push_back(keypointsSceneImage[good_matches[i].trainIdx].pt);
                    }

                    //-- Get homography matrix
                    homographyMatrix = cv::findHomography(obj,scene,RANSAC);

                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(plant.cols, 0);
                    obj_corners[2] = cvPoint(plant.cols, plant.rows); obj_corners[3] = cvPoint(0, plant.rows);

                    cv::perspectiveTransform( obj_corners, scene_corners, homographyMatrix);

                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                    cv::line( img_matches, scene_corners[0] + Point2f( plant.cols, 0), scene_corners[1] + Point2f( plant.cols, 0), Scalar(0, 255, 0), 4 );
                    cv::line( img_matches, scene_corners[1] + Point2f( plant.cols, 0), scene_corners[2] + Point2f( plant.cols, 0), Scalar( 0, 255, 0), 4 );
                    cv::line( img_matches, scene_corners[2] + Point2f( plant.cols, 0), scene_corners[3] + Point2f( plant.cols, 0), Scalar( 0, 255, 0), 4 );
                    cv::line( img_matches, scene_corners[3] + Point2f( plant.cols, 0), scene_corners[0] + Point2f( plant.cols, 0), Scalar( 0, 255, 0), 4 );
                    //-- Show detected matches
                    //imshow( "Good Matches & Homography Calculation", img_matches );

                    for( int i = 0; i < (int)good_matches.size(); i++ ){
                        //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
                    }

                    std::cout << "Homography Matrix = " << std::endl << " " << homographyMatrix << std::endl;

                    //-- Check determinant of the matrix to see if its too close to zero
                    double HMDeterminant = cv::determinant(homographyMatrix);
                    if (HMDeterminant>0.1) HMDFlag = 1;
                    else HMDFlag = 0;
                    std::cout << "Determinant of matrix is: " << HMDeterminant << std::endl;

                    //-- DO SVD on homography matrix and check its values
                    singularValues = cv::Mat();
                    U, Vt = cv::Mat();
                    CNFlag = 0;
                    cv::SVDecomp(homographyMatrix, singularValues, U, Vt, 2);

                    std::cout << "Printing the singular values of the homography matrix: " << std::endl;
                    s = singularValues.size();
                    rows = s.height;
                    cols = s.width;
                    for (int i = 0; i < rows; i++)
                        for(int j = 0; j < cols; j++){
                            std::cout << "Element at " << i << " and " << j << " is " << singularValues.at<double>(i,j) << std::endl;
                        }
                        conditionNumber = singularValues.at<double>(0,0)/singularValues.at<double>(2,0);
                        std::cout << "Condition number is: " << conditionNumber << std::endl;
                        if(conditionNumber <= 10000000){
                            std::cout << "Condition number check passed" << std::endl;
                            CNFlag = 1;
                        }
                }
                if(CNFlag == 1 && HMDFlag == 1) seePlant = 1;
                if(seePlant == 1) break;
            }
            
            //Check if obstacle in front or see plant
            ros::spinOnce();
            if (seePlant == 1){
                ROS_INFO("See Plant");
                world_state = 3;
            }
            else{
                ROS_INFO("Obstacle Ahead");
                world_state = 2;
            }
		}
#endif
        //Check if Wall-E lost
#ifdef STATE4
		else if(isLost()){
			//When loose track of person
			clock_t t = clock();
			bool foundPerson = false;

			//look left for 2 seconds
			while((clock()-t) < 2){
				vel.angular.z=1;
				vel_pub.publish(vel);
				ros::spinOnce();
				if(isLost()==false){
					foundPerson = true;
					break;
                }
            }
            //if still lost, look right for 4 seconds
			if(foundPerson == false){
				//Display almost crying
                cv::imshow("Display Window",sad1);
                cv::waitKey(30);

				//look right for 4 seconds
				t = clock();
				while((clock()-t) < 4){
					vel.angular.z=-1;
					vel_pub.publish(vel);
					ros::spinOnce();
					if(isLost()==true){
						foundPerson = true;
                        break;
					}
				}
				//if still can't find person, do world_state 4 emotion
                if(foundPerson == false){
                    ROS_INFO("I'm Lost");
					world_state = 4;
                }
			}
		}
#endif
        //0.If nothing sensed, keep following person
		else{
			//keep following person
			world_state = 0;
		}

		//Wall-E Emotion Reactions
        //0.Follow Person
		if(world_state == 0){
			vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);
            //	ROS_INFO("follow_cmd = %d",follow_cmd);
            
            sleep(0.25);
            /*
			sc.playWave(path_to_sounds+"follow.wav");
			sleep(0.25);
			sc.stopWave(path_to_sounds+"follow.wav");
             */

		}
        //1.Suprised
        else if(world_state == 1){
            ROS_INFO("Surprised");
            cv::imshow("Display Window",surprised);
            cv::waitKey(30);
            if (wheelLeft == 1 && wheelRight == 0){
  				//tilted right suprised image
  				sc.playWave(path_to_sounds+"suprised.wav"); // change .wav file to suprised sound clip
  				sleep(1.0);
                ROS_INFO("Left wheel is up");
  			}
  			else if (wheelLeft == 0 && wheelRight == 1){
  				//tilted left suprised image
  				sc.playWave(path_to_sounds+"surprised.wav"); // change .wav file to suprised sound clip
  				sleep(1.0);
                ROS_INFO("Right wheel is up");
  			}
  			else if (wheelLeft == 1 && wheelRight == 1){
  				// normal suprised image
                sc.playWave(path_to_sounds+"surprised.wav"); // change .wav file to suprised sound clip
                sleep(1.0);
                sc.stopWave(path_to_sounds+"surprised.wav");
                ROS_INFO("Both wheels are up");
			    clock_t t = clock();
                while((clock()-t) < 3){
  					vel.angular.x= -1;
  					vel_pub.publish(vel);
                    if (wheelLeft == 0 && wheelRight == 0)
                        break;
                }
            }
            else if (wheelLeft == 0 && wheelRight == 0) {
  				world_state = 0;
            }
		}
        //2.Angry
#ifdef STATE2AND3
		else if(world_state == 2){
            ROS_INFO("Angry");
            //Display 'angry' Image
            cv::imshow("Display Window",angry);
            cv::waitKey(30);
            
            //Play Angry Sound
            clock_t t = clock();
            sc.playWave(path_to_sounds+"angry.wav");
            
            //Move back for 1 second
            while((clock()-t) < 1){
                vel.angular.x= -1;
                vel_pub.publish(vel);
            }

            //Shake angrily 6 times.
            int j = -1;
            for (int i = 1; i<=6; i++){
                while((clock()-t) < 0.5)
                    vel.angular.z *= j;
                vel_pub.publish(vel);
            }
            sc.stopWave(path_to_sounds+"angry.wav");
		}
        //3.Excited/Happy
		else if(world_state == 3){
            ROS_INFO("Happy");
            cv::imshow("Display Window",happy);
            cv::waitKey(30);
            clock_t t = clock();
            //Display 'Happy' Image
            sc.playWave(path_to_sounds+"happy.wav");
            
            
            //Move back
            while((clock()-t) < 1){
                vel.angular.x= -1;
                vel_pub.publish(vel);
            }
            
            //Continuous rotation
            while((clock()-t) < 5){
                vel.angular.z =1;
                vel_pub.publish(vel);
            }
            sc.stopWave(path_to_sounds+"happy.wav");
		}
#endif
        //4. Sad
#ifdef STATE4
		else if(world_state ==4){
            ROS_INFO("Sad");
			//Display 'sad' image
            cv::imshow("Display Window",sad2);
            cv::waitKey(30);

			//Play 'sad' sounds
            sc.playWave(path_to_sounds+"sad.wav");
            sleep(2.0);
            sc.stopWave(path_to_sounds+"sad.wav");
		}
#endif
        
        //Reset
        cv::imshow("Display Window",follow);
        cv::waitKey(30);
        world_state=0;
	}
	return 0;
}

