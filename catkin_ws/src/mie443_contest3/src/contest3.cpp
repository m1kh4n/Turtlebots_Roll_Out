#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
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
	    bumper.center = !bumper.center;
    else if(msg->bumper ==2)
	    bumper.right = !bumper.right;
}
struct Wheel{
	bool right,left;
};
struct Wheel wheel = {0,0};

void wheeldropCB(const kobuki_msgs::WheelDropEvent::ConstPtr msg){
	if (msg-> wheel == 1)
		wheel.right = !wheel.right;
	else if(msg->wheel == 0)
		wheel.left = !wheel.left;
}

//Helper Functions
bool isLost(){
	if(vel.angular.z==0 && vel.linear.x && vel.linear.y ==0)
		return true;
	else
		return false;
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
            clock_t t = clock();
            bool bumperRelease = false;
            bool seePlant = false;
            
            while((clock()-t)<3){
                //if whithin 3 seconds, the obstacle is removed break.
                if(bumper.left == 0 || bumper.right == 0 || bumper.centre == 0){
                    bumperRelease = true;
                    break;
                }
                /*
                 if(see plant code here){
                 seePlant = true;
                 break;
                 }
                 */
            }
            if (bumperRelease = false && seePlant = false){
                world_state = 2;
            }
            else if (bumperRelease = true && seePlant = false){
                world_state = 0;
            }
            else if (seePlant = true){
                world_state = 3;
            }
            
            
            
            
		}
		else if(0){
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
				ros::spinOnce();
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
					ros::spinOnce();
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
			//sc.playWave(path_to_sounds+"sad";
		}
	}

	return 0;
}
                     
// Image Detection Code, put wherever necessary
{
    int matchFlag = 0;
    cv::Mat sceneImage = imageTransporter.getImg();
    cv::Mat plant = imread("/path/to/image.jpg", IMREAD_GRAYSCALE);
    
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    vector<KeyPoint> keypointsSceneImage, keypointsPlant;
    detector->detect(sceneImage, keypointsSceneImage);
    detector->detect(plant, keypointsPlant);
    
    Ptr<SURF> extractor = SURF::create();
    Mat descriptorSceneImage;
    Mat descriptorPlant;
    extractor->compute(sceneImage, keypointsSceneImage, descriptorSceneImage);
    extractor->compute(plant, keypointsPlant, descriptorPlant);
    
    int notEnoughMatches = 0;
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptorPlant, descriptorSceneImage, matches);
    
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descriptorPlant.rows; i++ ){
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    std::vector<DMatch>good_matches;
    
    for(int i = 0; i < descriptorPlant.rows; i++){
        if( matches[i].distance <= max(2*min_dist, 0.02) ){
            good_matches.push_back( matches[i]);
        }
    }
    Mat img_matches;
    drawMatches(plant, keypointsPlant, sceneImage, keypointsSceneImage,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    
    if(good_matches.size() < 10) notEnoughMatches = 1;
    std::vector<Point2f>obj;
    std::vector<Point2f>scene;

    int HMDFlag = 0; int CNFlag = 0;
    int rows; int cols;
    double conditionNumber;
    Mat homographyMatrix, singularValues, U, Vt = Mat();
    std::vector<Point2f> obj_corners(4);
    std::vector<Point2f> scene_corners(4);
    Size s;
    
    if(!notEnoughMatches){
        
        for(int i = 0; i<good_matches.size(); i++){
            //-- Get the keypoints from the good matches
            obj.push_back(keypointsPlant[good_matches[i].queryIdx].pt);
            scene.push_back(keypointsSceneImage[good_matches[i].trainIdx].pt);
        }
        
        //-- Get homography matrix
        homographyMatrix = findHomography(obj,scene,RANSAC);
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(plant.cols, 0);
        obj_corners[2] = cvPoint(plant.cols, plant.rows); obj_corners[3] = cvPoint(0, plant.rows);
        
        perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + Point2f( plant.cols, 0), scene_corners[1] + Point2f( plant.cols, 0), Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + Point2f( plant.cols, 0), scene_corners[2] + Point2f( plant.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + Point2f( plant.cols, 0), scene_corners[3] + Point2f( plant.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + Point2f( plant.cols, 0), scene_corners[0] + Point2f( plant.cols, 0), Scalar( 0, 255, 0), 4 );
        
        //-- Show detected matches
        //imshow( "Good Matches & Homography Calculation", img_matches );
        
        for( int i = 0; i < (int)good_matches.size(); i++ ){
            //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
        }
        
        std::cout << "Homography Matrix = " << std::endl << " " << homographyMatrix << std::endl;
        
        //-- Check determinant of the matrix to see if its too close to zero
        double HMDeterminant = determinant(homographyMatrix);
        if (HMDeterminant>0.1) HMDFlag = 1;
        else HMDFlag = 0;
        std::cout << "Determinant of matrix is: " << HMDeterminant << std::endl;
        
        //-- DO SVD on homography matrix and check its values
        singularValues = Mat();
        U, Vt = Mat();
        CNFlag = 0;
        SVDecomp(homographyMatrix, singularValues, U, Vt, 2);
        
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
    if(CNFlag == 1 && HMDFlag == 1) matchFlag = 1;
}
