#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Execute strategy - Optimate Sequence then Navigate to catpure image
    //Part 1 - Squence Optimatization (TSP)
    int seq[5];
    for(int i=0;i<5;i++){
	    seq[i]=i;
    }

    //Part 2 - Navigate and capture image
    float xGoal,yGoal;
    float offsetFactor = 0.5;
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
	/*Hints:
        - Use: boxes.coords[object index][x=0,y=1,phi=2]
        - Use: robotPose.x, robotPose.y, robotPose.phi
	*/
	for(int i=0;i<5;i++){
		object = seq[i];
		phiGoal = boxes.coords[object][2];
		xGoal = boxes.coords[object][0]-offsetFactor*cos(phiGoal);
		yGoal = boxes.coords[object][1]-offsetFactor*sin(phiGoal);
		Navigation::moveToGoal(xGoal,yGoal,phiGoal);
		//imagePipeline.getTemplateID(boxes);
        	ros::Duration(2).sleep();
	}	
    }
    return 0;
}
