#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

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
    //Part 2 - Navigate and capture image
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
	/*Hints:
        - Use: boxes.coords[object index][x=0,y=1,phi=2]
        - Use: robotPose.x, robotPose.y, robotPose.phi
	*/
	xGoal = 0;
	yGoal = 0;
	phiGoal = 0;
	
	Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal);

	xGoal = 1;
	yGoal = 1;
	phiGoal = 1;

	Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal);

        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
