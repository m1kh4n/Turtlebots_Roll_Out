#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <string.h>

//Global Variables
Boxes start;
int startIndex = boxes.coords.size();
int seq[boxes.coords.size()+1];
float distance[boxes.coords.size()+1][boxes.coords.size()+1];
float deltaX, deltaY;

//Helper Functions
float minPath(int unvisited[],int object){
	//Count number of unvisited Nodes
	int unvisitedCount=0;
       	for(int i=0;i<sizeof(unvisited)/sizeof(int);i++){
	       	unvisitedCount+=unvisited[i];
       	}
	//At Bottom of the Tree
	if(unvisitedCount==1){
		int lastUnvisited;
		for(int i=0;i<sizeof(unvisited)/sizeof(int);i++){
	       		if(unvisited[i]==0)
				lastUnvisited = i;
   	    	}
		seq[boxes.coords.size()-1] = lastUnvisited;
		return distance[object,lastUnvisited]+distance[lastUnvisited,boxes.coords.size()];
	}
	//At Intermediate Levels of the Tree
	else{
		float minCost = 99999;
		for(int i=0;i<sizeof(unvisited)/sizeof(int);i++){
			if(unvisited[i]==1){
				int newUnvisited[boxes.coords.size()+1];
				newUnvisited = memcpy(newUnvisited,unvisited,boxes.coords.size()+1];
				newUnvisited[i] = 0;
				cost = minPath(newUnvisited,i)+distance(object,i);
				if (cost<minCost){
					minCost = cost;
					seq[boxes.coords.size()-unvisitedCount] = i;
				}
			}
		}
		return minCost;
	}
}

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
    float startX = robotPose.x;
    float startY = robotPose.y; 
    float startPhi = robotPose.phi;

    //Initialize Distance Map
    for(int i=0;i<boxes.coords.size+1;i++){
	    for(int j=0;j<boxes.coords.size+1;j++){
		    if(i==j){
			    deltaX=0;
			    deltaY=0;
		    }		  
		    else if(i==boxes.coords.size()){
			    deltaX = startX - boxes.coords[j][0];
			    deltaY = startY - boxes.coords[j][1];
				
		    }
		    else if(j==boxes.coords.size()){
			    deltaX=boxes.coords[i][0]-startX;
			    deltaY=boxes.coords[i][1]-startY;
		    }
		    else{
			  deltaX=boxes.coords[i][0]-boxes.coords[j][0];
			  deltaY=boxes.coords[i][1]-boxes.coords[j][1];
		    }
	    	    distance[i][j]=sqrt(deltaX*deltaX+deltaY*deltaY);
	    }
    }
    //Shortest Sequence Algorithm Using DP
    unvisited[boxes.coords.size()+1]= {1};
    seq[0] = startIndex;
    seq[boxes.coords.size()]=startIndex;
    totalCost = minCost(unvisited,startIndex);

    /*
    //For Testing Navigation
    for(int i=0;i<5;i++){
	    seq[i]=i;
    }
    */

    //Part 2 - Navigate and capture image
    float xGoal,yGoal,phiGoal;
    float offsetFactor = 0.4;
    //bool rotateflag = 0
    //int angleIncrement = 1; // 1 degree angle increment (in rad)
    //int targetRotate = 30; // angle to which robot rotates to when in front of image to adjust position.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
	/*Hints:
        - Use: boxes.coords[object index][x=0,y=1,phi=2]
        - Use: robotPose.x, robotPose.y, robotPose.phi
	*/
	for(int i=0;i<boxes.coords.size();i++){
		object = seq[i];
		phiGoal = boxes.coords[object][2]-3.14;
		xGoal = boxes.coords[object][0]-offsetFactor*cos(phiGoal);
		yGoal = boxes.coords[object][1]-offsetFactor*sin(phiGoal);
		Navigation::moveToGoal(xGoal,yGoal,phiGoal);
		//imagePipeline.getTemplateID(boxes);
        // assuming we get a flag from imagepipeline that image is not clear. Robot will rotate +45 and -45 degrees
        
       /* while (imagePipeline.getTemplateID(boxes) == -1) {
            if (rotateflag == 0){ // rotate right
                angleIncrement +=1;
                phiGoal += angleIncrement*3.14/180;
                Navigation::moveToGoal(xGoal,yGoal,phiGoal);
                if (angleIncrement == targetRotate){ // if we have reached max rotate angle, we want to rotate left.
                    rotateflag = 1;
                    Navigation::moveToGoal(xGoal,yGoal,phiGoal - angleIncrement*(3.14/180)); // reset robot position to original
                    angleIncrement = 0;
                }

            }
            if (rotateflag == 1){ // rotate left
                angleIncrement -= 1;
                phiGoal += angleIncrement*3.14/180;
                Navigation::moveToGoal(xGoal,yGoal,phiGoal);
            }

            imagepipeline.getTemplateID(boxes);
            if (angleIncrement == (-1*targetRotate))
                ROS_INFO("Robot has rotated both sides and cannot find image");
            imagePipeline.getTemplateID(boxes);
        }
       */     
        	ros::Duration(2).sleep();
	}


    }
    return 0;
}
