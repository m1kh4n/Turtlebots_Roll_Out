//#define CALCULATE_SHORTEST 1
//#define MOVEMENT 1

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <string.h>
#include <iostream>

//Global Variables
Boxes start;
Boxes boxes;
const int nodes = 6; //Starting Point considered a node
int startIndex = nodes-1; //Starting Point declared as the last Index
std::vector <float> optimalPath; //Index 0 stores mininmum cost, rest of vector stores sequence
float costMap[nodes][nodes];
float deltaX, deltaY;

//Helper Functions
std::vector <float> find_minPath(int unvisited[],int currentNode){
    //Count number of unvisited Nodes
    int unvisitedCount=0;
    for(int i=0;i<nodes;i++){
        unvisitedCount+=unvisited[i];
    }
    //At Bottom of the Tree
    if(unvisitedCount==1){
        int lastUnvisited;
        for(int i=0;i<nodes;i++){
            if(unvisited[i]==1)
                lastUnvisited = i;
        }
        std::vector <float> minPath;
        minPath.push_back(costMap[currentNode][lastUnvisited]+costMap[lastUnvisited][startIndex]);
        minPath.push_back((float) startIndex);
        minPath.push_back((float) lastUnvisited);
        return minPath;
    }
    
    //At Intermediate Levels of the Tree
    else{
        //Initialize minCost to large cost and next optimal node as nextNode
        float minCost = 99999;
        float nextNode;
        std::vector <float> minPath;
        for(int i=0;i<nodes;i++){
            if(unvisited[i]==1){
                //Initialize newUnvisited Array for branch
                int newUnvisited[nodes];
                for(int j=0;j<nodes;j++){
                    newUnvisited[j]=unvisited[j];
                }
                newUnvisited[i] = 0;
                //Find cost of current path
                std::vector <float> currentPath = find_minPath(newUnvisited,i);
                float cost = currentPath[0]+costMap[currentNode][i];
                //If current path more optimal, store it
                if (cost<minCost){
                    minCost = cost;
                    nextNode = i;
                    minPath = currentPath;
                }
            }
        }
        //Update minPath
        minPath.push_back(nextNode);
        minPath[0]+=costMap[(int)nextNode][currentNode];
        if (unvisitedCount==nodes-1)
            minPath.push_back(startIndex);
        
        return minPath;
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
    for(int i=0;i<nodes;i++){
	    for(int j=0;j<nodes;j++){
		    if(i==j){
			    deltaX=0;
			    deltaY=0;
		    }		  
		    else if(i==startIndex){
			    deltaX = startX - boxes.coords[j][0];
			    deltaY = startY - boxes.coords[j][1];
				
		    }
		    else if(j==startIndex){
			    deltaX=boxes.coords[i][0]-startX;
			    deltaY=boxes.coords[i][1]-startY;
		    }
		    else{
			  deltaX=boxes.coords[i][0]-boxes.coords[j][0];
			  deltaY=boxes.coords[i][1]-boxes.coords[j][1];
		    }
	    	    costMap[i][j]=sqrt(deltaX*deltaX+deltaY*deltaY);
	    }
    }
#ifdef CALCULATE_SHORTEST
    //Shortest Sequence Algorithm Using DP
    int unvisited[nodes];
    for (int i=0; i<nodes;i++){
        unvisited[i]=1;
    }
    unvisited[startIndex] = 0;
    optimalPath  = find_minPath(unvisited,startIndex);

    ROS_INFO("Optimal Sequence: ");
    for(int i=1;i<optimalPath.size();i++){
	    ROS_INFO("%f",optimalPath[i]);
    }
    ROS_INFO("Minimum Cost: %f", optimalPath[0]);
#endif
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
#ifdef MOVEMENT
	int object;
	for(int i=1;i<optimalPath.size()-1;i++){
		object =(int) optimalPath[i];
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

	//Move Back to Starting Position
	Navigation::moveToGoal(startX,startY,startPhi);
#endif
	imagePipeline.getTemplateID(boxes);
	ros::Duration(1).sleep();
    }
    return 0;
}

/* -------------------------------------------ARCHIVE--------------------------------------------

 //Original Shortest Path Algorithm using C
 float minPath(int unvisited[],int object){
	//Count number of unvisited Nodes
	int unvisitedCount=0;
       	for(int i=0;i<nodes;i++){
	       	unvisitedCount+=unvisited[i];
       	}
	//At Bottom of the Tree
	if(unvisitedCount==1){
		int lastUnvisited;
		for(int i=0;i<nodes;i++){
	       		if(unvisited[i]==0)
				lastUnvisited = i;
   	    	}
		seq[nodes-unvisitedCount] = lastUnvisited;
		return distance[object,lastUnvisited]+distance[lastUnvisited,startIndex];
	}
	//At Intermediate Levels of the Tree
	else{
		float minCost = 99999;
		for(int i=0;i<nodes;i++){
			if(unvisited[i]==1){
				int newUnvisited[nodes];
				newUnvisited = memcpy(newUnvisited,unvisited,nodes);
				newUnvisited[i] = 0;
				cost = minPath(newUnvisited,i)+distance[object,i];
				if (cost<minCost){
					minCost = 
					seq[nodes-unvisitedCount] = i;
				}
			}
		}
		return minCost;
	}
}
 */

