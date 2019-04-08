#define CALCULATE_SHORTEST 1
#define MOVEMENT 1

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>

//Global Variables
Boxes start;
Boxes boxes;
const int nodes = 6; //Starting Point considered a node
int startIndex = nodes-1; //Starting Point declared as the last Index
std::vector <float> optimalPath; //Index 0 stores mininmum cost, rest of vector stores sequence
float costMap[nodes][nodes];
float deltaX, deltaY;
int locationTag[nodes-1];
const float scanRange = 10*3.14/180; //Scans 45 degrees in each direction
const float scanIncrement = 5*3.14/180;

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

    // Initialize image object and subscriber.
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
    float offsetFactor = 0.7;
    
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
	float objectFloat;
	int object;
	ROS_INFO("Starting Navigation Sequence to locate tags");
	for(int i=2;i<=nodes;i++){
        	//Calculate where to navigate to
		objectFloat = optimalPath[i];
		object = static_cast<int>(objectFloat);
		std::cout << "PhiGoal: " << boxes.coords[object][2] << std::endl;
		phiGoal = boxes.coords[object][2]-3.14;
		xGoal = boxes.coords[object][0]-offsetFactor*cos(phiGoal);
		yGoal = boxes.coords[object][1]-offsetFactor*sin(phiGoal);
		ROS_INFO("Moving to box:%d",object);
	
        	//Navigate to Oject and Scan Images
        	int scanData[4]={0};
        	int scanData_index=0;
        	int currentTag;
        	for (float phiOffset=-scanRange;phiOffset<scanRange;phiOffset+=scanIncrement){
            		Navigation::moveToGoal(xGoal,yGoal,phiGoal+phiOffset);
            		ros::spinOnce();
			currentTag=imagePipeline.getTemplateID(boxes);
            		ROS_INFO("Scan %d: Tag = %d",scanData_index,currentTag);
            		scanData_index++;
			if(currentTag==-1)
				ROS_INFO("IMAGEPIPELINE ERROR");
			else if (currentTag==3)
            			scanData[currentTag]++;
			else
				scanData[currentTag]+=2;

			if(scanData[currentTag]>4)
				break;		
        	}
        
        	//Store most likely Tag
		int predictedTag=0;
		for (int j=1;j<4;j++){
			if(scanData[j]>scanData[predictedTag])
				predictedTag=j;
		}
        	locationTag[object]=predictedTag;
        	ROS_INFO("At box %d, Tag Determined to be %d",object,locationTag[object]);
       	 	ros::Duration(2).sleep();
	}

	//Print out File
	std::ofstream resultFile;
        resultFile.open("Contest2_Results");
        for(int i;i<nodes-1;i++){
            resultFile <<"Box " << i << "->";
            switch(locationTag[i]){
                case 0: resultFile << "Raisin Bran\n"; break;
                case 1: resultFile << "Rice Krispie\n"; break;
                case 2: resultFile << "Cinnamon Toast Crunch\n"; break;
                case 3: resultFile << "Empty\n"; break;
            }
        }
        resultFile.close();

	//Move Back to Starting Position and Output file
	Navigation::moveToGoal(startX,startY,startPhi);

#endif
#ifndef MOVEMENT
	int scanData[4]={0};
        int scanData_index=0;
        int currentTag;
        for (int i = 0;i<(int)(2*scanRange/scanIncrement+1);i++){
            	ros::spinOnce();
		currentTag=imagePipeline.getTemplateID(boxes);
            	ROS_INFO("Scan %d: Tag = %d",scanData_index,currentTag);
            	scanData_index++;
		if(currentTag!=-1)
            		scanData[currentTag]++;
		else
			ROS_INFO("IMAGEPIPELINE ERROR");
        	}
        
        //Store most likely Tag
	int predictedTag=0;
	for (int j=1;j<4;j++){
		if(scanData[j]>scanData[predictedTag])
			predictedTag=j;
		}
        locationTag[0]=predictedTag;
        ROS_INFO("Tag Determined to be %d",locationTag[0]);	
#endif
#ifdef MOVEMENT
        
	ros::Duration(100).sleep();
#endif
    }
    return 0;
}

/* -------------------------------------------ARCHIVE--------------------------------------------
1. Shortest Path Algorithum using C
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

2. Rotation Code
 // assuming we get a flag from imagepipeline that image is not clear. Robot will rotate +45 and -45 degrees
 
 while (scanComplete==0) {
 if (rotateflag == 0){ // rotate right
 angleIncrement +=5*3.14/180;
 Navigation::moveToGoal(xGoal,yGoal,phiGoal+angleIncrement);
 if (angleIncrement == targetRotate){ // if we have reached max rotate angle, we want to rotate left.
 rotateflag = 1;
 angleIncrement = 0;
 }
 
 }
 if (rotateflag == 1){ // rotate left
 angleIncrement -= 1f;
 phiGoal += angleIncrement*3.14/180;
 Navigation::moveToGoal(xGoal,yGoal,phiGoal);
 if(angleIncre
 }
 
 imagepipeline.getTemplateID(boxes);
 if (angleIncrement == (-1*targetRotate))
 ROS_INFO("Robot has rotated both sides and cannot find image");
 imagePipeline.getTemplateID(boxes);
 }
 */
 

