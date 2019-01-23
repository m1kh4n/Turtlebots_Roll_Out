#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;

double posX, posY, yaw;
double pi=3.1416;

void odomCallback (const navmsgs::Odometry::ConstPtr& msg){
    posx = msg->pose.pose.position.x;
    posy = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

bool bumperLeft = 0, bumperCenter = 0, bumperRight =0;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    if(msg.bumper == 0)
        bumperLeft = !bumperLeft;
    else if(msg.bumper == 1)
        bumperCenter = !bumperCenter;
    else if(msg.bumper == 2)
        bumperRight = !bumperRight;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //fill with your code
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    teleController eStop;
    
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    
    double angular = 0.0;
    double linear = 0.0;
    geometry_msgs::Twist vel;
    
    while(ros::ok()){
        ros::spinOnce();
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................
        
        //fill with your code
        ROS_INFO(“Position: (%f, %f) Orientation: %f rad or %f degrees.”, posX, posY, yaw, yaw*180/pi);
        ROS_INFO(“Laser Range: %i”, laserRange);
        
        vel.angular.z = angular;
        vel.linear.x = linear;
        
        vel_pub.publish(vel);
    }
    
    return 0;
}
