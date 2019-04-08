//
//  imagePipeline.cpp
//  
//
//  Created by Albert Liu on 2019-04-08.
//

#include <imagePipeline.h>
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#define PRINT_IMAGE
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
        << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

cv::Mat ImagePipeline::getImg(){
    return img;
}


