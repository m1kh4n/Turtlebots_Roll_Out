#include <imagePipeline.h>
#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc.hpp"

#define PRINT_IMAGE
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

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

using namespace cv;
using namespace cv::xfeatures2d;

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        template_id = 3;
        Mat Raisin = boxes.templates[0];
        Mat Cinnamon = boxes.templates[1];
        Mat Rice = boxes.templates[2];
    
        int recogTemplates[4];
        for(int i = 0; i < 4; i++){
            recogTemplates[i] = 0;
        }

        //-- Detecting keypoints and descriptors of test image and template images
        //-- Detecting keypoints and descriptors of test image and template images
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypointsTestImage, keypointsRaisin, keypointsRice, keypointsCinnamon;
        detector->detect(img, keypointsTestImage);
        detector->detect(Raisin, keypointsRaisin);
        detector->detect(Rice, keypointsRice);
        detector->detect(Cinnamon, keypointsCinnamon);
        
        Ptr<SURF> extractor = SURF::create();
        Mat descriptorTestImage;
        Mat descriptorRaisin;
        Mat descriptorRice;
        Mat descriptorCinnamon;
        extractor->compute(img, keypointsTestImage, descriptorTestImage);
        extractor->compute(Raisin, keypointsRaisin, descriptorRaisin);
        extractor->compute(Rice, keypointsRice, descriptorRice);
        extractor->compute(Cinnamon, keypointsCinnamon, descriptorCinnamon);
        //cv::imshow("view", Raisin);
        //cv::waitKey(10);
        
        //-- Identifying which image it best matches
        int notEnoughRaisinMatches = 0;
        int notEnoughRiceMatches = 0;
        int notEnoughCinnamonMatches = 0;
        //-- Testing Raisin Bran First
        //-- Step 1: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector<DMatch> matches;
        matcher.match(descriptorRaisin, descriptorTestImage, matches);
        
        double max_dist = 0; double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptorRaisin.rows; i++ ){
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        
        //printf("-- Max dist : %f \n", max_dist );
        //printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small
        //-- PS.- radiusMatch can also be used here.
        std::vector<DMatch>good_matches;
        
        for(int i = 0; i < descriptorRaisin.rows; i++){
            if( matches[i].distance <= max(2*min_dist, 0.02) ){
                good_matches.push_back( matches[i]);
            }
        }
        
        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches(Raisin, keypointsRaisin, img, keypointsTestImage,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        if(good_matches.size() < 10) notEnoughRaisinMatches = 1;
        
        //-- Localise the Object
        std::vector<Point2f>obj;
        std::vector<Point2f>scene;
        
        int HMRDFlag = 0; int CNRFlag = 0; int DCRFlag = 0;
        int rows; int cols;
        double conditionNumber;
        Mat homographyMatrix, singularValues, U, Vt = Mat();
        std::vector<Point2f> obj_corners(4);
        std::vector<Point2f> scene_corners(4);
        Size s;
        if(!notEnoughRaisinMatches){
            
            for(int i = 0; i<good_matches.size(); i++){
                //-- Get the keypoints from the good matches
                obj.push_back(keypointsRaisin[good_matches[i].queryIdx].pt);
                scene.push_back(keypointsTestImage[good_matches[i].trainIdx].pt);
            }
            
            //-- Get homography matrix
            homographyMatrix = findHomography(obj,scene,RANSAC);
            
            //-- Get the corners from the image_1 ( the object to be "detected" )
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(Raisin.cols, 0);
            obj_corners[2] = cvPoint(Raisin.cols, Raisin.rows); obj_corners[3] = cvPoint(0, Raisin.rows);
            
            perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
            
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( Raisin.cols, 0), scene_corners[1] + Point2f( Raisin.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( Raisin.cols, 0), scene_corners[2] + Point2f( Raisin.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( Raisin.cols, 0), scene_corners[3] + Point2f( Raisin.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( Raisin.cols, 0), scene_corners[0] + Point2f( Raisin.cols, 0), Scalar( 0, 255, 0), 4 );
            
            //-- Show detected matches
            //imshow( "Good Matches & Homography Calculation", img_matches );
            
            for( int i = 0; i < (int)good_matches.size(); i++ ){
                //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
            }
            
            std::cout << "Homography Matrix = " << std::endl << " " << homographyMatrix << std::endl;
            
            //-- Try and determine a good match from bad match through homography matrix analysis
            DCRFlag = 1;
            //-- Check determinant of the matrix to see if its too close to zero
            double HMRDeterminant = determinant(homographyMatrix);
            if (HMRDeterminant>0.1) HMRDFlag = 1;
            else HMRDFlag = 0;
            std::cout << "Determinant of Raisin Bran matrix is: " << HMRDeterminant << std::endl;
            
            //-- DO SVD on homography matrix and check its values
            singularValues = Mat();
            U, Vt = Mat();
            CNRFlag = 0;
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
                CNRFlag = 1;
            }
            
#ifdef DIAGONAL_CHECK
            double mainDiagComp;
            double offDiagComp;
            DCRFlag = 0;
            mainDiagComp = abs((abs(homographyMatrix.at<double>(0,0))-abs(homographyMatrix.at<double>(1,1))));
            offDiagComp = abs((abs(homographyMatrix.at<double>(0,1))-abs(homographyMatrix.at<double>(1,0))));
            if(mainDiagComp <= 0.1){
                if(offDiagComp <= 0.1){
                    std::cout << "Homography Matrix is bad" << endl;
                }
            }
            else{
                std::cout<< "Homography matrix passes diagonal check" << endl;
                DCRFlag = 1;
            }
            std::cout << "Main diagonal comparison is: " << mainDiagComp << " Off diagonal comparison is : " << offDiagComp << endl;
#endif
        }
        //-- Testing Rice Krispies
        //-- Step 1: Matching descriptor vectors using FLANN matches
        matches.clear();
        matcher.match(descriptorRice, descriptorTestImage, matches);
        
        max_dist = 0; min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptorRice.rows; i++ ){
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        
        //printf("-- Max dist : %f \n", max_dist );
        //printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small
        //-- PS.- radiusMatch can also be used here.
        good_matches.clear();
        
        for(int i = 0; i < descriptorRice.rows; i++){
            if( matches[i].distance <= max(2*min_dist, 0.02) ){
                good_matches.push_back( matches[i]);
            }
        }
        
        //-- Draw only "good" matches
        drawMatches(Rice, keypointsRice, img, keypointsTestImage,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        if(good_matches.size() < 10) notEnoughRiceMatches = 1;
        int HMRKDFlag = 0; int CNRKFlag = 0; int DCRKFlag = 0;
        if(!notEnoughRiceMatches){
            
            //-- Localise the Object
            obj.clear();
            scene.clear();
            
            for(int i = 0; i<good_matches.size(); i++){
                //-- Get the keypoints from the good matches
                obj.push_back(keypointsRice[good_matches[i].queryIdx].pt);
                scene.push_back(keypointsTestImage[good_matches[i].trainIdx].pt);
            }
            
            //-- Get homography matrix
            homographyMatrix = findHomography(obj,scene,RANSAC);
            
            //-- Get the corners from the image_1 ( the object to be "detected" )
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(Rice.cols, 0);
            obj_corners[2] = cvPoint(Rice.cols, Rice.rows); obj_corners[3] = cvPoint(0, Rice.rows);
            
            perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
            
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( Rice.cols, 0), scene_corners[1] + Point2f( Rice.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( Rice.cols, 0), scene_corners[2] + Point2f( Rice.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( Rice.cols, 0), scene_corners[3] + Point2f( Rice.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( Rice.cols, 0), scene_corners[0] + Point2f( Rice.cols, 0), Scalar( 0, 255, 0), 4 );
            
            //-- Show detected matches
            //imshow( "Good Matches & Homography Calculation", img_matches );
            
            for( int i = 0; i < (int)good_matches.size(); i++ ){
                //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
            }
            
            std::cout << "Homography Matrix = " << std::endl << " " << homographyMatrix << std::endl;
            
            //-- Try and determine a good match from bad match through homography matrix analysis
            DCRKFlag = 1;
            //-- Check determinant of the matrix to see if its too close to zero
            double HMRKDeterminant = determinant(homographyMatrix);
            if (HMRKDeterminant>0.1) HMRKDFlag = 1;
            else HMRKDFlag = 0;
            std::cout << "Determinant of Rice Krispies matrix is: " << HMRKDeterminant << std::endl;
            
            //-- DO SVD on homography matrix and check its values
            singularValues = Mat();
            U, Vt = Mat();
            CNRKFlag = 0;
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
                CNRKFlag = 1;
            }
            
#ifdef DIAGONAL_CHECK
            double mainDiagComp;
            double offDiagComp;
            DCRKFlag = 0;
            mainDiagComp = abs((abs(homographyMatrix.at<double>(0,0))-abs(homographyMatrix.at<double>(1,1))));
            offDiagComp = abs((abs(homographyMatrix.at<double>(0,1))-abs(homographyMatrix.at<double>(1,0))));
            if(mainDiagComp <= 0.1){
                if(offDiagComp <= 0.1){
                    std::cout << "Homography Matrix is bad" << endl;
                }
            }
            else{
                std::cout<< "Homography matrix passes diagonal check" << endl;
                DCRKFlag = 1;
            }
            std::cout << "Main diagonal comparison is: " << mainDiagComp << " Off diagonal comparison is : " << offDiagComp << endl;
#endif
        }
        
        //-- Testing Cinnamon Toast Crunch
        //-- Step 1: Matching descriptor vectors using FLANN matches
        matches.clear();
        matcher.match(descriptorCinnamon, descriptorTestImage, matches);
        
        max_dist = 0; min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptorCinnamon.rows; i++ ){
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        
        //printf("-- Max dist : %f \n", max_dist );
        //printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small
        //-- PS.- radiusMatch can also be used here.
        good_matches.clear();
        
        for(int i = 0; i < descriptorCinnamon.rows; i++){
            if( matches[i].distance <= max(2*min_dist, 0.02) ){
                good_matches.push_back( matches[i]);
            }
        }
        
        //-- Draw only "good" matches
        drawMatches(Cinnamon, keypointsCinnamon, img, keypointsTestImage,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        if(good_matches.size() < 10) notEnoughCinnamonMatches = 1;
        int HMCFlag = 0; int CNCFlag = 0; int DCCFlag = 0;
        if(!notEnoughCinnamonMatches){
            //-- Localise the Object
            obj.clear();
            scene.clear();
            
            for(int i = 0; i<good_matches.size(); i++){
                //-- Get the keypoints from the good matches
                obj.push_back(keypointsCinnamon[good_matches[i].queryIdx].pt);
                scene.push_back(keypointsTestImage[good_matches[i].trainIdx].pt);
            }
            
            //-- Get homography matrix
            homographyMatrix = findHomography(obj,scene,RANSAC);
            
            //-- Get the corners from the image_1 ( the object to be "detected" )
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(Cinnamon.cols, 0);
            obj_corners[2] = cvPoint(Cinnamon.cols, Cinnamon.rows); obj_corners[3] = cvPoint(0, Cinnamon.rows);
            
            perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
            
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( Cinnamon.cols, 0), scene_corners[1] + Point2f( Cinnamon.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( Cinnamon.cols, 0), scene_corners[2] + Point2f( Cinnamon.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( Cinnamon.cols, 0), scene_corners[3] + Point2f( Cinnamon.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( Cinnamon.cols, 0), scene_corners[0] + Point2f( Cinnamon.cols, 0), Scalar( 0, 255, 0), 4 );
            
            //-- Show detected matches
            //imshow( "Good Matches & Homography Calculation", img_matches );
            
            for( int i = 0; i < (int)good_matches.size(); i++ ){
                //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
            }
            
            std::cout << "Homography Matrix = " << std::endl << " " << homographyMatrix << std::endl;
            
            //-- Try and determine a good match from bad match through homography matrix analysis
            DCCFlag = 1;
            //-- Check determinant of the matrix to see if its too close to zero
            double HMCDeterminant = determinant(homographyMatrix);
            if (HMCDeterminant>0.1) HMCFlag = 1;
            else HMCFlag = 0;
            std::cout << "Determinant of Cinnamon Toast Crunch  matrix is: " << HMCDeterminant << std::endl;
            
            //-- DO SVD on homography matrix and check its values
            singularValues = Mat();
            U, Vt = Mat();
            CNCFlag = 0;
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
                CNCFlag = 1;
            }
            
#ifdef DIAGONAL_CHECK
            double mainDiagComp;
            double offDiagComp;
            DCCFlag = 0;
            mainDiagComp = abs((abs(homographyMatrix.at<double>(0,0))-abs(homographyMatrix.at<double>(1,1))));
            offDiagComp = abs((abs(homographyMatrix.at<double>(0,1))-abs(homographyMatrix.at<double>(1,0))));
            if(mainDiagComp <= 0.1){
                if(offDiagComp <= 0.1){
                    std::cout << "Homography Matrix is bad" << endl;
                }
            }
            else{
                std::cout<< "Homography matrix passes diagonal check" << endl;
                DCCFlag = 1;
            }
            std::cout << "Main diagonal comparison is: " << mainDiagComp << " Off diagonal comparison is : " << offDiagComp << endl;
#endif
        }
        //-- Checking all the flags to determine which template best matches the scene image
        if(HMRDFlag == 1 && CNRFlag == 1 && DCRFlag == 1 && notEnoughRaisinMatches == 0){
            std::cout << "Homography matrix is good, probable Raisin Bran match" << std::endl;
            recogTemplates[0] = 1;
#ifdef PRINT_IMAGE
            matcher.match(descriptorRaisin, descriptorTestImage, matches);
            max_dist = 0; min_dist = 100;
            for( int i = 0; i < descriptorRaisin.rows; i++ ){
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            
            std::vector<DMatch>good_matches_raisin;
            
            for(int i = 0; i < descriptorRaisin.rows; i++){
                if( matches[i].distance <= max(2*min_dist, 0.02) ){
                    good_matches_raisin.push_back( matches[i]);
                }
            }
            
            drawMatches(Raisin, keypointsRaisin, img, keypointsTestImage,
                        good_matches_raisin, img_matches, Scalar::all(-1), Scalar::all(-1),
                        std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            
            obj.clear();
            scene.clear();
            
            for(int i = 0; i<good_matches_raisin.size(); i++){
                //-- Get the keypoints from the good matches
                obj.push_back(keypointsRaisin[good_matches_raisin[i].queryIdx].pt);
                scene.push_back(keypointsTestImage[good_matches_raisin[i].trainIdx].pt);
            }
            
            homographyMatrix = findHomography(obj,scene,RANSAC);
            
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(Raisin.cols, 0);
            obj_corners[2] = cvPoint(Raisin.cols, Raisin.rows); obj_corners[3] = cvPoint(0, Raisin.rows);
            
            perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
            
            line( img_matches, scene_corners[0] + Point2f( Raisin.cols, 0), scene_corners[1] + Point2f( Raisin.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( Raisin.cols, 0), scene_corners[2] + Point2f( Raisin.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( Raisin.cols, 0), scene_corners[3] + Point2f( Raisin.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( Raisin.cols, 0), scene_corners[0] + Point2f( Raisin.cols, 0), Scalar( 0, 255, 0), 4 );
            
            imshow( "Good Matches & Homography Calculation", img_matches );
            
            for( int i = 0; i < (int)good_matches_raisin.size(); i++ ){
                printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches_raisin[i].queryIdx, good_matches_raisin[i].trainIdx );
            }
#endif
        }
        else std::cout << "Homography matrix is bad, no Raisen Bran match." << std::endl;
        
        if(HMRKDFlag == 1 && CNRKFlag == 1 && DCRKFlag == 1 && notEnoughRiceMatches == 0){
            std::cout << "Homography matrix is good, probable Rice Krispies match" << std::endl;
            recogTemplates[1] = 1;
#ifdef PRINT_IMAGE
            matcher.match(descriptorRice, descriptorTestImage, matches);
            
            max_dist = 0; min_dist = 100;
            
            for( int i = 0; i < descriptorRice.rows; i++ ){
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            
            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );
            
            std::vector<DMatch>good_matches_rice;
            
            for(int i = 0; i < descriptorRice.rows; i++){
                if( matches[i].distance <= max(2*min_dist, 0.02) ){
                    good_matches_rice.push_back( matches[i]);
                }
            }
            drawMatches(Rice, keypointsRice, img, keypointsTestImage,
                        good_matches_rice, img_matches, Scalar::all(-1), Scalar::all(-1),
                        std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            
            obj.clear();
            scene.clear();
            
            for(int i = 0; i<good_matches_rice.size(); i++){
                //-- Get the keypoints from the good matches
                obj.push_back(keypointsRice[good_matches_rice[i].queryIdx].pt);
                scene.push_back(keypointsTestImage[good_matches_rice[i].trainIdx].pt);
            }
            
            homographyMatrix = findHomography(obj,scene,RANSAC);
            
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(Rice.cols, 0);
            obj_corners[2] = cvPoint(Rice.cols, Rice.rows); obj_corners[3] = cvPoint(0, Rice.rows);
            
            perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
            
            line( img_matches, scene_corners[0] + Point2f( Rice.cols, 0), scene_corners[1] + Point2f( Rice.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( Rice.cols, 0), scene_corners[2] + Point2f( Rice.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( Rice.cols, 0), scene_corners[3] + Point2f( Rice.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( Rice.cols, 0), scene_corners[0] + Point2f( Rice.cols, 0), Scalar( 0, 255, 0), 4 );
            
            imshow( "Good Matches & Homography Calculation", img_matches );
            
            for( int i = 0; i < (int)good_matches_rice.size(); i++ ){
                printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
            }
#endif
        }
        else std::cout << "Homography matrix is bad, no Rice Krispies match." << std::endl;
        
        if(HMCFlag == 1 && CNCFlag == 1 && DCCFlag == 1 && notEnoughCinnamonMatches == 0){
            std::cout << "Homography matrix is good, probable Cinnamon Toast Crunch match" << std::endl;
            recogTemplates[2] = 1;
#ifdef PRINT_IMAGE
            matcher.match(descriptorCinnamon, descriptorTestImage, matches);
            
            max_dist = 0; min_dist = 100;
            for( int i = 0; i < descriptorCinnamon.rows; i++ ){
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            
            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );
            
            std::vector<DMatch>good_matches_cinnamon;
            
            for(int i = 0; i < descriptorCinnamon.rows; i++){
                if( matches[i].distance <= max(2*min_dist, 0.02) ){
                    good_matches_cinnamon.push_back( matches[i]);
                }
            }
            
            drawMatches(Cinnamon, keypointsCinnamon, img, keypointsTestImage,
                        good_matches_cinnamon, img_matches, Scalar::all(-1), Scalar::all(-1),
                        std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            
            obj.clear();
            scene.clear();
            
            for(int i = 0; i<good_matches_cinnamon.size(); i++){
                //-- Get the keypoints from the good matches
                obj.push_back(keypointsCinnamon[good_matches_cinnamon[i].queryIdx].pt);
                scene.push_back(keypointsTestImage[good_matches_cinnamon[i].trainIdx].pt);
            }
            
            homographyMatrix = findHomography(obj,scene,RANSAC);
            
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(Cinnamon.cols, 0);
            obj_corners[2] = cvPoint(Cinnamon.cols, Cinnamon.rows); obj_corners[3] = cvPoint(0, Cinnamon.rows);
            
            perspectiveTransform( obj_corners, scene_corners, homographyMatrix);
            
            line( img_matches, scene_corners[0] + Point2f( Cinnamon.cols, 0), scene_corners[1] + Point2f( Cinnamon.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( Cinnamon.cols, 0), scene_corners[2] + Point2f( Cinnamon.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( Cinnamon.cols, 0), scene_corners[3] + Point2f( Cinnamon.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( Cinnamon.cols, 0), scene_corners[0] + Point2f( Cinnamon.cols, 0), Scalar( 0, 255, 0), 4 );
            
            imshow( "Good Matches & Homography Calculation", img_matches );
            
            for( int i = 0; i < (int)good_matches_cinnamon.size(); i++ ){
                printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches_cinnamon[i].queryIdx, good_matches_cinnamon[i].trainIdx );
            }
#endif
        }
        else std::cout << "Homography matrix is bad, no Cinnamon Toast Crunch match." << std::endl;
        for(int i = 0; i < 4; i++){
            //std::cout << recogTemplates[i] << std::endl;
	    if (recogTemplates[i] == 1) template_id = i;
        }
    }
    //waitKey(0);
    return template_id;
}
