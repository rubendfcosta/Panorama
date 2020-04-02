/**
 * @file main.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief Turn an algorithm a ros package
 * @version 0.1
 * @date 2019-04-17
 *
 * @copyright Copyright (c) 2019
 *
 */

/*Opencv*/
#include <opencv/cv.h>
#include <opencv/highgui.h>

/*Mensagens*/
//#include <geometry_msgs/Point32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

/*ROS*/
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <iostream>
#include <string>

//#include "advanced_lane_detection/laneDetection.h"

/*Namesapaces*/
using namespace std;
using namespace ros;
using namespace cv;

// // Global Variables
// // Img size 964*724:
// // Point2f perspectiveSrc[] = {Point2f( 340, 412), Point2f(535, 412),
// // Point2f(88, 700), Point2f(858, 700)}; Point2f perspectiveDst[] =
// // {Point2f(226, 0), Point2f(737, 0), Point2f(226, 724), Point2f(737, 724)};
// // Img size 640*480
// // Point2f perspectiveSrc[] = {Point2f(125, 262), Point2f(400, 262), Point2f(44,
// // 453), Point2f(550, 453)}; Point2f perspectiveDst[] = {Point2f(126, 0),
// // Point2f(426, 0), Point2f(126, 480), Point2f(426, 480)};
// // Img size (320*240)
// // Point2f perspectiveSrc[] = {Point2f(83, 61), Point2f(220, 61), Point2f(42,
// // 300), Point2f(270, 300)}; Point2f perspectiveDst[] = {Point2f(63, 0),
// // Point2f(263, 0), Point2f(63, 240), Point2f(263, 240)};

// // Onboard Atlas
// // Point2f perspectiveSrc[] = {Point2f(370, 412), Point2f(535, 412),
// //                             Point2f(88, 700), Point2f(858, 700)};
// // Point2f perspectiveDst[] = {Point2f(226, 0), Point2f(737, 0), Point2f(226, 724),
// //                             Point2f(737, 724)};

// // Point2f perspectiveSrc[] = {Point2f(130, 135), Point2f(160, 135), Point2f(60,
// // 220), Point2f(310, 220)}; Point2f perspectiveDst[] = {Point2f(63, 0),
// // Point2f(263, 0), Point2f(63, 240), Point2f(263, 240)};

class alg2 {
public:
  alg2();
  void Looping();

private:
  ros::NodeHandle n;

  /*Important Variables*/
  Mat imgPerspective;
  Mat perspectiveMatrix; // Homography Matrix.
  Mat warpEdge;
  Mat imageRedChannel;
  Mat redBinary;
  Mat mergeImage;
  Mat histImage;
  Mat warpMask;
  Mat maskImage;
  Mat finalResult;
  Size frameSize;
  Mat result;
  int cols_resize = 964; // default parameters
  int rows_resize = 724; // default parameters

  // bool info_set = false;

  /*ROS*/
  image_transport::ImageTransport it;

  /*Publishers && Subs*/
  ros::Publisher initial_image;
  ros::Publisher FL_image;
  ros::Publisher FM_image;
  ros::Publisher FR_image;
  ros::Publisher poly_image;
  ros::Subscriber camInfo;

  image_transport::Subscriber sub_imgFL;
  image_transport::Subscriber sub_imgFM;
  image_transport::Subscriber sub_imgFR;

  /*Images pointers reveive*/
  cv_bridge::CvImagePtr current_imageFL;
  cv_bridge::CvImagePtr current_imageFM;
  cv_bridge::CvImagePtr current_imageFR;

  /*Images messages*/
  sensor_msgs::ImagePtr img_FL;
  sensor_msgs::ImagePtr img_FM;
  sensor_msgs::ImagePtr img_FR;
  sensor_msgs::ImagePtr poly_draw;
  sensor_msgs::ImagePtr img_init;


  /*Functions*/
  void Publishers();
  void receiveInitImgFL(const sensor_msgs::ImageConstPtr &img);
  void receiveInitImgFM(const sensor_msgs::ImageConstPtr &img);
  void receiveInitImgFR(const sensor_msgs::ImageConstPtr &img);
  void processFrames();
};

/**
 * @brief Class constructor
 *
 */

alg2::alg2() : it(n) {
  // Resize image
  ros::param::get("~cols_resize", cols_resize);
  ros::param::get("~rows_resize", rows_resize);

  /*Subscribers*/
  sub_imgFL =
      it.subscribe("/FL_camera/image_raw", 10, &alg2::receiveInitImgFL, this);
  sub_imgFM =
      it.subscribe("/FM_camera/image_raw", 10, &alg2::receiveInitImgFM, this);
  sub_imgFR =
      it.subscribe("/FR_camera/image_raw", 10, &alg2::receiveInitImgFR, this);


   FL_image =
       n.advertise<sensor_msgs::Image>("/TESTE/FL_IMAGE", 10);
   FM_image =
       n.advertise<sensor_msgs::Image>("/TESTE/FM_IMAGE", 10);
   FR_image =
       n.advertise<sensor_msgs::Image>("/TESTE/FR_IMAGE", 10);
}

/**
 * @brief Loop that runs the main functions
 *
 */

void alg2::Looping() {
  if (current_imageFL && current_imageFM && current_imageFR) {
    //processFrames();
    Publishers();
  }
}

/**
 * @brief Publishers function
 *
 * @param i
 */
void alg2::Publishers() {
  if (current_imageFL && current_imageFM && current_imageFR) {
    FL_image.publish(current_imageFL->toImageMsg());
    FM_image.publish(current_imageFM->toImageMsg());
    FR_image.publish(current_imageFR->toImageMsg());
    //initial_image.publish(img_init);
  }
}

/**
 * @brief Callback that receives the image from the camera already retified
 *
 * @param img
 */
void alg2::receiveInitImgFL(const sensor_msgs::ImageConstPtr &img) {
  try {
    current_imageFL =
        cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }
}

/**
 * @brief Callback that receives the image from the camera already retified
 *
 * @param img
 */
void alg2::receiveInitImgFM(const sensor_msgs::ImageConstPtr &img) {
  try {
    current_imageFM =
        cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }
}

/**
 * @brief Callback that receives the image from the camera already retified
 *
 * @param img
 */
void alg2::receiveInitImgFR(const sensor_msgs::ImageConstPtr &img) {
  try {
    current_imageFR =
        cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }
}

/**
 * @brief process algorithm application
 *
 */

void alg2::processFrames() {
  //SEM NADA PARA JA 
}


//Parece estar bem 
void DetectAndDescribe(Mat image, vector<KeyPoint>& keypoints, Mat& descriptor) {
    //create the greyscale image
    Mat grayImage;
    cv::cvtColor(image, grayImage, COLOR_BGR2GRAY);

    //create the keypoint detector and descriptor as "feature" using SIFT
    Ptr<Feature2D> feature = xfeatures2d::SIFT::create();

    //create a matrix of keypoints using feature
    feature->detect(grayImage, keypoints);

    //create a maatrix of descriptors using feature and keypoints
    feature->compute(grayImage, keypoints, descriptor);
}

//Parece estar bem 
Mat matchKeypoints(Mat imageA, Mat imageB, vector<KeyPoint> keypointA, vector<KeyPoint> keypointB, Mat featuresA, Mat featuresB, float ratio, double repojThresh){
    //create a vector of vector to hold raw matches
    vector<vector<DMatch>> rawMatches;

    //create a vector of DMatches to hold good matches
    vector<DMatch> goodMatches;

    //create two vector points to hold the points where the lines will be drawn
    vector<Point2f> pointsA;
    vector<Point2f> pointsB;

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    matcher->knnMatch(featuresA, featuresB, rawMatches, 2);

    goodMatches.reserve(rawMatches.size());

    for (size_t i = 0; i < rawMatches.size(); i++)
    {
        if ((rawMatches[i].size()==2) && (rawMatches[i][0].distance < (rawMatches[i][1].distance*ratio)))
        {
            goodMatches.push_back(rawMatches[i][0]);
        }
    }

    cv::KeyPoint::convert(keypointA, pointsA);
    cv::KeyPoint::convert(keypointB, pointsB);

    if (goodMatches.size() > 25) {
        Mat homographyM = findHomography(pointsA, pointsB, RANSAC, repojThresh);
        return(goodMatches, homographyM);
    }
}

//nao necessariA
Mat drawMatches(Mat imageA, Mat imageB, vector<KeyPoint> keypointsA, vector<KeyPoint> keypointsB, vector<DMatch> matches) {
    //initialize the output visualization image
    float hA = imageA.size().height;
    float wA = imageA.size().width;

    float hB = imageB.size().height;
    float wB = imageB.size().width;

    Mat resultImage = Mat(fmax(hA, hB), wA + wB, 3, "uint8");

    //connect lines between the selected points
    Point2f pointA;
    Point2f pointB;

    for (int i=0; i<matches.size();i++) {
        pointA = Point2f(keypointsA[matches[i].queryIdx].pt.x, keypointsA[matches[i].queryIdx].pt.y);
        pointB = Point2f(keypointsA[matches[i].trainIdx].pt.x+ wA, keypointsB[matches[i].trainIdx].pt.y);

        cv::line(resultImage, pointA, pointB, (0, 255, 0), 1);
    }

    return resultImage;
}

Mat stitch(Mat imageL, Mat imageM,Mat imageR, float ratio, double repojThresh, bool showMatches) {
    vector<KeyPoint> keypointL;
    vector<KeyPoint> keypointM;
    vector<KeyPoint> keypointR;
    Mat featuresL;
    Mat featuresM;
    Mat featuresR;
    Mat matchFeatures;
    Mat matchesLM;
    Mat matchesMR;
    Mat homographyLM = {{0.0, 0.0, 0.0} ,
                        {0.0, 0.0, 0.0} ,
                        {0.0, 0.0, 0.0} };
    Mat homographyMR = {{0.0, 0.0, 0.0} ,
                        {0.0, 0.0, 0.0} ,
                        {0.0, 0.0, 0.0} };
    Mat result;

    if (~homographyLM && ~homographyMR)
    {
      (keypointL, featuresA) = DetectAndDescribe(imageL);
      (keypointM, featuresB) = DetectAndDescribe(imageM);
      (keypointR, featuresB) = DetectAndDescribe(imageR);

      (matchesLM, homographyLM) = matchKeypoints(imageL, imageM, keypointL, keypointM, featuresL, featuresM, ratio, repojThresh);
      (matchesMR, homographyMR) = matchKeypoints(imageM, imageR, keypointM, keypointR, featuresM, featuresR, ratio, repojThresh);

      if (matchesLM && matchesRM)
      {return -1;}

      homographyLM = matchesLM[1];
      homographyMR = matchesMR[1]; 
    }

    int result_width = 1920;
    float hL = imageL.size().height;
    float wL = imageL.size().width;
    float hM = imageM.size().height;
    float wM = imageM.size().width;
    float hR = imageR.size().height;
    float wR = imageR.size().width;


    float T[3][3] = {  {1.0, 0.0, ((result_width/2)-(wL/2))} ,  /*  initializers for row indexed by 0 */
                       {0.0, 1.0, 0.0} ,                        /*  initializers for row indexed by 1 */
                       {0.0, 0.0, 1.0} };                       /*  initializers for row indexed by 2 */
    
    vector()


    for (i = 0; i < sizeof(images); i++)
    {
        cv::warpPerspective(images[i], result, homographyM, Size(wA+wB, hA));
    }
    
    

    //Point a cv::Mat header at it (no allocation is done)
    Mat final(Size(imageB.cols * 2 + imageB.cols, imageA.rows * 2), CV_8UC3);

    //velikost img1
    Mat roi1(final, Rect(0, 0, imageB.cols, imageB.rows));
    Mat roi2(final, Rect(0, 0, result.cols, result.rows));
    result.copyTo(roi2);
    imageB.copyTo(roi1);


    // if (showMatches) {
    //     Mat visiblelines = drawMatches(imageA, imageB, keypointA, keypointB, matches);
    //     return (result, visiblelines);
    // }

    return result;
}


/**
 * @brief Main function
 *
 */

int main(int argc, char **argv) {

  ros::init(argc, argv, "TESTE");

  alg2 processImage;
  ros::Rate loop_rate(8.2);

  while (ros::ok()) {
    processImage.Looping();
    ros::spinOnce();
    loop_rate.sleep();
    try
    {
      if (alg2::FL_image && alg2::FM_image && alg2::FR_image)
      {
        result = stitch(alg2::FL_image, alg2::FM_image, alg2::FR_image, 0.75, 4.0, true);

        if (~result)
        { print("There was an error in the stitching procedure");
        }else
        {
          FL_image.publish(current_imageFL->toImageMsg());
          print("A Publicar a panoramica");
        }  
      }else
      {
        continue;
      } 
    }
    catch()
    {
      print("Shutting down")
    } 
  }

  return 0;
}