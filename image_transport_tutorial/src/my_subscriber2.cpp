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

#include <iostream>
#include <string>

//#include "advanced_lane_detection/laneDetection.h"

/*Namesapaces*/
using namespace std;
using namespace ros;
using namespace cv;

// Global Variables
// Img size 964*724:
// Point2f perspectiveSrc[] = {Point2f( 340, 412), Point2f(535, 412),
// Point2f(88, 700), Point2f(858, 700)}; Point2f perspectiveDst[] =
// {Point2f(226, 0), Point2f(737, 0), Point2f(226, 724), Point2f(737, 724)};
// Img size 640*480
// Point2f perspectiveSrc[] = {Point2f(125, 262), Point2f(400, 262), Point2f(44,
// 453), Point2f(550, 453)}; Point2f perspectiveDst[] = {Point2f(126, 0),
// Point2f(426, 0), Point2f(126, 480), Point2f(426, 480)};
// Img size (320*240)
// Point2f perspectiveSrc[] = {Point2f(83, 61), Point2f(220, 61), Point2f(42,
// 300), Point2f(270, 300)}; Point2f perspectiveDst[] = {Point2f(63, 0),
// Point2f(263, 0), Point2f(63, 240), Point2f(263, 240)};

// Onboard Atlas
Point2f perspectiveSrc[] = {Point2f(370, 412), Point2f(535, 412),
                            Point2f(88, 700), Point2f(858, 700)};
Point2f perspectiveDst[] = {Point2f(226, 0), Point2f(737, 0), Point2f(226, 724),
                            Point2f(737, 724)};

// Point2f perspectiveSrc[] = {Point2f(130, 135), Point2f(160, 135), Point2f(60,
// 220), Point2f(310, 220)}; Point2f perspectiveDst[] = {Point2f(63, 0),
// Point2f(263, 0), Point2f(63, 240), Point2f(263, 240)};

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
  int cols_resize = 964; // default parameters
  int rows_resize = 724; // default parameters

  // bool info_set = false;

  /*ROS*/
  image_transport::ImageTransport it;

  /*Publishers && Subs*/
  ros::Publisher initial_image;
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

  // void callbackCamInfo(const sensor_msgs::CameraInfo::ConstPtr &cm, bool
  // *done);
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


   FL_imageP =
       n.advertise<sensor_msgs::Image>("/TESTE/FL_IMAGE", 10);
   FM_imageP =
       n.advertise<sensor_msgs::Image>("/TESTE/FM_IMAGE", 10);
   FR_imageP =
       n.advertise<sensor_msgs::Image>("/TESTE/FR_IMAGE", 10);
//   initial_image =
//       n.advertise<sensor_msgs::Image>("/advanced_algorithm/finalResult", 10);
  // camInfo = n.subscribe<sensor_msgs::CameraInfo>("/camera/camera_info", 10,
  // std::bind(readCameraInfo, std::placeholders::_1, &info_set));
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
    FL_IMAGE.publish(img_FL);
    FM_IMAGE.publish(img_FM);
    FR_IMAGE.publish(img_FR);
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
//   Size size_img(cols_resize, rows_resize);
//   Mat init_img = current_image->image;
//   resize(init_img, init_img, size_img);
//   perspectiveMatrix = getPerspectiveTransform(perspectiveSrc, perspectiveDst);
//   Mat _used_img = init_img.clone();

  // draw the roi (for perspective transform)
  // line(init_img, perspectiveSrc[0], perspectiveSrc[1], Scalar(0, 0, 255),
  // 0.01); line(init_img, perspectiveSrc[1], perspectiveSrc[3], Scalar(0, 0,
  // 255), 0.01); line(init_img, perspectiveSrc[3], perspectiveSrc[2], Scalar(0,
  // 0, 255), 0.01); line(init_img, perspectiveSrc[2], perspectiveSrc[0],
  // Scalar(0, 0, 255), 0.01); circle(init_img, perspectiveSrc[0], 0.01,
  // Scalar(0, 0, 255), CV_FILLED); circle(init_img, perspectiveSrc[1], 0.01,
  // Scalar(0, 0, 255), CV_FILLED); circle(init_img, perspectiveSrc[2], 0.01,
  // Scalar(0, 0, 255), CV_FILLED); circle(init_img, perspectiveSrc[3], 0.01,
  // Scalar(0, 0, 255), CV_FILLED);
  // frameSize = init_img.size();

  // Applying lane detection algorithm
//   laneDetection LaneAlgo(init_img, perspectiveMatrix);
//   LaneAlgo.laneDetctAlgo();

//   warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
//   imageRedChannel = LaneAlgo.getRedChannel().clone();
//   redBinary = LaneAlgo.getRedBinary().clone();
//   mergeImage = LaneAlgo.getMergeImage().clone();
//   histImage = LaneAlgo.getHistImage().clone();
//   maskImage = LaneAlgo.getMaskImage().clone();
//   warpMask = LaneAlgo.getWarpMask().clone();
//   finalResult = LaneAlgo.getFinalResult().clone();

  // init_img.convertTo(init_img,CV_8UC3);

  img_FL =cv_bridge::toCvCopy(current_imageFL, sensor_msgs::image_encodings::BGR8);
  img_FM =cv_bridge::toCvCopy(current_imageFM, sensor_msgs::image_encodings::BGR8);
  img_FR =cv_bridge::toCvCopy(current_imageFR, sensor_msgs::image_encodings::BGR8);

//   img_init = cv_bridge::CvImage{current_image->header, "bgr8", finalResult}
//                  .toImageMsg();
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
  }

  return 0;
}