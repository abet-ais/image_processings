/**********************************************************************
   File    face_detection.cpp
   Author  Takahiro Yamazaki  and abe
   Environment    ROS_kinetic
   OS       Ubuntu 16.04 LTS
   StartDay 2019/3/7
   FinishDay 2019/3/7
**********************************************************************/
/**********************************************************************
   Problems to be fixed
   -
**********************************************************************/


/**********************************************************************
   Include Libraries
**********************************************************************/
//include
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <numeric>


/**********************************************************************
   Declare variables
**********************************************************************/
#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480



/**********************************************************************
   Globle
**********************************************************************/
cv::Mat color_raw;
cv::Mat depth_raw;
cv::Mat display_color;
cv::Mat display_depth;

/**********************************************************************
   Proto_type_Declare functions
**********************************************************************/
void image_detect(const sensor_msgs::ImageConstPtr& rgb_image,const sensor_msgs::ImageConstPtr& depth_image);
cv::Mat detectFaceInImage(cv::Mat &image, std::string& filename);



/**********************************************************************
   Typedef
**********************************************************************/
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;



/**********************************************************************
   Func detectFaceInImage
**********************************************************************/
cv::Mat detectFaceInImage(cv::Mat &image, std::string &cascade_file){
  cv::CascadeClassifier cascade;
  cascade.load(cascade_file);
  std::vector<cv::Rect> faces;
  cascade.detectMultiScale(image, faces, 1.1, 3, 0, cv::Size(20,20));

  for(int i=0; i<faces.size(); i++){
    rectangle(image, cv::Point(faces[i].x, faces[i].y), 
  	      cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
  	      cv::Scalar(0,200,0),
  	      3,
  	      CV_AA);
  }
  return image;
}



/**********************************************************************
   Func image_detect
**********************************************************************/
void image_detect(const sensor_msgs::ImageConstPtr& rgb_image,const sensor_msgs::ImageConstPtr& depth_image){

  /*--- カメラのデータをopencvで扱える型に変更 ---*/
  cv_bridge::CvImagePtr cv_rgb;
  try{
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  color_raw  = cv_rgb->image;
  display_color = color_raw.clone();
  
  /*--- カメラdepthデータをopencvで扱える型に変更 ---*/
  cv_bridge::CvImagePtr cv_depth;
  try{
    cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s",e.what());
    return;
  }
  depth_raw = cv_depth->image;

  std::string filename = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml";

  cv::Mat detect_FaceImage= detectFaceInImage(color_raw, filename);

  cv::imshow("display_color", detect_FaceImage);
  cv::waitKey(10);
}


/**********************************************************************
   Main
**********************************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "image_reading");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image(nh,"/camera/rgb/image_raw",1);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_image(nh,"/camera/depth/image_raw",1);
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync;
  sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(50);
  sync->connectInput(sub_rgb_image, sub_depth_image);
  sync->registerCallback(image_detect);

  cv::namedWindow("display_color", CV_WINDOW_NORMAL);
  cv::resizeWindow("display_color", WINDOW_WIDTH, WINDOW_HEIGHT);

  ros::spin();
  return 0;
}
