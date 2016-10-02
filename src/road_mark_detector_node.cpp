#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <IPM.h>
#include <string>
/*
 * Stop Line, Crossroad Detect
 * 1. Input an undistorted image
 * 2. IPM
 * 3. Adaptive thresholding(color picking for lane)
 * 4.
 *
 */
class road_mark_detector_node{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  std::string paramTopicName;

  std::string     fisheye_camera_info_url_;
  cv::Mat         K_;
  cv::Mat         D_;
  double          balance_;

public:
  road_mark_detector_node(ros::NodeHandle _nh):nh(_nh), it(_nh){
    nh.param("topic_name", paramTopicName, std::string("image_raw"));
    image_sub = it.subscribe("image_raw", 1, &road_mark_detector_node::process, this);

    nh.param("camera_info_url", fisheye_camera_info_url_, std::string("~/.ros/camera_info/camera.yml"));
    nh.param("balance", balance_, 0.2);

  }
  void process(const sensor_msgs::ImageConstPtr &img){
    float width, height;
    std::vector<cv::Point2f> origPoints;
    origPoints.push_back( cv::Point2f(0, height) );
    origPoints.push_back( cv::Point2f(width, height) );
    origPoints.push_back( cv::Point2f(width/2+30, 140) );
    origPoints.push_back( cv::Point2f(width/2-50, 140) );
    std::vector<cv::Point2f> dstPoints;
    dstPoints.push_back( cv::Point2f(0, height) );
    dstPoints.push_back( cv::Point2f(width, height) );
    dstPoints.push_back( cv::Point2f(width, 0) );
    dstPoints.push_back( cv::Point2f(0, 0) );
    cv::Mat inputImg, outputImg;
    // IPM object
    IPM ipm( cv::Size(width, height), cv::Size(width, height), origPoints, dstPoints );
     ipm.applyHomography( inputImg, outputImg );
  }
  bool getCAMINFO() {

      cv::FileStorage fs;
      cv::Matx33d     K;
      cv::Mat         D;

      if (fs.open(fisheye_camera_info_url_, cv::FileStorage::READ) ) {

          fs["camera_matrix"] >> K_;
          fs["distortion_coefficients"] >> D_;

          fs.release();

          return true;
      } else {
          return false;
      }

  }

  bool calib_fisheye() {
    cv::Mat temp, process_img_;
          cv::Matx33d newK = K_;

          cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_, D_, temp.size(), cv::noArray(), newK, balance_);
          cv::fisheye::undistortImage(temp, process_img_, K_, D_, newK);

  }
};
 int main (int argc, char** argv)
 {
   ros::init(argc, argv, "road_mark_detector_node");
   ros::NodeHandle nh;
   road_mark_detector_node node(nh);
   return 0;
 }
