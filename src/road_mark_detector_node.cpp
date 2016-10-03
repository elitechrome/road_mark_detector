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
using namespace cv;
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
  cv::Mat         img;

public:
  road_mark_detector_node(ros::NodeHandle _nh):nh(_nh), it(_nh){
    nh.param("topic_name", paramTopicName, std::string("/stereo/right/image_raw"));
    image_sub = it.subscribe(paramTopicName, 1, &road_mark_detector_node::process, this);

    nh.param("camera_info_url", fisheye_camera_info_url_, std::string("/home/jaemin/clothoid_ws/src/road_mark_detector/camera.yml"));
    nh.param("balance", balance_, 0.6);
    if(!getCAMINFO()){
      ROS_ERROR("Couldn't get the camera infomation file on %s", fisheye_camera_info_url_.c_str());
      return;
    }
    ros::spin();
  }

  void process(const sensor_msgs::ImageConstPtr &msg){
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Fail to convert image.");
    }
    cv::Mat temp, undistorted_img_;
    cv::Matx33d newK = K_;
    img.copyTo(temp);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_, D_, temp.size(), cv::noArray(), newK, balance_);
    cv::fisheye::undistortImage(temp, undistorted_img_, K_, D_, newK);

    float width = img.cols, height = img.rows;

    std::vector<cv::Point2f> origPoints;
    //    origPoints.push_back( cv::Point2f(-120, height) );
    //    origPoints.push_back( cv::Point2f(width+120, height) );
    //    origPoints.push_back( cv::Point2f(width/2-5, 235) );
    //    origPoints.push_back( cv::Point2f(width/2-40, 235) );
    origPoints.push_back( cv::Point2f(-(3000+200), height) );
    origPoints.push_back( cv::Point2f(width+3000-200, height) );
    origPoints.push_back( cv::Point2f(width-(300+15), 235) );
    origPoints.push_back( cv::Point2f(300-15, 235) );

    std::vector<cv::Point2f> dstPoints;
    double scale = 2., ratio = 4;
    dstPoints.push_back( cv::Point2f(width/2-width/2*scale, height) );
    dstPoints.push_back( cv::Point2f(width/2+width/2*scale, height) );
    dstPoints.push_back( cv::Point2f(width/2+width/2*scale, height/2 - height/2*scale*ratio) );
    dstPoints.push_back( cv::Point2f(width/2-width/2*scale, height/2 - height/2*scale*ratio) );


    cv::Mat inputThresholdImg, inputThresholdImg_ROI, outputIPMImg, outputThresholdImg;
    cv::cvtColor(undistorted_img_, inputThresholdImg, CV_BGR2GRAY);
    inputThresholdImg_ROI = inputThresholdImg(cv::Rect(0, inputThresholdImg.rows/2, inputThresholdImg.cols, inputThresholdImg.rows/2));

    //equalizeHist( inputThresholdImg_ROI, inputThresholdImg_ROI );

    //Todo : Laplacian(Emboss filter)
    cv::threshold(inputThresholdImg, outputThresholdImg, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    cv::Mat hsv_image;
    cv::cvtColor(undistorted_img_, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat HSV_yellow_range;
    cv::inRange(hsv_image, cv::Scalar(10, 70, 0), cv::Scalar(35, 255, 255), HSV_yellow_range);

    IPM ipm( cv::Size(width, height), cv::Size(width, height), origPoints, dstPoints );
    ipm.applyHomography( outputThresholdImg, outputIPMImg );
    //ipm.drawPoints(origPoints, undistorted_img_ );

    cv::imshow("original", img);
    cv::imshow("undistort", undistorted_img_);
    cv::imshow("IPM", outputIPMImg);
    cv::imshow("Threshold", outputThresholdImg);
    cv::imshow("HSV_yellow_range", HSV_yellow_range);

    cv::waitKey(1);
  }

  bool getCAMINFO() {
    cv::FileStorage fs;

    if (fs.open(fisheye_camera_info_url_, cv::FileStorage::READ) ) {
      fs["camera_matrix"] >> K_;
      fs["distortion_coefficients"] >> D_;
      fs.release();
      return true;
    }
    else {
      return false;
    }
  }


};
int main (int argc, char** argv)
{
  ros::init(argc, argv, "road_mark_detector_node");
  ros::NodeHandle nh;
  ROS_INFO("road_mark_detector_node has started.");
  road_mark_detector_node node(nh);
  return 0;
}
