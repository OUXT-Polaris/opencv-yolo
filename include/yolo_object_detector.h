#ifndef YOLO_OBJECT_DETECTOR_H_INCLUDED
#define YOLO_OBJECT_DETECTOR_H_INCLUDED

//headers in ros
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//headers in sensor_msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//headers in opencv
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>

class yolo_object_detector
{
public:
  yolo_object_detector();
  ~yolo_object_detector();
private:
  //config file path
  std::string config_path_;
  std::string binary_path_;
  std::string class_names_file_path_;
  //deep neural network and parameters
  cv::dnn::Net yolo_net_;
  std::vector<std::string> class_names_;
  //ros related members
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  void image_callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif  //YOLO_OBJECT_DETECTOR_H_INCLUDED
