#include <yolo_object_detector.h>
#include <fstream>

yolo_object_detector::yolo_object_detector():it_(nh_)
{
  config_path_ = ros::package::getPath("opencv-yolo")+"/config/tiny-yolo-voc.cfg";
  binary_path_ = ros::package::getPath("opencv-yolo")+"/config/tiny-yolo-voc.weights";
  class_names_file_path_ = ros::package::getPath("opencv-yolo")+"/config/voc.names";
  yolo_net_ = cv::dnn::readNetFromDarknet(config_path_, binary_path_);
  std::ifstream class_names_file;
  class_names_file.open(class_names_file_path_.c_str());
  //parse class names and put into class_names_ vector
  if (class_names_file.is_open())
  {
    std::string class_name = "";
    while (std::getline(class_names_file, class_name))
    {
      class_names_.push_back(class_name);
    }
  }
  if(yolo_net_.empty())
  {
    ROS_ERROR_STREAM("failed to load yolo");
    exit(-1);
  }
  image_pub_ = it_.advertise(ros::this_node::getName()+"/detected_image", 1);
  image_sub_ = it_.subscribe(ros::this_node::getName()+"/image_raw", 10, &yolo_object_detector::image_callback, this);
}

yolo_object_detector::~yolo_object_detector()
{
}

void yolo_object_detector::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat input_blob = cv::dnn::blobFromImage(cv_ptr->image, 1 / 255.F, cv::Size(416, 416), cv::Scalar(), true, false);
  yolo_net_.setInput(input_blob, "data");
  cv::Mat detection_mat = yolo_net_.forward("detection_out");
  for (int i = 0; i < detection_mat.rows; i++)
  {
    const int probability_index = 5;
    const int probability_size = detection_mat.cols - probability_index;
    float *prob_array_ptr = &detection_mat.at<float>(i, probability_index);
    size_t object_class = std::max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
    float confidence = detection_mat.at<float>(i, (int)object_class + probability_index);
    if (confidence > 0.5)
    {
      float x = detection_mat.at<float>(i, 0);
      float y = detection_mat.at<float>(i, 1);
      float width = detection_mat.at<float>(i, 2);
      float height = detection_mat.at<float>(i, 3);
      int xLeftBottom = static_cast<int>((x - width / 2) * cv_ptr->image.cols);
      int yLeftBottom = static_cast<int>((y - height / 2) * cv_ptr->image.rows);
      int xRightTop = static_cast<int>((x + width / 2) * cv_ptr->image.cols);
      int yRightTop = static_cast<int>((y + height / 2) * cv_ptr->image.rows);

      cv::Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);
      cv::rectangle(cv_ptr->image, object, cv::Scalar(0, 255, 0));

      if (object_class < class_names_.size())
      {
        std::ostringstream ss;
        ss.str("");
        ss << confidence;
        cv::String conf(ss.str());
        cv::String label = cv::String(class_names_[object_class]) + ": " + conf;
        int baseLine = 0;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::rectangle(cv_ptr->image, cv::Rect(cv::Point(xLeftBottom, yLeftBottom ), cv::Size(labelSize.width, labelSize.height + baseLine)), cv::Scalar(255, 255, 255), CV_FILLED);
        cv::putText(cv_ptr->image, label, cv::Point(xLeftBottom, yLeftBottom+labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
      }
      else
      {
        ROS_INFO_STREAM("Class: " << object_class << "Confidence: " << confidence << " " <<xLeftBottom << " " << yLeftBottom << " " << xRightTop << " " << yRightTop);
      }
    }
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}
