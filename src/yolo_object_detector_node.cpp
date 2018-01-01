//headers in ros
#include <ros/ros.h>
#include <ros/package.h>

#include <yolo_object_detector.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_node");
  yolo_object_detector detector;
  ros::spin();
  return 0;
}
