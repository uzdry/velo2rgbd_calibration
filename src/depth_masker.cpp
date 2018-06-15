/*
  depth_masker: Mask a depth map according to an edge detected image
*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

class Masker
{
public:
  ros::NodeHandle nh_;

  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> mask_sub_;
  ros::Publisher masked_pub_;

  bool isfreeobs_;
  int edges_threshold_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExSync;
  message_filters::Synchronizer<ExSync> sync;

  Masker():
    nh_("~"),
    image_sub_(nh_, "image", 1),
    mask_sub_(nh_, "mask", 1),
    sync(ExSync(100), image_sub_, mask_sub_)
  {
    masked_pub_ = nh_.advertise<sensor_msgs::Image>("output", 1);

    nh_.param("isFreeobs", isfreeobs_, false);
    nh_.param("edges_threshold", edges_threshold_, 16);

    sync.registerCallback(boost::bind(&Masker::callback, this, _1, _2));

  }

  void callback(const sensor_msgs::ImageConstPtr& dpth, const sensor_msgs::ImageConstPtr& ma)
  {
    cv::Mat mask, binary_mask, depth32, depth;
    cv_bridge::CvImageConstPtr depth_ptr;

    try
    {
      depth_ptr = cv_bridge::toCvShare(dpth, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_ptr->image.convertTo(depth32, CV_32F, 1);
      mask = cv_bridge::toCvShare(ma, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("CvBridge failed: %s", e.what());
    }

    if (isfreeobs_){
      const static int OBSTACLE_LABEL = 32;
      cv::Mat obs_pattern(mask.rows, mask.cols, CV_8UC1, cv::Scalar(OBSTACLE_LABEL));
      cv::bitwise_and(mask, obs_pattern, binary_mask);
      binary_mask = binary_mask * (255.0/OBSTACLE_LABEL);
    }else{
      cv::threshold(mask, binary_mask, edges_threshold_, 255, 0);
    }

    cv::Mat result = cv::Mat(depth32.size(), CV_16UC1);
    depth32.copyTo(result, binary_mask);

    result.convertTo(depth, CV_16UC1);    

    cv_bridge::CvImage edges_depth;

    edges_depth.header   = dpth->header;
    edges_depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    edges_depth.image    = depth;

    sensor_msgs::ImagePtr edges_depth_msg = edges_depth.toImageMsg();

    masked_pub_.publish(edges_depth_msg);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_masker");

  Masker im;

  ROS_INFO("Ready");
  ros::spin();
}
