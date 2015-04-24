#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace tf;
using namespace message_filters;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Publisher pub;

void callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg, const CameraInfoConstPtr& info_color_msg, const CameraInfoConstPtr& info_depth_msg)
{
  cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
  cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_msg)->image;
  
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  tf::StampedTransform transform;
  try
  {
  	ros::Time now = ros::Time::now();
    ros::Time past = now - ros::Duration(0.5);
    listener.waitForTransform("/world", "/kinect", now, ros::Duration(1.0));
    //listener.lookupTransform("/tf", now, "/tf", past, "/world", transform);
    listener.lookupTransform("/world", "/kinect", now, transform);
    cout << transform.getOrigin().y() << endl;
  }
  catch (tf::TransformException &ex) 
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

/*
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
  turtle_vel.publish(vel_msg);
*/

  rate.sleep();

  // Set ROS default camera intrinsics
  /*
  float fx = info_depth_msg->K[0];
  float fy = info_depth_msg->K[4];
  float cx = info_depth_msg->K[2];
  float cy = info_depth_msg->K[5];
  */
  float fx = 525.0;
  float fy = 525.0;
  float cx = 319.5;
  float cy = 239.5;

  // Create point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_msg(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_msg->header = pcl_conversions::toPCL(image_depth_msg->header);

  pcl::PointXYZRGB pt;
  for(int y = 0; y < image_color.rows; y += 1)
  {
    for(int x = 0; x < image_color.cols; x += 1)
    {
      float depth = image_depth.at<short int>(cv::Point(x,y)) / 1.0; // or factor = 5000 (according to vision.in.tum.de)
      if(depth > 0)
      {
        pt.x = (x - cx) * depth / fx;
	pt.y = (y - cy) * depth / fy;
	pt.z = depth;
	pt.r = image_color.at<cv::Vec3b>(cv::Point(x,y))[0];
	pt.g = image_color.at<cv::Vec3b>(cv::Point(x,y))[1];
	pt.b = image_color.at<cv::Vec3b>(cv::Point(x,y))[2];

	pointcloud_msg->points.push_back (pt);
      }
    }
  }
  pointcloud_msg->height = 1;
  pointcloud_msg->width = pointcloud_msg->points.size();
  pub.publish(pointcloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reprojection");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_color_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<Image> image_depth_sub(nh, "/camera/depth/image", 1);
  message_filters::Subscriber<CameraInfo> camera_info_color_sub(nh, "/camera/rgb/camera_info", 1);
  message_filters::Subscriber<CameraInfo> camera_info_depth_sub(nh, "/camera/depth/camera_info", 1);
  message_filters::Subscriber<tfMessage> tf_sub(nh, "/tf", 1);
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/rgbd_pointcloud", 1);

  typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, CameraInfo> syncPolicies;
  Synchronizer<syncPolicies> sync(syncPolicies(10), image_color_sub, image_depth_sub, camera_info_color_sub, camera_info_depth_sub);
  
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
