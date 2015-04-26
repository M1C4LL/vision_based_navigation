#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Publisher rgbd_pointcloud_camera;
ros::Publisher rgbd_pointcloud_world;

//create Transform listener as a pointer (in order to be able to make it public)
//cannot create it as a transform listener directly, because this is only possible
//after ros:init()
//Important: TransformListener should to be instantiated as soon as possible to
//be able to start listening as soon as possible.
tf::TransformListener* ptr_tf_listener = NULL;

void callback(const sensor_msgs::ImageConstPtr& image_color_msg, const sensor_msgs::ImageConstPtr& image_depth_msg,
 const sensor_msgs::CameraInfoConstPtr& info_color_msg, const sensor_msgs::CameraInfoConstPtr& info_depth_msg)
{
  cout<< "Callback." << endl;
  cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
  cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_msg)->image;
  
  //ros::Rate loop_rate(10);
  //loop_rate.sleep();


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

  // Create point cloud 1
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_msg_camera(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_msg_camera->header = pcl_conversions::toPCL(image_depth_msg->header);

  pcl::PointXYZRGB pt;
  //do for all pixels
  for(int y = 0; y < image_color.rows; y += 1)
  {
    for(int x = 0; x < image_color.cols; x += 1)
    {
      //calculate the depth
      float depth = image_depth.at<short int>(cv::Point(x,y)) / 1.0; // or factor = 5000 (according to vision.in.tum.de)
      //use only valid points
      if(depth > 0)
      {
        //assign position
        pt.x = (x - cx) * depth / fx;
        pt.y = (y - cy) * depth / fy;
        pt.z = depth;
        //assign color
        pt.r = image_color.at<cv::Vec3b>(cv::Point(x,y))[0];
        pt.g = image_color.at<cv::Vec3b>(cv::Point(x,y))[1];
        pt.b = image_color.at<cv::Vec3b>(cv::Point(x,y))[2];
        pointcloud_msg_camera->points.push_back (pt);
      }
    }
  }
  pointcloud_msg_camera->height = 1;
  pointcloud_msg_camera->width = pointcloud_msg_camera->points.size();

  cout<< "PCL created" << endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_msg_world(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloud_msg_world->header = pcl_conversions::toPCL(image_depth_msg->header);

  try
  {
    cout<< "Try2." << endl;
    ptr_tf_listener->waitForTransform("/world", pcl_conversions::toPCL(image_depth_msg->header).frame_id, image_color_msg->header.stamp, ros::Duration(5.0));
    //ptr_tf_listener->waitForTransform("/world", "/kinect", now, ros::Duration(0.001));

    pcl_ros::transformPointCloud("/world", *pointcloud_msg_camera, *pointcloud_msg_world, *ptr_tf_listener);
    cout << "yeah, worked!!" << endl;
    // Create point cloud 2
  }
  catch (tf::TransformException &ex)
  {
    cout<< "Catch2." << endl;
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  rgbd_pointcloud_camera.publish(pointcloud_msg_camera);
  rgbd_pointcloud_world.publish(pointcloud_msg_world);
  cout<< "PCL transformed and published" << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reprojection");
  ptr_tf_listener = new (tf::TransformListener);

  ros::NodeHandle nh;

  cout << "image_reprojection node started." << endl;

  message_filters::Subscriber<sensor_msgs::Image> image_color_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "/camera/depth/image", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_color_sub(nh, "/camera/rgb/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_depth_sub(nh, "/camera/depth/camera_info", 1);
  //message_filters::Subscriber<tf::> tf_sub(nh, "/tf", 1);

  rgbd_pointcloud_world = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/rgbd_pointcloud_world", 5);
  rgbd_pointcloud_camera = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/rgbd_pointcloud_camera", 5);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> syncPolicies;
  message_filters::Synchronizer<syncPolicies> sync(syncPolicies(10), image_color_sub, image_depth_sub, camera_info_color_sub, camera_info_depth_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin(); // Run until interupted

  return 0;
}
