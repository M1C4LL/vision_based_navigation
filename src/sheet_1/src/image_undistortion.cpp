#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

ros::Publisher pub;
image_geometry::PinholeCameraModel model;
bool once = true;  
  
void callback(const sensor_msgs::ImageConstPtr& image_color_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_color_msg)
{
  cv_bridge::CvImagePtr raw;
  cv_bridge::CvImagePtr undistorted;
  cv::Point2d center, topLeft, topRight, bottomLeft, bottomRight;
  cv::Point2d centerRect, topLeftRect, topRightRect, bottomLeftRect, bottomRightRect;
  center.x = 320; center.y = 240; 
  topLeft.x = 0; topLeft.y = 0;
  topRight.x = 640; topRight.y = 0;
  bottomLeft.x = 0; bottomLeft.y = 480;
  bottomRight.x = 640; bottomRight.y = 480;

  try
  {
    // convert the picture to cvImagePtr
    raw = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
    undistorted = raw;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Setup Camera Info
  sensor_msgs::CameraInfo camera_info_msgs(*camera_info_color_msg);
  // Set Camera Info from calibration dataset 1:
  // Distortion D
/*
  camera_info_msgs.D[0] = 0.193921;
  camera_info_msgs.D[1] = -0.384550;
  camera_info_msgs.D[2] = -0.005101;
  camera_info_msgs.D[3] = 0.003890;
  camera_info_msgs.D[4] = 0.0;
  // Rectification matrix (stereo cameras only)
  camera_info_msgs.R[0] = 1.0;
  camera_info_msgs.R[1] = 0.0;
  camera_info_msgs.R[2] = 0.0;
  camera_info_msgs.R[3] = 0.0;
  camera_info_msgs.R[4] = 1.0;
  camera_info_msgs.R[5] = 0.0;
  camera_info_msgs.R[6] = 0.0;
  camera_info_msgs.R[7] = 0.0;
  camera_info_msgs.R[8] = 1.0;
  // Projection/camera matrix
  camera_info_msgs.P[0] = 528.361816;
  camera_info_msgs.P[1] = 0.0;
  camera_info_msgs.P[2] = 322.476729;
  camera_info_msgs.P[3] = 0.0;
  camera_info_msgs.P[4] = 0.0;
  camera_info_msgs.P[5] = 531.181702;
  camera_info_msgs.P[6] = 252.636874;
  camera_info_msgs.P[7] = 0.0;
  camera_info_msgs.P[8] = 0.0;
  camera_info_msgs.P[9] = 0.0;
  camera_info_msgs.P[10] = 1.0;
  camera_info_msgs.P[11] = 0.0;
*/

  // Set Camera Info from calibration dataset 2:
  // Distortion D
  camera_info_msgs.D[0] = 0.199634;
  camera_info_msgs.D[1] = -0.449776;
  camera_info_msgs.D[2] = -0.003955;
  camera_info_msgs.D[3] = 0.000012;
  camera_info_msgs.D[4] = 0.0;
  // Rectification matrix (stereo cameras only)
  camera_info_msgs.R[0] = 1.0;
  camera_info_msgs.R[1] = 0.0;
  camera_info_msgs.R[2] = 0.0;
  camera_info_msgs.R[3] = 0.0;
  camera_info_msgs.R[4] = 1.0;
  camera_info_msgs.R[5] = 0.0;
  camera_info_msgs.R[6] = 0.0;
  camera_info_msgs.R[7] = 0.0;
  camera_info_msgs.R[8] = 1.0;
  // Projection/camera matrix
  camera_info_msgs.P[0] = 525.826599;
  camera_info_msgs.P[1] = 0.0;
  camera_info_msgs.P[2] = 325.653036;
  camera_info_msgs.P[3] = 0.0;
  camera_info_msgs.P[4] = 0.0;
  camera_info_msgs.P[5] = 531.369263;
  camera_info_msgs.P[6] = 247.685010;
  camera_info_msgs.P[7] = 0.0;
  camera_info_msgs.P[8] = 0.0;
  camera_info_msgs.P[9] = 0.0;
  camera_info_msgs.P[10] = 1.0;
  camera_info_msgs.P[11] = 0.0;
    
  // Set calibrated camera informaton
  model.fromCameraInfo(camera_info_msgs);

  // Rectify image
  model.rectifyImage(raw->image, undistorted->image, CV_INTER_LINEAR);

  // Rectify origin and corner points once
  if (once)
  {
    centerRect = model.rectifyPoint( center );
    topLeftRect = model.rectifyPoint( topLeft );
    topRightRect = model.rectifyPoint( topRight );
    bottomLeftRect = model.rectifyPoint( bottomLeft );
    bottomRightRect = model.rectifyPoint( bottomRight );
    std::cout << "center: " << center << " , centerRect" << centerRect << std::endl;
    std::cout << "topLeft: " << topLeft << " , topLeftRect" << topLeftRect << std::endl;
    std::cout << "topRight: " << topRight << " , topRightRect" << topRightRect << std::endl;
    std::cout << "bottomLeft: " << bottomLeft << " , bottomLeftRect" << bottomLeftRect << std::endl;
    std::cout << "bottomRight: " << bottomRight << " , bottomRightRect" << bottomRightRect << std::endl;
    once = false;
  }

  // Publish undistorted image
  pub.publish(undistorted->toImageMsg());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_undistortion");
  ros::NodeHandle nh;
  
  // filter image messages
  message_filters::Subscriber<sensor_msgs::Image> image_color_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_color_sub(nh, "/camera/rgb/camera_info", 1);

  // advertise undistorted image
  pub = nh.advertise<sensor_msgs::Image> ("/camera/rgb/image_color_undistorted", 1);

  // Use approximate time to sync messages
  // ApproximateTime takes a queue size as its constructor argument, hence syncPolicies(10)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicies;
  message_filters::Synchronizer<syncPolicies> sync(syncPolicies(10), image_color_sub, camera_info_color_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
