
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <image_transport/image_transport.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;

//TODO *Dynamic reconfigure for all params
//     *Documentation
//     *Try it with navigation stack and evaluate performance


void kinectCallback(const sensor_msgs::Image::ConstPtr& rawImgPtr, sensor_msgs::Image* rawImg, sensor_msgs::Image* procImg){
    *rawImg = *rawImgPtr;
    cv_bridge::CvImagePtr cv_ptr;
    try{
       cv_ptr = cv_bridge::toCvCopy(*rawImg, rawImg->encoding);
      }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

      cv::Mat imgIn = cv_ptr->image;
      cv::Mat imgIn_copy;
      imgIn.copyTo(imgIn_copy);
      cv::medianBlur(imgIn_copy, imgIn_copy, 5);

      cv_bridge::CvImage cvi;
      cvi.header = rawImg->header;
      cvi.encoding = rawImg->encoding;
      cvi.image = imgIn_copy;
      cvi.toImageMsg(*procImg);
}

void pointCloudCallback(const PointCloud::ConstPtr& rawPointCloud, PointCloud::Ptr cloudFiltered) {
  PointCloud::Ptr cloud (new PointCloud);
  *cloud = *rawPointCloud;
  pcl::VoxelGrid<PointXYZ> vox;
  vox.setInputCloud(cloud);
  // The leaf size is more or less the size of voxels. Note that these values affect what a good threshold value would be.
  vox.setLeafSize(0.05f, 0.05f, 0.05f);
  // This limits the overall volume of points. Being "far away" points considered as irrelevant.
  vox.setFilterLimits(-1.0, 3.0);
  // The line below is perhaps the most important as it reduces ghost points.
  vox.setMinimumPointsNumberPerVoxel(8);
  vox.filter(*cloudFiltered);
  cloudFiltered->header.frame_id = "base_footprint";
}

void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& cameraInfo, sensor_msgs::CameraInfo* output) {
  *output = *cameraInfo;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "kinect_filter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    sensor_msgs::Image procImg;
    sensor_msgs::Image rawImg;
    sensor_msgs::CameraInfo cameraInfo;

    PointCloud::Ptr cloudFiltered (new PointCloud);

    std::string depth_image_topic;
    std::string camera_info_topic;
    std::string output_depth_image_topic;

    if (!ros::param::get("~depth_image_topic", depth_image_topic))
    {
      ROS_ERROR("Depth image topic was not provided!");
      return -1;
    }

    if (!ros::param::get("~camera_info_topic", camera_info_topic))
    {
      ROS_ERROR("Camera_info topic was not provided!");
      return -1;
    }

    ros::param::param<std::string>("~output_depth_image_topic", output_depth_image_topic, "kinect2/depth_filtered");

    ros::Subscriber kinectImg = nh.subscribe<sensor_msgs::Image>(depth_image_topic, 1, boost::bind(kinectCallback, _1, &rawImg, &procImg));
    ros::Subscriber kinectInfo = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(infoCallback, _1, &cameraInfo));
    image_transport::CameraPublisher kinectDepthPub = it.advertiseCamera(output_depth_image_topic, 1);
    ros::Subscriber kinectCloud = nh.subscribe<PointCloud>("/points", 1, boost::bind(pointCloudCallback, _1, cloudFiltered));
    ros::Publisher kinectCloudProc = nh.advertise<PointCloud> ("/points_filtered", 1);


    ros::Rate loop_rate(35);
    while(ros::ok()) {

    kinectDepthPub.publish(procImg, cameraInfo, ros::Time::now());
    kinectCloudProc.publish(cloudFiltered);
    ros::spinOnce();
    loop_rate.sleep();
}

    return 0;

}
