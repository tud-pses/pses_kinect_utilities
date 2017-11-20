/**
 * @file "kinect_filter.cpp"
 * @brief kinect_filter node. Applies a median filter to the kinect depth image
 * and after that a voxelgrid based filter to its point cloud. This node publishes
 * a topic with the filtered point cloud.
 *
*/

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
#include <opencv2/core/ocl.hpp>
#include <dynamic_reconfigure/server.h>
#include <pses_kinect_filter/KinectFilterConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;
typedef pses_kinect_filter::KinectFilterConfig FilterConfig;

// TODOs:
//     *Documentation
//     *Try it with navigation stack and evaluate performance

/**
 * @brief This function will be called whenever a node parameter is changed over dynamic reconfigure.
 * @param[in] inputConfig Reference to the object containing the new configuration of the filter.
 * @param[out] outConfig Pointer to the object where the new configuration of the filter has to be stored.
 * @param[in] level This parameter is not used by this callback and can be ignored.
*/
void dynReconfCallback(FilterConfig &inputConfig, FilterConfig* outConfig, uint32_t level) {
  *outConfig = inputConfig;
  // If an even kernel size was selected, add 1 to make it odd.
  if (outConfig->median_kernel_size % 2 == 0) outConfig->median_kernel_size += 1;
  ROS_INFO("Reconfigured kinect filter params");
}

/**
 * @brief This function will be called whenever a depth image is received over the corresponding ROS topic.
 * @param[in] rawImgPtr Pointer to the received depth image.
 * @param[out] procImg This parameter is not used by this callback and can be ignored.
 * @param[in] filterConfig Pointer to the object containing the configuration of the filter.
*/
void kinectCallback(const sensor_msgs::Image::ConstPtr& rawImgPtr, sensor_msgs::Image* rawImg, sensor_msgs::Image* procImg, FilterConfig* filterConfig){
  *rawImg = *rawImgPtr;
  cv_bridge::CvImagePtr cv_ptr;
    try{
       cv_ptr = cv_bridge::toCvCopy(*rawImg, rawImg->encoding);
      }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

      // Apply a median filter using the OpenCL libraries of OpenCV
      cv::medianBlur(cv_ptr->image.getUMat(cv::ACCESS_READ), cv_ptr->image.getUMat(cv::ACCESS_WRITE), filterConfig->median_kernel_size);

      // Convert the data back to a ROS Image and store it in procImg
      cv_bridge::CvImage cvi;
      cvi.header = rawImg->header;
      cvi.encoding = rawImg->encoding;
      cvi.image = cv_ptr->image;
      cvi.toImageMsg(*procImg);
}

/**
 * @brief This function will be called whenever a point cloud message is received over its corresponding ROS topic.
 * @param[in] rawPointCloud Pointer to the received point cloud.
 * @param[out] cloudFiltered Pointer to the object where the filtered point cloud has to be stored.
 * @param[in] filterConfig Pointer to the object containing the configuration of the filter.
 * @param[in] tf_frame Pointer to the string with the name of the tf frame of the point cloud.
*/
void pointCloudCallback(const PointCloud::ConstPtr& rawPointCloud, PointCloud::Ptr cloudFiltered, FilterConfig* filterConfig, std::string* tf_frame) {
  PointCloud::Ptr cloud (new PointCloud);
  *cloud = *rawPointCloud;
  pcl::VoxelGrid<PointXYZ> vox;
  vox.setInputCloud(cloud);
  // The leaf size is more or less the size of voxels. Note that these values affect what a good threshold value would be.
  vox.setLeafSize(filterConfig->leaf_size, filterConfig->leaf_size,filterConfig->leaf_size);
  // This limits the overall volume of points. Being "far away" points considered as irrelevant.
  vox.setFilterLimits(filterConfig->min_filter_limit, filterConfig->max_filter_limit);
  // The line below is perhaps the most important as it reduces ghost points.
  vox.setMinimumPointsNumberPerVoxel(filterConfig->min_points_per_voxel);
  vox.filter(*cloudFiltered);
  cloudFiltered->header.frame_id = *tf_frame;
}

/**
 * @brief This function will be once called after receving the first camera info message. Then the camera info
 * topic will be unsuscribed from this node.
 * @param[in] cameraInfo Pointer to the object containing the new camera info message.
 * @param[out] output Pointer to the object where the camera info has to be stored.
 * @param[in] infoSubscriber Pointer to the camera info subscriber.
*/
void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& cameraInfo, sensor_msgs::CameraInfo* output, ros::Subscriber* infoSubscriber) {
  *output = *cameraInfo;
  // Stop subscribing the camera info topic after the getting the first camera info message.
  infoSubscriber->shutdown();
}

int main(int argc, char **argv){

    // Init ROS
    ros::init(argc, argv, "kinect_filter");
    ros::NodeHandle nh;

    // Init image transport
    image_transport::ImageTransport it(nh);

    // Variables declarations/inits
    sensor_msgs::Image procImg;
    sensor_msgs::Image rawImg;
    sensor_msgs::CameraInfo cameraInfo;
    PointCloud::Ptr cloudFiltered (new PointCloud);
    std::string depth_image_topic;
    std::string camera_info_topic;
    std::string output_depth_image_topic;
    std::string tf_frame;
    FilterConfig filterConfig;

    // Init dynamic reconfigure
    dynamic_reconfigure::Server<FilterConfig> reconfigureServer;
    dynamic_reconfigure::Server<FilterConfig>::CallbackType f;
    f = boost::bind(&dynReconfCallback, _1, &filterConfig, _2);
    reconfigureServer.setCallback(f);

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
    ros::param::param<std::string>("~tf_frame", tf_frame, "base_link");

    ros::Subscriber kinectImg = nh.subscribe<sensor_msgs::Image>(depth_image_topic, 1, boost::bind(kinectCallback, _1, &rawImg, &procImg, &filterConfig));
    ros::Subscriber kinectInfo = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(infoCallback, _1, &cameraInfo, &kinectInfo));
    image_transport::CameraPublisher kinectDepthPub = it.advertiseCamera(output_depth_image_topic, 1);
    ros::Subscriber kinectCloud = nh.subscribe<PointCloud>("/kinect_filter/points", 1, boost::bind(pointCloudCallback, _1, cloudFiltered, &filterConfig, &tf_frame));
    ros::Publisher kinectCloudProc = nh.advertise<PointCloud> ("/kinect_filter/points_filtered", 1);


    ros::Rate loop_rate(35);
    while(ros::ok()) {

    kinectDepthPub.publish(procImg, cameraInfo, ros::Time::now());
    //kinectCloudProc.publish(cloudFiltered);
    ros::spinOnce();
    loop_rate.sleep();
}

    return 0;

}
