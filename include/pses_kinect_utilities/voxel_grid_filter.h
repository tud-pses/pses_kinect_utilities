#ifndef VOXEL_GRID_FILTER_H
#define VOXEL_GRID_FILTER_H

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pses_kinect_utilities/VoxelGridFilterConfig.h>
#include <dynamic_reconfigure/server.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace pses_kinect_utilities
{

class VoxelGridFilterNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  int queue_size_;
  std::string cloud_topic_;
  std::string output_topic_;
  std::string tf_frame_;
  PointCloud::Ptr filtered_cloud_;
  VoxelGridFilterConfig config_;
  boost::shared_ptr<dynamic_reconfigure::Server<VoxelGridFilterConfig>> reconfigureServer;
  // Subscriptions
  ros::Subscriber sub_cloud_;
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_cloud_;

  virtual void onInit();

  void connectCb();

  void pointCloudCb(const PointCloud::ConstPtr& cloud_msg);

  void dynReconfCallback(VoxelGridFilterConfig& inputConfig, uint32_t level);
};
}

#endif // VOXEL_GRID_FILTER_H
