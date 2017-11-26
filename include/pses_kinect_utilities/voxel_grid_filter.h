/**
 * @file "pses_utilities/voxel_grid_filter.h"
 * @brief Contains the VoxelGridFilterNodelet class, its member variables and
 *callback functions.
 *
*/

#ifndef VOXEL_GRID_FILTER_H
#define VOXEL_GRID_FILTER_H

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pses_kinect_utilities/VoxelGridFilterConfig.h>
#include <dynamic_reconfigure/server.h>

/**
 * @typedef pcl::PointCloud<pcl::PointXYZ> PointCloud
 * @brief Shortcut for a XYZ Point cloud of the pcl library.
*/
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * @namespace pses_kinect_utilities
 * @brief This namespace is used by the nodelets inside our package
 *pses_kinect_utilities.
 *
*/
namespace pses_kinect_utilities
{

/**
 * @class VoxelGridFilterNodelet voxel_grid_filter.h
 * @brief Class of the voxel grid filter nodelet
 *
*/
class VoxelGridFilterNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_; /**< ROS node handle. */
  int queue_size_; /**< Attribute to store the desired queue size for the depth
                      image subscriber. */
  std::
      string tf_frame_; /**< Attribute to store the tf frame id that has to be
                           assigned to the output point cloud. */
  PointCloud::Ptr
      filtered_cloud_; /**< Attribute to store the filtered point cloud. */
  VoxelGridFilterConfig config_; /**< Configuration of the voxel grid filter
                                    that is updated and set up by dynamic
                                    reconfigure. */
  boost::shared_ptr<dynamic_reconfigure::Server<VoxelGridFilterConfig>>
      reconfigureServer; /**< Dynamic reconfigure server. */
  // Subscriptions
  ros::Subscriber sub_cloud_; /**< Subscriber of the input point cloud. */
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_cloud_; /**< Publisher of the filtered point cloud. */

  /**
   * @brief This function is called by the initialization of the nodelet and
   * acts as a constructor.
  */
  virtual void onInit();

  /**
   * @brief Callback function that is called whenever anyone is subscribed to
   * the output point cloud.
  */
  void connectCb();

  /**
   * @brief Callback function that is called whenever a new point cloud is
   * received.
   * @param[in] cloud_msg Contains the received point cloud.
  */
  void pointCloudCb(const PointCloud::ConstPtr& cloud_msg);

  /**
   * @brief Callback function that is called whenever the filter configuration
   * is updated or set up through dynamic reconfigure.
   * @param[in] inputConfig New filter configuration.
   * @param[in] level This parameter is not used in the implementation of this
   * callback. Hence it can be ignored.
  */
  void dynReconfCb(VoxelGridFilterConfig& inputConfig, uint32_t level);
};
}

#endif // VOXEL_GRID_FILTER_H
