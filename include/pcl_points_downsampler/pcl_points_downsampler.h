#ifndef PCL_CUDA_DOWNSAMPLER_H
#define PCL_CUDA_DOWNSAMPLER_H

// ros hdrs
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <dynamic_reconfigure/server.h>

// pcl hdrs
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

// std hdrs
#include <mutex>
#include <memory>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <condition_variable>

using namespace std;
using PointT = pcl::PointXYZ;

namespace gpuac
{
class PclDownSampler
{
  public:
    PclDownSampler(ros::NodeHandle & nh);
    ~PclDownSampler();

  private:
    // ini
    void init();
    void initializeLaunchParams();
    void initializeSubPub();
    void initializeThread();

    // sys fun
    void detect_pcl_thread();

    // ros interface
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg);

  private:
    // ros-com resource
    ros::NodeHandle m_private_nh;
    ros::Subscriber m_points_sub;
    // pubs
    ros::Publisher m_voxel_pcl_pub;
    // 消息发布及数据:
    sensor_msgs::PointCloud2 m_pub_pcl_pc;

    // sys res
    shared_ptr<std::thread> m_detect_pcl_thread;
    std::mutex m_detect_pcl_mutex;
    std::condition_variable m_detect_pcl_cond;
    volatile bool m_detect_pcl_thread_enable{false};

    // alg processor
    // std::shared_ptr<geometry::PointCloud> m_points_pcl_cloud{std::make_shared<geometry::PointCloud>()};
    pcl::PointCloud<PointT>::Ptr m_points_pcl_cloud{new pcl::PointCloud<PointT>};

    bool m_is_downsample{true};
    bool m_is_pub_pc{false};
    bool m_is_use_gpu{false};

    // downsample
    double m_downsample_res{0.01};

};  // class
}  // namespace gpuac
#endif