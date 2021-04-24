#ifndef GA_CUPOCH_DOWNSAMPLER_CUPOCH_H
#define GA_CUPOCH_DOWNSAMPLER_CUPOCH_H

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

// cupoch hdrs
#include "cupoch/cupoch.h"
// open3d hdrs
// #include "open3d/Open3D.h"

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

// cuda hdrs
#include <cuda.h>
#include <cuda_runtime.h>

using namespace std;
using PointT = pcl::PointXYZ;

namespace gpuac
{
class CupochDownSampler
{
  public:
    CupochDownSampler(ros::NodeHandle & nh);
    ~CupochDownSampler();

  private:
    // ini
    void init();
    void initializeLaunchParams();
    void initializeSubPub();
    void initializeThread();

    // sys fun
    void detect_cupoch_thread();

    // ros interface
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg);

  private:
    // ros-com resource
    ros::NodeHandle m_private_nh;
    ros::Subscriber m_points_sub;
    // pubs
    ros::Publisher m_voxel_cupoch_pub;
    // 消息发布及数据:
    sensor_msgs::PointCloud2 m_pub_cupoch_pc;

    // sys res
    shared_ptr<std::thread> m_detect_cupoch_thread;
    std::mutex m_detect_cupoch_mutex;
    std::condition_variable m_detect_cupoch_cond;
    volatile bool m_detect_cupoch_thread_enable{false};

    // alg processor
    std::shared_ptr<cupoch::geometry::PointCloud> m_points_cupoch_cloud{std::make_shared<cupoch::geometry::PointCloud>()};
    // std::shared_ptr<open3d::geometry::PointCloud> m_points_open3d_cloud{std::make_shared<open3d::geometry::PointCloud>()};
    pcl::PointCloud<PointT>::Ptr m_points_pcl_cloud{new pcl::PointCloud<PointT>};

    bool m_is_downsample{true};
    bool m_is_pub_pc{false};
    bool m_is_use_gpu{true};
    bool m_is_use_open3d{false};

    // downsample
    double m_downsample_res{0.01};

};  // class
}  // namespace gpuac
#endif