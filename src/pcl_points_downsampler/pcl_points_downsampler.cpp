#include <pcl_points_downsampler/pcl_points_downsampler.h>
#include "pcl_conversions/pcl_conversions.h"

namespace gpuac
{
PclDownSampler::PclDownSampler(ros::NodeHandle& nh)
{
    m_private_nh = nh;
    init();
}

PclDownSampler::~PclDownSampler()
{
    m_detect_pcl_thread_enable = false;
    m_detect_pcl_thread->join();
    ROS_INFO("pcl_test_normal threads joined");
}

void PclDownSampler::init()
{
    ROS_INFO_STREAM("PclDownSampler initializeLaunchParams...");
    initializeLaunchParams();
    ROS_INFO_STREAM("PclDownSampler initializeSubPub...");
    initializeSubPub();
    ROS_INFO_STREAM("PclDownSampler initializeThread...");
    initializeThread();
}

void PclDownSampler::initializeThread()
{
    m_detect_pcl_thread_enable = true;
    m_detect_pcl_thread = std::make_shared<std::thread>(std::bind(&PclDownSampler::detect_pcl_thread, this));
}

void PclDownSampler::initializeSubPub()
{
    m_points_sub = m_private_nh.subscribe("/rslidar_points", 10, &PclDownSampler::points_callback, this);
    // 初始化消息发布, 算法流程发布
    m_voxel_pcl_pub = m_private_nh.advertise<sensor_msgs::PointCloud2>("/gpuac/voxelgrid/output", 1);

}

void PclDownSampler::initializeLaunchParams()
{
    ROS_INFO_STREAM("initialize_launch_params");
    // 下面相机
    m_private_nh.param("is_downsample", m_is_downsample, true);
    m_private_nh.param("is_pub_pc", m_is_pub_pc, false);
    m_private_nh.param("is_use_gpu", m_is_use_gpu, true);
    m_private_nh.param("downsample_res", m_downsample_res, 0.1);
}

inline void voxelFilter(pcl::PointCloud<PointT>::Ptr& point_cloud, const float& voxelgridleafsize)
{
    pcl::VoxelGrid<PointT> sor_Vox;                                                //创建体素滤波器
    sor_Vox.setInputCloud(point_cloud);                                            //设置点云输入
    sor_Vox.setLeafSize(voxelgridleafsize, voxelgridleafsize, voxelgridleafsize);  //设置滤波的体素大小，0.05m立方体
    sor_Vox.filter(*point_cloud);
}

void PclDownSampler::detect_pcl_thread()
{
    while (m_detect_pcl_thread_enable)  // && m_ndt_observed_pose_pub.getNumSubscribers()
    {
        std::unique_lock<std::mutex> detect_pcl_lock(m_detect_pcl_mutex);
        m_detect_pcl_cond.wait(detect_pcl_lock);
        detect_pcl_lock.unlock();

        ROS_INFO_ONCE("detect_pcl_thread working now!");

        if (m_points_pcl_cloud->empty())
        {
            ROS_ERROR("input_cloud is empty!!");
            return;
        }
        auto start = ros::WallTime::now();
        auto end = ros::WallTime::now();
        auto t1 = ros::WallTime::now();
        auto t2 = ros::WallTime::now();

        // auto filtered_cloud_ptr{std::make_shared<geometry::PointCloud>()};
        pcl::PointCloud<PointT>::Ptr filtered_cloud_ptr{ new pcl::PointCloud<PointT> };

        ROS_INFO("m_detect_pcl_thread-PointCloud size is %d .", m_points_pcl_cloud->size());

        // 体素滤波
        t1 = ros::WallTime::now();
        if(m_is_downsample)
        {
            if(m_is_use_gpu)
            {
            }
            else
            {
                // pcl
                voxelFilter(m_points_pcl_cloud, m_downsample_res);
                filtered_cloud_ptr = m_points_pcl_cloud;
            }
        }
        else
        {
            filtered_cloud_ptr = m_points_pcl_cloud;
        }
        t2 = ros::WallTime::now();
        ROS_INFO_STREAM("detect_pcl_thread voxelFilter_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
        ROS_INFO("m_detect_pcl_thread-After Voxel size is %d .", filtered_cloud_ptr->size());

        end = ros::WallTime::now();
        ROS_INFO_STREAM("detect_pcl_thread processing_time: " << (end - start).toSec() * 1000.0 << "[ms]");

        if(m_is_pub_pc)
        {
            t1 = ros::WallTime::now();
            pcl::toROSMsg(*filtered_cloud_ptr, m_pub_pcl_pc);
            m_pub_pcl_pc.header.stamp = ros::Time::now();
            m_pub_pcl_pc.header.frame_id = "/gpuac/pointcloud_frame";
            m_voxel_pcl_pub.publish(m_pub_pcl_pc);
            t2 = ros::WallTime::now();
            ROS_INFO_STREAM("detect_pcl_thread toROSMsg time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
        }
        ROS_INFO_STREAM("################END#####################################");
    }
}

void PclDownSampler::points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    ROS_INFO_STREAM_ONCE("points_callback");

    // std::lock_guard<std::mutex> detect_pcl_guard(m_detect_pcl_mutex);
    std::lock_guard<std::mutex> detect_pcl_guard(m_detect_pcl_mutex);

    auto t1 = ros::WallTime::now();
    // 新来的数据赋值给成员
    pcl::fromROSMsg(*points_msg, *m_points_pcl_cloud);
    // pcl_conversions::rosToCupoch(points_msg, m_points_pcl_cloud);
    auto t2 = ros::WallTime::now();
    ROS_INFO_STREAM("################START####################################");
    ROS_INFO_STREAM("detect_pcl_thread pcl::fromROSMsg time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");

    // 通知线程可以处理了
    // m_detect_pcl_cond.notify_one();
    m_detect_pcl_cond.notify_one();

    ROS_INFO_STREAM_ONCE("points_callback end");
}

}  // namespace gpuac