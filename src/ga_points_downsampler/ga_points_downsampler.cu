#include <ga_points_downsampler/ga_points_downsampler.h>
#include "cupoch_conversions/cupoch_conversions.h"
// #include "open3d_conversions/open3d_conversions.h"

namespace gpuac
{
CupochDownSampler::CupochDownSampler(ros::NodeHandle& nh)
{
    m_private_nh = nh;
    init();
}

CupochDownSampler::~CupochDownSampler()
{
    m_detect_cupoch_thread_enable = false;
    m_detect_cupoch_thread->join();
    ROS_INFO("cupoch_test_normal threads joined");
}

void CupochDownSampler::init()
{
    ROS_INFO_STREAM("CupochDownSampler initializeLaunchParams...");
    initializeLaunchParams();
    ROS_INFO_STREAM("CupochDownSampler initializeSubPub...");
    initializeSubPub();
    ROS_INFO_STREAM("CupochDownSampler initializeThread...");
    initializeThread();
}

void CupochDownSampler::initializeThread()
{
    m_detect_cupoch_thread_enable = true;
    m_detect_cupoch_thread = std::make_shared<std::thread>(std::bind(&CupochDownSampler::detect_cupoch_thread, this));
}

void CupochDownSampler::initializeSubPub()
{
    m_points_sub = m_private_nh.subscribe("/rslidar_points", 10, &CupochDownSampler::points_callback, this);
    // 初始化消息发布, 算法流程发布
    m_voxel_cupoch_pub = m_private_nh.advertise<sensor_msgs::PointCloud2>("/cupoch/voxelgrid/output", 1);

}

void CupochDownSampler::initializeLaunchParams()
{
    ROS_INFO_STREAM("initialize_launch_params");
    // 下面相机
    m_private_nh.param("is_downsample", m_is_downsample, true);
    m_private_nh.param("is_pub_pc", m_is_pub_pc, false);
    m_private_nh.param("is_use_gpu", m_is_use_gpu, true);
    // m_private_nh.param("is_use_open3d", m_is_use_open3d, false);
    m_private_nh.param("downsample_res", m_downsample_res, 0.01);
}

inline void voxelFilter(pcl::PointCloud<PointT>::Ptr& point_cloud, const float& voxelgridleafsize)
{
    pcl::VoxelGrid<PointT> sor_Vox;                                                //创建体素滤波器
    sor_Vox.setInputCloud(point_cloud);                                            //设置点云输入
    sor_Vox.setLeafSize(voxelgridleafsize, voxelgridleafsize, voxelgridleafsize);  //设置滤波的体素大小，0.05m立方体
    sor_Vox.filter(*point_cloud);
}

void CupochDownSampler::detect_cupoch_thread()
{
    while (m_detect_cupoch_thread_enable)  // && m_ndt_observed_pose_pub.getNumSubscribers()
    {
        std::unique_lock<std::mutex> detect_cupoch_lock(m_detect_cupoch_mutex);
        m_detect_cupoch_cond.wait(detect_cupoch_lock);
        detect_cupoch_lock.unlock();

        ROS_INFO_ONCE("detect_cupoch_thread working now!");

        if(m_is_use_gpu)
        {
            if(m_is_use_open3d)
            {
                // if (!m_points_open3d_cloud->HasPoints())
                {
                    // ROS_ERROR("m_points_open3d_cloud is empty!!");
                    return;
                }
                // ROS_INFO("m_detect_open3d_thread-PointCloud size is %d .", m_points_open3d_cloud->points_.size());
            }
            else
            {
                if (!m_points_cupoch_cloud->HasPoints())
                {
                    ROS_ERROR("m_points_cupoch_cloud is empty!!");
                    return;
                }
                ROS_INFO("m_detect_cupoch_thread-PointCloud size is %d .", m_points_cupoch_cloud->points_.size());
            }
        }

        auto start = ros::WallTime::now();
        auto end = ros::WallTime::now();
        auto t1 = ros::WallTime::now();
        auto t2 = ros::WallTime::now();

        auto filtered_cupoch_cloud{std::make_shared<cupoch::geometry::PointCloud>()};
        // // auto filtered_open3d_cloud{std::make_shared<open3d::geometry::PointCloud>()};
        pcl::PointCloud<PointT>::Ptr filtered_pcl_ptr{ new pcl::PointCloud<PointT> };

        // 体素滤波
        t1 = ros::WallTime::now();
        if(m_is_downsample)
        {
            if(m_is_use_gpu)
            {
                if(m_is_use_open3d)
                {
                    // filtered_open3d_cloud = m_points_open3d_cloud->VoxelDownSample(m_downsample_res);
                    // ROS_INFO("m_detect_open3d_thread-After Voxel size is %d .", filtered_open3d_cloud->points_.size());
                }
                else
                {
                    filtered_cupoch_cloud = m_points_cupoch_cloud->VoxelDownSample(m_downsample_res);
                    ROS_INFO("m_detect_cupoch_thread-After Voxel size is %d .", filtered_cupoch_cloud->points_.size());
                }
            }
            else
            {
                // pcl
                voxelFilter(m_points_pcl_cloud, m_downsample_res);
                filtered_pcl_ptr = m_points_pcl_cloud;
                ROS_INFO("m_detect_pcl_thread-After Voxel size is %d .", filtered_pcl_ptr->size());
            }
        }
        else
        {
            if(m_is_use_gpu)
            {
                if(m_is_use_open3d)
                {
                    // filtered_open3d_cloud = m_points_open3d_cloud;
                    // ROS_INFO("m_detect_open3d_thread-After Voxel size is %d .", filtered_open3d_cloud->points_.size());
                }
                else
                {
                    filtered_cupoch_cloud = m_points_cupoch_cloud;
                    ROS_INFO("m_detect_cupoch_thread-After Voxel size is %d .", filtered_cupoch_cloud->points_.size());
                }
            }
            else
            {
                filtered_pcl_ptr = m_points_pcl_cloud;
            }
        }
        t2 = ros::WallTime::now();
        ROS_INFO_STREAM("detect_cupoch_thread voxelFilter_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
        end = ros::WallTime::now();
        ROS_INFO_STREAM("detect_cupoch_thread processing_time: " << (end - start).toSec() * 1000.0 << "[ms]");

        if(m_is_pub_pc)
        {
            t1 = ros::WallTime::now();
            if(m_is_use_gpu)
            {
                if(m_is_use_open3d)
                {
                    // open3d_conversions::open3dToRos(filtered_open3d_cloud, m_pub_cupoch_pc, "/gpuac/pointcloud_frame");
                }
                else
                {
                    cupoch_conversions::cupochToRos(filtered_cupoch_cloud, m_pub_cupoch_pc, "/gpuac/pointcloud_frame");
                }
            }
            else
            {
                pcl::toROSMsg(*filtered_pcl_ptr, m_pub_cupoch_pc);
            }
            m_pub_cupoch_pc.header.stamp = ros::Time::now();
            m_pub_cupoch_pc.header.frame_id = "/gpuac/pointcloud_frame";
            m_voxel_cupoch_pub.publish(m_pub_cupoch_pc);
            t2 = ros::WallTime::now();
            ROS_INFO_STREAM("detect_cupoch_thread cupochToRos time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
        }
        ROS_INFO_STREAM("################END#####################################");
    }
}

void CupochDownSampler::points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    ROS_INFO_STREAM_ONCE("points_callback");

    // std::lock_guard<std::mutex> detect_pcl_guard(m_detect_pcl_mutex);
    std::lock_guard<std::mutex> detect_cupoch_guard(m_detect_cupoch_mutex);
    // 新来的数据赋值给成员

    auto t1 = ros::WallTime::now();
    if(m_is_use_gpu)
    {
        if(m_is_use_open3d)
        {
            // m_points_open3d_cloud->Clear();
            // open3d_conversions::rosToOpen3d(points_msg, m_points_open3d_cloud);
        }
        else
        {
            cupoch_conversions::rosToCupoch(points_msg, m_points_cupoch_cloud);
        }
    }
    else
    {
        pcl::fromROSMsg(*points_msg, *m_points_pcl_cloud);
    }
    auto t2 = ros::WallTime::now();
    ROS_INFO_STREAM("################START####################################");
    ROS_INFO_STREAM("detect_cupoch_thread rosToCupoch time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");

    // 通知线程可以处理了
    m_detect_cupoch_cond.notify_one();

    ROS_INFO_STREAM_ONCE("points_callback end");
}

}  // namespace gpuac