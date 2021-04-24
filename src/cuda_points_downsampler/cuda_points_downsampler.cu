#include <cuda_points_downsampler/cuda_points_downsampler.h>
#include "pcl_conversions/pcl_conversions.h"
#include "cuda-filter/cudaFilter.h"

namespace gpuac
{
CudaDownSampler::CudaDownSampler(ros::NodeHandle& nh)
{
    m_private_nh = nh;
    init();
}

CudaDownSampler::~CudaDownSampler()
{
    m_detect_cuda_thread_enable = false;
    m_detect_cuda_thread->join();
    ROS_INFO("cuda_test_normal threads joined");
}

void CudaDownSampler::init()
{
    ROS_INFO_STREAM("CudaDownSampler initializeLaunchParams...");
    initializeLaunchParams();
    ROS_INFO_STREAM("CudaDownSampler initializeSubPub...");
    initializeSubPub();
    ROS_INFO_STREAM("CudaDownSampler initializeThread...");
    initializeThread();
}

void CudaDownSampler::initializeThread()
{
    m_detect_cuda_thread_enable = true;
    m_detect_cuda_thread = std::make_shared<std::thread>(std::bind(&CudaDownSampler::detect_cuda_thread, this));
}

void CudaDownSampler::initializeSubPub()
{
    m_points_sub = m_private_nh.subscribe("/rslidar_points", 10, &CudaDownSampler::points_callback, this);
    // 初始化消息发布, 算法流程发布
    m_voxel_cuda_pub = m_private_nh.advertise<sensor_msgs::PointCloud2>("/gpuac/voxelgrid/output", 1);

}

void CudaDownSampler::initializeLaunchParams()
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

inline void testCUDA(pcl::PointCloud<PointT>::Ptr& cloudSrc, pcl::PointCloud<PointT>::Ptr& cloudDst, const float& downsample_res)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    t1 = std::chrono::steady_clock::now();
    cudaStream_t stream = NULL;
    cudaStreamCreate(&stream);
    t2 = std::chrono::steady_clock::now();
    auto time_span0 = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

    unsigned int nCount = cloudSrc->width * cloudSrc->height;
    float* inputData = (float*)cloudSrc->points.data();

    // cloudDst->width = nCount;
    // cloudDst->height = 1;
    // cloudDst->resize(cloudDst->width * cloudDst->height);
    // float* outputData = (float*)cloudDst->points.data();
    // memset(outputData, 0, sizeof(float) * 4 * nCount);

    // std::cout << "\n------------checking CUDA ---------------- " << std::endl;
    // std::cout << "CUDA Loaded " << cloudSrc->width * cloudSrc->height
            //   << " data points from PCD file with the following fields: " << pcl::getFieldsList(*cloudSrc) << std::endl;

    float* input = NULL;
    t1 = std::chrono::steady_clock::now();
    cudaMallocManaged(&input, sizeof(float) * 4 * nCount, cudaMemAttachHost);
    cudaStreamAttachMemAsync(stream, input);
    cudaMemcpyAsync(input, inputData, sizeof(float) * 4 * nCount, cudaMemcpyHostToDevice, stream);
    cudaStreamSynchronize(stream);

    float* output = NULL;
    cudaMallocManaged(&output, sizeof(float) * 4 * nCount, cudaMemAttachHost);
    cudaStreamAttachMemAsync(stream, output);
    cudaStreamSynchronize(stream);
    t2 = std::chrono::steady_clock::now();
    auto time_span1 = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

    cudaFilter filterTest(stream);
    FilterParam_t setP;
    FilterType_t type;
    {
        unsigned int countLeft = 0;
        // std::cout << "\n------------checking CUDA VoxelGrid---------------- " << std::endl;

        // memset(outputData, 0, sizeof(float) * 4 * nCount);

        type = VOXELGRID;

        setP.type = type;
        setP.voxelX = downsample_res;
        setP.voxelY = downsample_res;
        setP.voxelZ = downsample_res;

        filterTest.set(setP);
        int status = 0;
        cudaDeviceSynchronize();
        t1 = std::chrono::steady_clock::now();
        status = filterTest.filter(output, &countLeft, input, nCount);
        cudaDeviceSynchronize();
        t2 = std::chrono::steady_clock::now();

        if (status != 0)
            return;
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
        std::cout << "CUDA cudaStreamCreate by Time: " << time_span0.count() << " ms." << std::endl;
        std::cout << "CUDA MemCpy by Time: " << time_span1.count() << " ms." << std::endl;
        std::cout << "CUDA VoxelGrid by Time: " << time_span.count() << " ms." << std::endl;
        // std::cout << "CUDA VoxelGrid before filtering: " << nCount << std::endl;
        // std::cout << "CUDA VoxelGrid after filtering: " << countLeft << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNew(new pcl::PointCloud<pcl::PointXYZ>);
        cloudNew->width = countLeft;
        cloudNew->height = 1;
        cloudNew->points.resize(cloudNew->width * cloudNew->height);

        for (std::size_t i = 0; i < cloudNew->size(); ++i)
        {
            cloudNew->points[i].x = output[i * 4 + 0];
            cloudNew->points[i].y = output[i * 4 + 1];
            cloudNew->points[i].z = output[i * 4 + 2];
        }
        cloudDst = cloudNew;
        // pcl::io::savePCDFileASCII("after-cuda-VoxelGrid.pcd", *cloudNew);
    }

    t1 = std::chrono::steady_clock::now();
    cudaFree(input);
    cudaFree(output);
    t2 = std::chrono::steady_clock::now();
    auto time_span3 = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

    // todo: stream as a member
    t1 = std::chrono::steady_clock::now();
    cudaStreamDestroy(stream);
    t2 = std::chrono::steady_clock::now();
    auto time_span4 = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

    std::cout << "CUDA cudaFree by Time: " << time_span3.count() << " ms." << std::endl;
    std::cout << "CUDA cudaStreamDestroy by Time: " << time_span4.count() << " ms." << std::endl;
}

// 内存分配函数
// 输入点云和流，输出绑定了流的内存指针
void ros2Cudapcl(pcl::PointCloud<PointT>::Ptr& point_cloud, cudaStream_t* pstream, float** pinput_d, float** poutput_d)
{

    unsigned int nCount = point_cloud->width * point_cloud->height;
    float* inputData = (float*)point_cloud->points.data();

    auto t1 = std::chrono::steady_clock::now();
    cudaMallocManaged(pinput_d, sizeof(float) * 4 * nCount, cudaMemAttachHost);
    cudaStreamAttachMemAsync(*pstream, *pinput_d);
    cudaMemcpyAsync(*pinput_d, inputData, sizeof(float) * 4 * nCount, cudaMemcpyHostToDevice, *pstream);
    cudaStreamSynchronize(*pstream);

    cudaMallocManaged(poutput_d, sizeof(float) * 4 * nCount, cudaMemAttachHost);
    cudaStreamAttachMemAsync(*pstream, *poutput_d);
    cudaStreamSynchronize(*pstream);
    auto t2 = std::chrono::steady_clock::now();
    auto time_span1 = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
    std::cout << "CUDA ros2Cudapcl by Time: " << time_span1.count() << " ms." << std::endl;
}

// 内存读取函数
// 输入GPU内存，输出点云指针
// todo:
void cudapcl2Ros(float** poutput_d, unsigned int& countLeft, pcl::PointCloud<PointT>::Ptr& point_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNew(new pcl::PointCloud<pcl::PointXYZ>);
    cloudNew->width = countLeft;
    cloudNew->height = 1;
    cloudNew->points.resize(cloudNew->width * cloudNew->height);
    for (std::size_t i = 0; i < cloudNew->size(); ++i)
    {
        cloudNew->points[i].x = *poutput_d[i * 4 + 0];
        cloudNew->points[i].y = *poutput_d[i * 4 + 1];
        cloudNew->points[i].z = *poutput_d[i * 4 + 2];
    }
    point_cloud = cloudNew;
}

void CudaDownSampler::detect_cuda_thread()
{
    while (m_detect_cuda_thread_enable)  // && m_ndt_observed_pose_pub.getNumSubscribers()
    {
        std::unique_lock<std::mutex> detect_cuda_lock(m_detect_cuda_mutex);
        m_detect_cuda_cond.wait(detect_cuda_lock);
        detect_cuda_lock.unlock();

        ROS_INFO_ONCE("detect_cuda_thread working now!");

        if (m_points_cuda_cloud->empty())
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

        ROS_INFO("m_detect_cuda_thread-PointCloud size is %d .", m_points_cuda_cloud->size());

        // 体素滤波
        t1 = ros::WallTime::now();
        if(m_is_downsample)
        {
            if(m_is_use_gpu)
            {
                // cuda-pcl
                // testCUDA(m_points_cuda_cloud, filtered_cloud_ptr, m_downsample_res);
                // todo: stream as a class member
                auto t1 = std::chrono::steady_clock::now();
                cudaStream_t stream = NULL;
                cudaStreamCreate(&stream);
                auto t2 = std::chrono::steady_clock::now();
                auto time_span0 =
                    std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
                std::cout << "CUDA cudaStreamCreate by Time: " << time_span0.count() << " ms." << std::endl;

                float* input_d = NULL;
                float* output_d = NULL;

                // unsigned int nCount = m_points_cuda_cloud->width * m_points_cuda_cloud->height;
                // float* inputData = (float*)m_points_cuda_cloud->points.data();
                // input_d = NULL;
                // t1 = std::chrono::steady_clock::now();
                // cudaMallocManaged(&input_d, sizeof(float) * 4 * nCount, cudaMemAttachHost);
                // cudaStreamAttachMemAsync(stream, input_d);
                // cudaMemcpyAsync(input_d, inputData, sizeof(float) * 4 * nCount, cudaMemcpyHostToDevice, stream);
                // cudaStreamSynchronize(stream);
                // output_d = NULL;
                // cudaMallocManaged(&output_d, sizeof(float) * 4 * nCount, cudaMemAttachHost);
                // cudaStreamAttachMemAsync(stream, output_d);
                // cudaStreamSynchronize(stream);
                // t2 = std::chrono::steady_clock::now();
                // auto time_span1 =
                //     std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
                // std::cout << "CUDA ros2Cudapcl by Time: " << time_span1.count() << " ms." << std::endl;

                ros2Cudapcl(m_points_cuda_cloud, &stream, &input_d, &output_d);

                // todo: filterTest as a class member
                cudaFilter filterTest(stream);
                FilterParam_t setP;
                FilterType_t type;
                unsigned int countLeft = 0;
                type = VOXELGRID;

                setP.type = type;
                setP.voxelX = m_downsample_res;
                setP.voxelY = m_downsample_res;
                setP.voxelZ = m_downsample_res;

                filterTest.set(setP);
                int status = 0;
                cudaDeviceSynchronize();
                t1 = std::chrono::steady_clock::now();
                status = filterTest.filter(output_d, &countLeft, input_d,
                                           m_points_cuda_cloud->width * m_points_cuda_cloud->height);
                cudaDeviceSynchronize();
                t2 = std::chrono::steady_clock::now();

                if (status != 0)
                    return;
                auto time_span =
                    std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
                std::cout << "CUDA VoxelGrid by Time: " << time_span.count() << " ms." << std::endl;

                t1 = std::chrono::steady_clock::now();
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNew(new pcl::PointCloud<pcl::PointXYZ>);
                cloudNew->width = countLeft;
                cloudNew->height = 1;
                cloudNew->points.resize(cloudNew->width * cloudNew->height);
                for (std::size_t i = 0; i < cloudNew->size(); ++i)
                {
                    cloudNew->points[i].x = output_d[i * 4 + 0];
                    cloudNew->points[i].y = output_d[i * 4 + 1];
                    cloudNew->points[i].z = output_d[i * 4 + 2];
                }
                filtered_cloud_ptr = cloudNew;
                // cudapcl2Ros(&input_d, countLeft, filtered_cloud_ptr);
                t2 = std::chrono::steady_clock::now();
                auto time_span3 =
                    std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
                std::cout << "CUDA Cudapcl2Ros by Time: " << time_span3.count() << " ms." << std::endl;

                cudaFree(input_d);
                cudaFree(output_d);
                cudaStreamDestroy(stream);
            }
            else
            {
                // pcl
                voxelFilter(m_points_cuda_cloud, m_downsample_res);
                filtered_cloud_ptr = m_points_cuda_cloud;
            }
        }
        else
        {
            filtered_cloud_ptr = m_points_cuda_cloud;
        }
        t2 = ros::WallTime::now();
        ROS_INFO_STREAM("detect_cuda_thread voxelFilter_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
        ROS_INFO("m_detect_cuda_thread-After Voxel size is %d .", filtered_cloud_ptr->size());

        end = ros::WallTime::now();
        ROS_INFO_STREAM("detect_cuda_thread processing_time: " << (end - start).toSec() * 1000.0 << "[ms]");

        if(m_is_pub_pc)
        {
            t1 = ros::WallTime::now();
            pcl::toROSMsg(*filtered_cloud_ptr, m_pub_cuda_pc);
            m_pub_cuda_pc.header.stamp = ros::Time::now();
            m_pub_cuda_pc.header.frame_id = "/gpuac/pointcloud_frame";
            m_voxel_cuda_pub.publish(m_pub_cuda_pc);
            t2 = ros::WallTime::now();
            ROS_INFO_STREAM("detect_cuda_thread toROSMsg time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");
        }
        ROS_INFO_STREAM("################END#####################################");
    }
}

void CudaDownSampler::points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    ROS_INFO_STREAM_ONCE("points_callback");

    // std::lock_guard<std::mutex> detect_pcl_guard(m_detect_pcl_mutex);
    std::lock_guard<std::mutex> detect_cuda_guard(m_detect_cuda_mutex);

    auto t1 = ros::WallTime::now();
    // 新来的数据赋值给成员
    pcl::fromROSMsg(*points_msg, *m_points_cuda_cloud);
    // cuda_conversions::rosToCupoch(points_msg, m_points_cuda_cloud);
    auto t2 = ros::WallTime::now();
    ROS_INFO_STREAM("################START####################################");
    ROS_INFO_STREAM("detect_cuda_thread pcl::fromROSMsg time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");

    // 通知线程可以处理了
    // m_detect_pcl_cond.notify_one();
    m_detect_cuda_cond.notify_one();

    ROS_INFO_STREAM_ONCE("points_callback end");
}

}  // namespace gpuac