#include <ga_points_downsampler/ga_points_downsampler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ga_points_downsampler");
    ros::NodeHandle nh("~");

#ifndef __aarch64__
    cupoch::utility::InitializeAllocator(cupoch::utility::PoolAllocation, 1000000000);
    cudaSetDeviceFlags( cudaDeviceScheduleBlockingSync);
#else
    // use managed memory allocation to speed up memcpy
    // utility::InitializeAllocator();
    // cupoch::utility::InitializeAllocator(cupoch::utility::PoolAllocation, 1000000000);
    cupoch::utility::InitializeAllocator(cupoch::utility::CudaManagedMemory, 1000000000);
#endif

    gpuac::CupochDownSampler cupoch_downsampler(nh);

    // ros::MultiThreadedSpinner spinner(std::thread::hardware_concurrency());
    ros::MultiThreadedSpinner spinner(4);
    ros::spin(spinner);
    return 0;
}