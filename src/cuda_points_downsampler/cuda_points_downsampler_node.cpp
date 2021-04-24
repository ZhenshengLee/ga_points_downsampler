#include <cuda_points_downsampler/cuda_points_downsampler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuda_points_downsampler");
    ros::NodeHandle nh("~");

    gpuac::CudaDownSampler cuda_downsampler(nh);

    // ros::MultiThreadedSpinner spinner(std::thread::hardware_concurrency());
    ros::MultiThreadedSpinner spinner(4);
    ros::spin(spinner);
    return 0;
}