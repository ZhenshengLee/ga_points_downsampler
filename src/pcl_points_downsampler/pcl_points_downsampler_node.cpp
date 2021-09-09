#include <pcl_points_downsampler/pcl_points_downsampler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_points_downsampler");
    ros::NodeHandle nh("~");

    gpuac::PclDownSampler pcl_downsampler(nh);

    // ros::MultiThreadedSpinner spinner(std::thread::hardware_concurrency());
    ros::MultiThreadedSpinner spinner(4);
    ros::spin(spinner);
    return 0;
}